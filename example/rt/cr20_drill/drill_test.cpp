/**
 * @file drill_test.cpp
 * @brief Drills one blind hole in the CR20 under Cartesian impedance control.
 *
 * This file is the part that talks to a real controller. The two pieces that
 * decide where the bit goes live elsewhere and are provable off-robot:
 *
 *   drill_frames.hpp   turns a scalar depth into a commanded pose
 *   drill_cycle.hpp    decides what that scalar should be
 *
 * Run drill_checks before every session. It exercises both on a laptop.
 *
 * HOW THE FORCE IS PRODUCED
 *
 * The controller behaves as a spring: it takes a POSITION command and pushes
 * with a force proportional to how far that command sits from where the tool
 * actually is. Thrust is therefore commanded indirectly, by deliberately
 * commanding a point ahead of the measured depth:
 *
 *     thrust = kStiffness[2] * lead
 *
 * The lead is also, exactly, how far the tool lunges if the bit breaks
 * through - so drill_cycle.hpp schedules it against depth rather than fixing
 * it. See that file's header.
 *
 * SCOPE AND LIMITS
 *
 *  - Blind holes only. A through-hole releases the stored lead at
 *    breakthrough; the depth-runaway abort catches it within a millimetre or
 *    so, but nothing here makes that a designed-for case.
 *  - No spindle control. The operator starts the drill by hand at the
 *    confirmation prompt.
 *  - setCollisionBehaviour is position-control only and does NOT apply in
 *    force control. The E-stop is the only hardware backstop during this run.
 *  - The sign convention of tauExt_inStiff on this controller is still
 *    unconfirmed, so every force comparison is on magnitude. The first
 *    contact run settles it: lead_mm * 3.0 should equal fz_n.
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <pthread.h>
#include <sys/stat.h>
#include <unistd.h>

#include "rokae/robot.h"
#include "rokae/utility.h"

#include "drill_cycle.hpp"
#include "drill_frames.hpp"

using namespace rokae;

using cr20_drill::AbortReason;
using cr20_drill::DrillConfig;
using cr20_drill::DrillCycle;
using cr20_drill::Feedback;
using cr20_drill::Phase;
using cr20_drill::Pose;

// ---------------------------------------------------------------------------
// Test parameters. For the commissioning ladder only the first two change -
// the lead cap alone flattens the schedule, so a shallow low-force run needs
// no other edits. See docs/drill-test-quickstart.md.
// ---------------------------------------------------------------------------

/// Hole depth, measured from the CONTACT DATUM rather than from the pose the
/// loop started in - the operator's standoff is eyeballed and must not come
/// out of the hole.
constexpr double kHoleDepthM = 0.055;

/// Ceiling on the commanded lead. THE safety dial: maximum thrust is
/// kLeadMaxM * kStiffness[2], and maximum lunge distance is kLeadMaxM.
constexpr double kLeadMaxM = 0.020;

/// Set false for a deliberate free-air stroke, where there is nothing to
/// touch. With it true the run refuses to leave spotting until the bit has
/// stalled against something.
constexpr bool kRequireContact = false;

/// [X, Y, Z, Rx, Ry, Rz] in the force-control (tool) frame.
///
/// Everything at the API ceiling (3000 N/m, 300 Nm/rad). The only compliance
/// this operation wants is axial, and that comes from the commanded LEAD, not
/// from a soft axis.
///
///  X/Y  3000 rather than the 200 an earlier version used. At 200 N/m the
///       13.7 N of lateral force measured during spotting is 68 mm of
///       deflection and the bit walks off the mark. At 3000 N/m it is 4.6 mm
///       - better, but still not rigid, so hole POSITION needs a pilot hole,
///       centre punch or drill guide. That is a fixturing answer, not a gains
///       answer.
/// Rx/Ry 300 rather than 150. Lateral force of 40 N at the 292 mm TCP offset
///       is an 11.7 Nm moment, which at 150 Nm/rad tilts 4.5 deg - right on
///       the axis-drift abort, so the run would trip on its own compliance.
///   Rz  300 to resist drill reaction torque (4.9 Nm mean, 9 Nm peak).
constexpr std::array<double, 6> kStiffness{3000, 3000, 3000, 300, 300, 300};

/// The tool and workobject to ACTIVATE, by their pendant names. robot.toolset()
/// only QUERIES whichever toolset the SDK session already has; with an
/// identity tool, tcpPose_m reports the FLANGE pose and "tool Z" is really
/// flange Z. That is the condition behind the run that pushed along a base
/// axis. A drill TCP existing on the pendant does not make it active here -
/// setToolset() does.
constexpr const char* kToolName = "g_tool_0";
constexpr const char* kWorkobjectName = "g_wobj_0";

/// What g_tool_0 is expected to contain on this arm. Checked rather than
/// printed-and-hoped: this is the assertion that catches an identity or
/// wrong toolset before anything is armed.
constexpr std::array<double, 3> kExpectedToolTransM{0.0202, 0.01476, 0.29149};
constexpr std::array<double, 3> kExpectedToolRpyRad{-0.017104, 0.558156, 0.259705};
constexpr double kExpectedToolMassKg = 2.714;
constexpr double kToolTransToleranceM = 0.002;
constexpr double kToolRpyToleranceRad = 0.02;
constexpr double kToolMassToleranceKg = 0.05;

/// How far the TCP may have moved between the plan being printed and the
/// operator confirming. Should be zero - the arm is powered and braked by
/// then - so this is a tripwire, not a tolerance to spend.
constexpr double kBaselineDriftToleranceM = 0.001;

constexpr int kRtThreadPriority = 80;
constexpr double kCyclePeriodSec = 0.001;

/// Wall-clock backstop, independent of anything the cycle believes.
constexpr double kWatchdogSec = 300.0;
constexpr double kWatchdogGraceSec = 30.0;

/// Depth-rate smoothing. One pole, ~50 ms, so a breakthrough still shows up
/// within tens of milliseconds while 1 kHz position quantisation does not.
constexpr double kDepthRateTimeConstantSec = 0.050;

/// How long to average the force residual for before the plan block. Long
/// enough that the standard error of the mean is well under a newton against
/// a ~5 N per-sample noise floor.
constexpr int kResidualWindowMs = 1000;

/// Trace buffer. Preallocated and never grown, so the RT callback does not
/// allocate; appends past capacity are dropped rather than reallocating.
constexpr std::size_t kTraceCapacity = 400000;

/// Every Nth captured cycle is written out. 1 kHz / 20 = 50 Hz, which is
/// plenty for the force and depth signals and keeps the file manageable.
/// Rows where a limit was binding or the phase changed are never dropped.
constexpr int kTraceDecimation = 20;

/// Bind-mounted from the host - see docker-compose.yml. An earlier attempt
/// used a repo-relative source path, which under a remote docker context is
/// resolved by the DAEMON, so it silently created a root-owned directory on
/// the robot PC and the logs went somewhere nobody could find. The mount is
/// an absolute host path now.
constexpr const char* kLogDir = "/logs";

namespace
{

std::atomic<bool> g_interrupt_requested{false};

void handleSignal(int signal)
{
    if (g_interrupt_requested.exchange(true))
    {
        // Second Ctrl+C: the operator has asked twice. Restore the default
        // and re-raise so the process actually dies even if the shaped
        // retract is stuck.
        std::signal(signal, SIG_DFL);
        std::raise(signal);
        return;
    }
}

/**
 * @class MotionModeGuard
 * @brief Returns the controller to Idle when it goes out of scope.
 *
 * MotionControlMode is controller-side state that OUTLIVES this process: it
 * survives disconnecting, and it survives setPowerState(false). Only a
 * controller restart clears it. So a run that exits while still in RtCommand
 * - whether it finished, faulted, or the operator pressed Ctrl+C at the
 * prompt - leaves the controller in RtCommand, and the NEXT run's task-space
 * calls (setToolset, calibrateForceSensor) fail with ec -18.
 *
 * main() has a dozen exit paths plus an exception path, so this is RAII
 * rather than a call at the end: the one arrangement that cannot be forgotten
 * on the path nobody thought about.
 *
 * Errors are swallowed deliberately - this runs during teardown, often while
 * another failure is already being reported, and there is nothing useful left
 * to do about it.
 */
class MotionModeGuard
{
  public:
    explicit MotionModeGuard(xMateRobot& robot) : robot_(&robot)
    {
    }

    MotionModeGuard(const MotionModeGuard&) = delete;
    MotionModeGuard& operator=(const MotionModeGuard&) = delete;

    ~MotionModeGuard()
    {
        std::error_code ec;
        robot_->setMotionControlMode(MotionControlMode::Idle, ec);
    }

  private:
    xMateRobot* robot_;
};

/// One captured control cycle.
struct Sample
{
    double t_s;
    double dt_ms;
    Phase phase;
    cr20_drill::Limit limit;
    double commanded_depth_m;
    double measured_depth_m;
    double hole_depth_m;
    double lead_m;
    double lateral_x_m;
    double lateral_y_m;
    double axis_drift_rad;
    std::array<double, 6> wrench;
    std::array<double, 6> joints;
};

/// Plain-language decode of a unit direction in the base frame, so the
/// operator can check it against what they can see without doing trigonometry
/// at the prompt.
std::string describeDirection(const std::array<double, 3>& unit)
{
    const double elevation_deg = std::asin(std::clamp(unit[2], -1.0, 1.0)) * 180.0 / M_PI;

    std::string vertical;
    if (elevation_deg > 80.0)
    {
        vertical = "straight UP";
    }
    else if (elevation_deg < -80.0)
    {
        vertical = "straight DOWN";
    }
    else if (std::abs(elevation_deg) < 10.0)
    {
        vertical = "roughly HORIZONTAL";
    }
    else
    {
        vertical = std::to_string(static_cast<int>(std::abs(elevation_deg))) + " deg " +
                   (elevation_deg > 0.0 ? "ABOVE" : "BELOW") + " horizontal";
    }

    std::string horizontal;
    if (std::abs(unit[0]) > std::abs(unit[1]))
    {
        horizontal = unit[0] > 0.0 ? "heading mostly +X" : "heading mostly -X";
    }
    else
    {
        horizontal = unit[1] > 0.0 ? "heading mostly +Y" : "heading mostly -Y";
    }

    return vertical + ", " + horizontal;
}

bool withinTolerance(double a, double b, double tolerance)
{
    return std::abs(a - b) <= tolerance;
}

std::string timestampedLogPath()
{
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);

    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm);

    return std::string{kLogDir} + "/drill-" + stamp + ".csv";
}

} // namespace

int main()
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::error_code ec;

    // -- connect ----------------------------------------------------------

    std::unique_ptr<xMateRobot> robot_ptr;
    try
    {
        robot_ptr = std::make_unique<xMateRobot>("192.168.2.160", "192.168.2.22");
    }
    catch (const std::exception& e)
    {
        std::cerr << "Connect failed: " << e.what() << std::endl;
        return 1;
    }
    xMateRobot& robot = *robot_ptr;
    std::cout << "Connected." << std::endl;

    // NOTE ON ORDERING: setMotionControlMode(RtCommand) is deliberately NOT
    // called yet. setToolset() and calibrateForceSensor() are non-real-time,
    // task-space operations, and the controller rejects those while it is in
    // RT command mode. So the sequence is: force the controller idle, do the
    // task-space setup, and only then switch to RT.
    robot.setRtNetworkTolerance(50, ec);
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);
    if (ec)
    {
        std::cerr << "Automatic mode / power on failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "Automatic mode + power on OK." << std::endl;

    // The controller REMEMBERS MotionControlMode across SDK sessions. It is
    // not reset by disconnecting, and not reset by setPowerState(false) -
    // only by restarting the controller. So a previous run that exited while
    // in RtCommand leaves the controller in RtCommand, and this run's
    // task-space calls then fail with
    //
    //     ec -18: this operation is not allowed in current robot running
    //             state (robot not idle - drag / RT mode / identification)
    //
    // which reads like a fault in THIS program rather than leftover state
    // from the last one. Setting the mode explicitly rather than inheriting
    // it is what makes the run reproducible; MotionModeGuard below puts it
    // back on the way out so this program does not do the same to whatever
    // runs next. Diagnosed 2026-09-22, see
    // docs/fault-logs/cr20-calibrate-force-sensor-task-space-unsupported.md.
    robot.setMotionControlMode(MotionControlMode::Idle, ec);
    if (ec)
    {
        std::cerr << "setMotionControlMode(Idle) failed: " << ec.message() << "\n"
                  << "The controller could not be returned to idle, so the task-space setup below\n"
                  << "would fail. If this persists, restart the controller." << std::endl;
        robot.setPowerState(false, ec);
        return 1;
    }

    // -- toolset, and the assertion that it is the one we think ------------

    robot.setToolset(kToolName, kWorkobjectName, ec);
    if (ec)
    {
        std::cerr << "setToolset(" << kToolName << ", " << kWorkobjectName << ") failed: " << ec.message()
                  << std::endl;
        return 1;
    }

    const Toolset active_toolset = robot.toolset(ec);
    if (ec)
    {
        std::cerr << "toolset() query failed: " << ec.message() << std::endl;
        return 1;
    }

    {
        bool matches = withinTolerance(active_toolset.load.mass, kExpectedToolMassKg, kToolMassToleranceKg);
        for (int i = 0; i < 3; ++i)
        {
            matches = matches && withinTolerance(active_toolset.end.trans[i], kExpectedToolTransM[i],
                                                 kToolTransToleranceM);
            matches = matches &&
                      withinTolerance(active_toolset.end.rpy[i], kExpectedToolRpyRad[i], kToolRpyToleranceRad);
        }

        std::cout << "Active toolset: mass " << active_toolset.load.mass << " kg, trans ["
                  << active_toolset.end.trans[0] << ", " << active_toolset.end.trans[1] << ", "
                  << active_toolset.end.trans[2] << "] m, rpy [" << active_toolset.end.rpy[0] << ", "
                  << active_toolset.end.rpy[1] << ", " << active_toolset.end.rpy[2] << "] rad" << std::endl;

        if (!matches)
        {
            std::cerr << "\nREFUSING TO RUN: the active toolset is not the calibrated drill TCP.\n"
                      << "  expected mass " << kExpectedToolMassKg << " kg, trans [" << kExpectedToolTransM[0]
                      << ", " << kExpectedToolTransM[1] << ", " << kExpectedToolTransM[2] << "] m, rpy ["
                      << kExpectedToolRpyRad[0] << ", " << kExpectedToolRpyRad[1] << ", "
                      << kExpectedToolRpyRad[2] << "] rad\n"
                      << "An identity or wrong toolset makes tcpPose_m report the FLANGE pose, so the push\n"
                      << "axis becomes flange Z rather than the bit. That is how a previous run ended up\n"
                      << "driving along a base axis. Re-teach the TCP on the pendant, or update the\n"
                      << "expected values in this file if the tool has genuinely changed." << std::endl;
            robot.setPowerState(false, ec);
            return 1;
        }
        std::cout << "Toolset matches the calibrated drill TCP." << std::endl;
    }

    // -- zero the force sensor, with the bit unloaded ----------------------
    //
    // Must come after setToolset: the SDK's own doc says the correct load has
    // to be set first or the calibration is skewed. It also zeroes any REAL
    // external load, which is why the quickstart has the bit standing off the
    // work rather than resting on it - a preload here would be zeroed out and
    // the force datum would be wrong for the whole run. The call does not
    // block, hence the sleep.

    std::cout << "Calibrating force sensor (bit unloaded)..." << std::endl;
    robot.calibrateForceSensor(true, 0, ec);
    if (ec)
    {
        std::cerr << "calibrateForceSensor failed: " << "ec code" << ec.value() << " - " << ec.message() << std::endl;
        robot.setPowerState(false, ec);
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::cout << "calibrateForceSensor OK." << std::endl;

    // -- switch to real-time control ---------------------------------------
    //
    // Only now, with every task-space operation done. Going in earlier is
    // what produced "calibrateForceSensor failed: task_space unsupported
    // operation" - the controller will not service non-RT calls once it is
    // in RT command mode.

    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    if (ec)
    {
        std::cerr << "setMotionControlMode(RtCommand) failed: " << ec.message() << std::endl;
        robot.setPowerState(false, ec);
        return 1;
    }

    // From here on every exit path - including the exception path and an
    // operator Ctrl+C at the prompt - puts the controller back to Idle, so
    // this run cannot break the next one. See MotionModeGuard.
    MotionModeGuard mode_guard{robot};

    std::shared_ptr<RtMotionControlCobot<6>> rtCon;
    try
    {
        rtCon = robot.getRtMotionController().lock();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Get RT motion controller failed: " << e.what() << std::endl;
        robot.setPowerState(false, ec);
        return 1;
    }
    std::cout << "RT command mode OK." << std::endl;

    // -- impedance configuration -------------------------------------------

    std::array<double, 16> ref_to_world{};
    std::array<double, 16> tool_to_flange{};
    Utils::toolsetCalcPos(active_toolset, ref_to_world, tool_to_flange);

    // setFcCoor wants the force-control frame relative to the FLANGE, which
    // is exactly what toolsetCalcPos just produced. This is the only use of
    // tool_to_flange - it is never used for command math.
    rtCon->setFcCoor(tool_to_flange, FrameType::tool, ec);
    if (ec)
    {
        std::cerr << "setFcCoor failed: " << ec.message() << std::endl;
        return 1;
    }

    rtCon->setCartesianImpedance(kStiffness, ec);
    if (ec)
    {
        std::cerr << "setCartesianImpedance failed: " << ec.message() << std::endl;
        return 1;
    }

    // Explicitly zero rather than simply not called, so a value left behind
    // by a previous session cannot leak into this one. All thrust comes from
    // the commanded lead, which keeps the force model to one equation.
    rtCon->setCartesianImpedanceDesiredTorque({0, 0, 0, 0, 0, 0}, ec);
    if (ec)
    {
        std::cerr << "setCartesianImpedanceDesiredTorque failed: " << ec.message() << std::endl;
        return 1;
    }

    rtCon->setFilterFrequency(50, 50, 50, ec);
    if (ec)
    {
        std::cerr << "setFilterFrequency failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "Impedance configured." << std::endl;

    // -- state stream and baseline -----------------------------------------

    try
    {
        robot.startReceiveRobotState(std::chrono::milliseconds(1),
                                     {RtSupportedFields::tcpPose_m, RtSupportedFields::tauExt_inStiff,
                                      RtSupportedFields::jointPos_m});
    }
    catch (const std::exception& e)
    {
        std::cerr << "startReceiveRobotState failed: " << e.what() << std::endl;
        return 1;
    }
    while (robot.updateRobotState(std::chrono::steady_clock::duration::zero()))
    {
    }

    Pose baseline{};
    robot.getStateData(RtSupportedFields::tcpPose_m, baseline);

    // -- measure the force residual, rather than sampling it once ----------
    //
    // A single tauExt_inStiff sample is worth very little: the per-sample
    // noise on this joint-transducer estimate is around 5 N, with a 10 N p95,
    // so one reading cannot tell a good zero from a bad one. Averaging over a
    // window drops the standard error to well under a newton, which makes the
    // mean a real test of systematic residual - and the sample spread is
    // itself the noise floor every abort threshold has to clear, so both get
    // reported and logged.

    std::array<double, 6> residual_mean{};
    std::array<double, 6> residual_sd{};
    {
        std::array<double, 6> sum{};
        std::array<double, 6> sum_sq{};
        int samples = 0;
        const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(kResidualWindowMs);
        while (std::chrono::steady_clock::now() < until)
        {
            robot.updateRobotState(std::chrono::milliseconds(2));
            std::array<double, 6> wrench{};
            robot.getStateData(RtSupportedFields::tauExt_inStiff, wrench);
            for (int i = 0; i < 6; ++i)
            {
                sum[i] += wrench[i];
                sum_sq[i] += wrench[i] * wrench[i];
            }
            ++samples;
        }
        if (samples > 0)
        {
            for (int i = 0; i < 6; ++i)
            {
                residual_mean[i] = sum[i] / samples;
                const double variance = sum_sq[i] / samples - residual_mean[i] * residual_mean[i];
                residual_sd[i] = std::sqrt(std::max(0.0, variance));
            }
        }
        std::cout << "Force residual over " << samples << " samples: mean [" << residual_mean[0] << ", "
                  << residual_mean[1] << ", " << residual_mean[2] << "] N, sd [" << residual_sd[0] << ", "
                  << residual_sd[1] << ", " << residual_sd[2] << "] N" << std::endl;
    }
    const double residual_magnitude =
        std::sqrt(residual_mean[0] * residual_mean[0] + residual_mean[1] * residual_mean[1] +
                  residual_mean[2] * residual_mean[2]);

    // -- the plan block -----------------------------------------------------

    DrillConfig config;
    config.hole_depth_m = kHoleDepthM;
    config.lead_max_m = kLeadMaxM;
    config.require_contact = kRequireContact;
    config.push_stiffness_n_per_m = kStiffness[2];

    DrillCycle cycle{config};

    std::vector<Sample> trace;
    trace.reserve(kTraceCapacity);

    // -- trace writing, reachable from EVERY exit path after this point ----
    //
    // Originally this lived only at the end of main(), which meant the
    // commissioning ladder's step 1 - "read the plan, then Ctrl+C at the
    // prompt" - produced no file at all, even though the toolset check, the
    // force residual and the decoded push direction are exactly what that
    // step exists to capture. A run that stops early is still a run worth
    // having a record of.
    auto writeTrace = [&](const char* exit_stage) -> std::string {
        const std::string log_path = timestampedLogPath();
        std::ofstream csv{log_path};
        if (!csv)
        {
            std::cerr << "Could not open " << log_path << " for writing. Is " << kLogDir
                      << " bind-mounted? See docker-compose.yml." << std::endl;
            return std::string{};
        }

        csv << std::fixed << std::setprecision(6);
        csv << "# hole_depth_m," << kHoleDepthM << '\n'
            << "# lead_max_m," << kLeadMaxM << '\n'
            << "# lead_at_surface_m," << config.lead_at_surface_m << '\n'
            << "# lead_slope," << config.lead_slope << '\n'
            << "# push_stiffness_n_per_m," << config.push_stiffness_n_per_m << '\n'
            << "# max_thrust_n," << kLeadMaxM * kStiffness[2] << '\n'
            << "# require_contact," << (kRequireContact ? 1 : 0) << '\n'
            << "# contact_datum_m," << cycle.contactDatum() << '\n'
            << "# contact_found," << (cycle.contactFound() ? 1 : 0) << '\n'
            << "# settle_residual_n," << cycle.settleResidualN() << '\n'
            << "# residual_mean_n," << residual_mean[0] << ',' << residual_mean[1] << ',' << residual_mean[2]
            << ',' << residual_mean[3] << ',' << residual_mean[4] << ',' << residual_mean[5] << '\n'
            << "# residual_sd_n," << residual_sd[0] << ',' << residual_sd[1] << ',' << residual_sd[2] << ','
            << residual_sd[3] << ',' << residual_sd[4] << ',' << residual_sd[5] << '\n'
            << "# residual_magnitude_n," << residual_magnitude << '\n'
            << "# stiffness," << kStiffness[0] << ',' << kStiffness[1] << ',' << kStiffness[2] << ','
            << kStiffness[3] << ',' << kStiffness[4] << ',' << kStiffness[5] << '\n'
            << "# feedforward_wrench,0,0,0,0,0,0\n"
            << "# tool_name," << kToolName << '\n'
            << "# tool_trans_m," << active_toolset.end.trans[0] << ',' << active_toolset.end.trans[1] << ','
            << active_toolset.end.trans[2] << '\n'
            << "# tool_rpy_rad," << active_toolset.end.rpy[0] << ',' << active_toolset.end.rpy[1] << ','
            << active_toolset.end.rpy[2] << '\n'
            << "# tool_mass_kg," << active_toolset.load.mass << '\n';

        // The baseline is written out in full so a real run can be replayed
        // through drill_checks off-robot.
        csv << "# baseline_pose";
        for (const double value : baseline)
        {
            csv << ',' << value;
        }
        csv << '\n';

        {
            const auto axis = cr20_drill::bitAxisInParentForDisplay(baseline);
            csv << "# bit_axis_in_base," << axis[0] << ',' << axis[1] << ',' << axis[2] << '\n';
        }

        csv << "# exit_stage," << exit_stage << '\n'
            << "# exit_phase," << cr20_drill::phaseName(cycle.phase()) << '\n'
            << "# abort_reason," << cr20_drill::abortReasonName(cycle.abortReason()) << '\n'
            << "# cycles_captured," << trace.size() << '\n'
            << "# trace_truncated," << (trace.size() >= trace.capacity() ? 1 : 0) << '\n';

        {
            int policy = 0;
            sched_param param{};
            pthread_getschedparam(pthread_self(), &policy, &param);
            csv << "# main_thread_policy," << policy << '\n';
        }

        csv << "t_s,dt_ms,phase,limit,cmd_depth_mm,meas_depth_mm,hole_depth_mm,lead_mm,"
               "lateral_x_mm,lateral_y_mm,axis_drift_deg,fx_n,fy_n,fz_n,tx_nm,ty_nm,tz_nm,"
               "q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg\n";

        csv << std::setprecision(4);
        for (const Sample& sample : trace)
        {
            csv << sample.t_s << ',' << sample.dt_ms << ',' << cr20_drill::phaseName(sample.phase) << ','
                << cr20_drill::limitName(sample.limit) << ',' << sample.commanded_depth_m * 1000.0 << ','
                << sample.measured_depth_m * 1000.0 << ',' << sample.hole_depth_m * 1000.0 << ','
                << sample.lead_m * 1000.0 << ',' << sample.lateral_x_m * 1000.0 << ','
                << sample.lateral_y_m * 1000.0 << ',' << sample.axis_drift_rad * 180.0 / M_PI;
            for (const double value : sample.wrench)
            {
                csv << ',' << value;
            }
            for (const double value : sample.joints)
            {
                csv << ',' << value * 180.0 / M_PI;
            }
            csv << '\n';
        }
        csv.close();

        // Root inside the container, so the file lands root-owned on the host.
        // 0644 lets the host user scp it without sudo.
        chmod(log_path.c_str(), 0644);
        return log_path;
    };

    {
        const DrillCycle& preview = cycle;

        // Computed with the SAME function the control loop commands with, so
        // the printed direction cannot disagree with the motion. A previous
        // version had two routes from "the bit axis" to "a pose", and they
        // drifted apart.
        const auto bit_axis = cr20_drill::bitAxisInParentForDisplay(baseline);
        const Pose probe = cr20_drill::advanceAlongBit(baseline, 0.010);
        const auto from = cr20_drill::translationForDisplay(baseline);
        const auto to = cr20_drill::translationForDisplay(probe);

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "\n=== Run plan ===\n"
                  << "  hole depth        " << kHoleDepthM * 1000.0 << " mm, from the CONTACT DATUM\n"
                  << "  lead cap          " << kLeadMaxM * 1000.0 << " mm  -> "
                  << kLeadMaxM * kStiffness[2] << " N max thrust, and " << kLeadMaxM * 1000.0
                  << " mm max lunge\n"
                  << "  stiffness         [" << kStiffness[0] << ", " << kStiffness[1] << ", " << kStiffness[2]
                  << ", " << kStiffness[3] << ", " << kStiffness[4] << ", " << kStiffness[5] << "]\n"
                  << "  feedforward       zero on every axis\n"
                  << "  contact required  " << (kRequireContact ? "yes" : "NO (free-air stroke)") << "\n";

        std::cout << "\n  Lead schedule\n"
                  << "      hole depth      lead    thrust\n";
        for (const double depth_mm : {0.0, 3.0, 10.0, 20.0, 30.0, 45.0, 55.0})
        {
            if (depth_mm > kHoleDepthM * 1000.0 + 1e-9)
            {
                break;
            }
            std::cout << "      " << std::setw(7) << depth_mm << " mm  " << std::setw(7)
                      << preview.leadForDepth(depth_mm / 1000.0) * 1000.0 << " mm  " << std::setw(7)
                      << preview.thrustForDepth(depth_mm / 1000.0) << " N\n";
        }

        std::cout << "\n  Push direction\n"
                  << "      bit axis (tool +Z) in base: [" << bit_axis[0] << ", " << bit_axis[1] << ", "
                  << bit_axis[2] << "]\n"
                  << "      -> " << describeDirection(bit_axis) << "\n"
                  << "      a 10 mm advance moves the TCP\n"
                  << "         from [" << from[0] * 1000.0 << ", " << from[1] * 1000.0 << ", "
                  << from[2] * 1000.0 << "] mm\n"
                  << "           to [" << to[0] * 1000.0 << ", " << to[1] * 1000.0 << ", " << to[2] * 1000.0
                  << "] mm  (base frame)\n";

        // NOT the force zero, despite appearances. The SDK documents tauExt_*
        // as valid only once RT control is running, and measurement bears
        // that out: sampled here, before startLoop(), the wrench carries the
        // tool's full weight; the instant the loop starts it drops to a few
        // newtons. Measured 2026-09-22 - 26.98 N here against m*g = 26.63 N,
        // then 5.7 N lateral once running.
        //
        // So the useful reading of this number is a sensor liveness check
        // whose EXPECTED value is the tool weight. The force zero proper is
        // measured during the settle phase, with the loop live, and that is
        // what the abort threshold applies to.
        const double tool_weight_n = active_toolset.load.mass * 9.81;
        std::cout << "\n  Raw wrench before the loop starts (payload not yet compensated)\n"
                  << "      mean [" << residual_mean[0] << ", " << residual_mean[1] << ", "
                  << residual_mean[2] << "] N, magnitude " << residual_magnitude << " N\n"
                  << "      sd   [" << residual_sd[0] << ", " << residual_sd[1] << ", " << residual_sd[2]
                  << "] N\n"
                  << "      Expect the magnitude to be the tool weight, " << tool_weight_n
                  << " N. That is what a live,\n"
                  << "      correctly loaded sensor reads at this point - it is not an error.\n"
                  << "      The force zero proper is measured during settle, and aborts above "
                  << config.calibration_residual_abort_n << " N.\n";
        if (std::abs(residual_magnitude - tool_weight_n) > 0.25 * tool_weight_n)
        {
            std::cout << "      WARNING: this does NOT match the tool weight. Either the bit is\n"
                      << "      already loaded, or the payload in the toolset is wrong.\n";
        }

        std::cout << "\n  Standoff: leave the bit 1-2 mm clear of the mark, not touching.\n"
                  << "      The gap must be under " << config.contact_check_arm_depth_m * 1000.0
                  << " mm or the bit never reaches the work before the\n"
                  << "      contact check fires. Hole depth is measured from where the bit lands,\n"
                  << "      so the standoff does not come out of the hole.\n";

        std::cout << "\n  NOTE: collision detection does NOT apply in force control.\n"
                  << "        The E-stop is the only hardware backstop.\n";
    }

    if (!isatty(STDIN_FILENO))
    {
        std::cerr << "\nREFUSING TO RUN: stdin is not a TTY, so Ctrl+C would not reach this process and\n"
                  << "the abort path would be unavailable. Run the container with a TTY - see\n"
                  << "docs/drill-test-quickstart.md." << std::endl;
        robot.setPowerState(false, ec);
        return 1;
    }

    std::cout << "\nCheck the push direction above against what you can see.\n"
              << "Then START THE SPINDLE, and press Enter to begin. Ctrl+C aborts at any time."
              << std::endl;
    std::cin.get();

    if (g_interrupt_requested.load())
    {
        // Ladder step 1 ends here deliberately, so this path still writes a
        // file: the toolset comparison, the force residual and the decoded
        // push direction are exactly what that step is for.
        const std::string path = writeTrace("aborted_at_prompt");
        std::cout << "Interrupted at the prompt; nothing was commanded." << std::endl;
        if (!path.empty())
        {
            std::cout << "Plan recorded: " << path << std::endl;
        }
        robot.setPowerState(false, ec);
        return 0;
    }

    // -- baseline drift tripwire -------------------------------------------

    while (robot.updateRobotState(std::chrono::steady_clock::duration::zero()))
    {
    }
    Pose baseline_now{};
    robot.getStateData(RtSupportedFields::tcpPose_m, baseline_now);
    {
        const cr20_drill::ToolDelta drift = cr20_drill::toolDeltaBetween(baseline, baseline_now);
        const double drift_m = std::sqrt(drift.x_m * drift.x_m + drift.y_m * drift.y_m + drift.z_m * drift.z_m);
        if (drift_m > kBaselineDriftToleranceM)
        {
            std::cerr << "REFUSING TO RUN: the TCP moved " << drift_m * 1000.0
                      << " mm between the plan and the confirmation. It should not have moved at all."
                      << std::endl;
            writeTrace("baseline_drift");
            robot.setPowerState(false, ec);
            return 1;
        }
        baseline = baseline_now;
    }

    // -- the control loop ---------------------------------------------------



    std::atomic<bool> run_finished{false};
    std::atomic<bool> abort_requested{false};
    std::atomic<bool> watchdog_expired{false};

    // Snapshots for the console thread. Plain atomics rather than a lock, so
    // the RT callback never blocks on a reader.
    std::atomic<double> live_hole_depth_mm{0.0};
    std::atomic<double> live_force_n{0.0};
    std::atomic<int> live_phase{0};

    const auto loop_start = std::chrono::steady_clock::now();
    auto previous_tick = loop_start;
    double previous_depth_m = 0.0;
    double depth_rate_m_per_s = 0.0;
    bool first_cycle = true;
    Phase previous_phase = Phase::kSettle;
    cr20_drill::Limit previous_limit = cr20_drill::Limit::kNone;
    std::uint64_t cycle_count = 0;

    std::function<CartesianPosition(void)> callback = [&]() -> CartesianPosition {
        const auto now = std::chrono::steady_clock::now();
        const double dt_s =
            first_cycle ? kCyclePeriodSec : std::chrono::duration<double>(now - previous_tick).count();
        previous_tick = now;

        Pose measured{};
        robot.getStateData(RtSupportedFields::tcpPose_m, measured);
        std::array<double, 6> wrench{};
        robot.getStateData(RtSupportedFields::tauExt_inStiff, wrench);
        std::array<double, 6> joints{};
        robot.getStateData(RtSupportedFields::jointPos_m, joints);

        // ONE call gives depth, lateral wander and tool tilt, all in the
        // baseline's own axes. The baseline is frozen deliberately: a hole is
        // straight, so the push axis must not follow a deflecting wrist.
        const cr20_drill::ToolDelta delta = cr20_drill::toolDeltaBetween(baseline, measured);

        if (first_cycle)
        {
            previous_depth_m = delta.z_m;
            first_cycle = false;
        }
        const double instant_rate = (delta.z_m - previous_depth_m) / std::max(dt_s, 1e-6);
        previous_depth_m = delta.z_m;
        const double alpha = std::clamp(dt_s / kDepthRateTimeConstantSec, 0.0, 1.0);
        depth_rate_m_per_s += alpha * (instant_rate - depth_rate_m_per_s);

        Feedback feedback{};
        feedback.depth_m = delta.z_m;
        feedback.lateral_m = std::hypot(delta.x_m, delta.y_m);
        feedback.axis_drift_rad = cr20_drill::bitAxisAngleRad(baseline, measured);
        feedback.axial_force_n = wrench[2];
        feedback.lateral_force_n = std::hypot(wrench[0], wrench[1]);
        feedback.reaction_torque_nm = wrench[5];
        feedback.depth_rate_m_per_s = depth_rate_m_per_s;
        feedback.dt_s = dt_s;
        feedback.operator_abort = abort_requested.load(std::memory_order_relaxed) ||
                                  g_interrupt_requested.load(std::memory_order_relaxed);
        feedback.watchdog_expired = watchdog_expired.load(std::memory_order_relaxed);

        const cr20_drill::Command command = cycle.step(feedback);

        const bool phase_changed = command.phase != previous_phase;
        previous_phase = command.phase;

        // Decimate on the CYCLE counter, not on trace.size(). Counting output
        // rows makes the two conditions fight each other: a quiet phase never
        // advances the counter, so it logs one row and then nothing, while a
        // phase that trips a limit every cycle logs at the full 1 kHz. The
        // first real run recorded 1 row for a 2 s settle and 47070 rows for
        // the spotting phase.
        // Forcing a row on "a limit is binding" was the other half of the
        // problem: during an advancing phase the lead cap binds continuously,
        // so that condition is true on essentially every cycle and the
        // decimation never gets a look in. What is worth catching at full
        // rate is a limit CHANGING - that is an event - not a limit being the
        // steady state.
        ++cycle_count;
        const bool interesting = phase_changed || command.limit != previous_limit;
        previous_limit = command.limit;
        if (trace.size() < trace.capacity() && (cycle_count % kTraceDecimation == 0 || interesting))
        {
            trace.push_back(Sample{std::chrono::duration<double>(now - loop_start).count(),
                                   dt_s * 1000.0,
                                   command.phase,
                                   command.limit,
                                   command.depth_m,
                                   delta.z_m,
                                   cycle.holeDepth(delta.z_m),
                                   command.lead_m,
                                   delta.x_m,
                                   delta.y_m,
                                   feedback.axis_drift_rad,
                                   wrench,
                                   joints});
        }

        live_hole_depth_mm.store(cycle.holeDepth(delta.z_m) * 1000.0, std::memory_order_relaxed);
        live_force_n.store(wrench[2], std::memory_order_relaxed);
        live_phase.store(static_cast<int>(command.phase), std::memory_order_relaxed);

        // THE only place a commanded pose is produced. There is deliberately
        // no route here that writes a base-frame translation component.
        CartesianPosition output{};
        output.pos = cr20_drill::advanceAlongBit(baseline, command.depth_m);
        if (command.finished)
        {
            output.setFinished();
            run_finished.store(true, std::memory_order_relaxed);
        }
        return output;
    };

    // -- operator abort on Enter -------------------------------------------

    std::thread abort_thread{[&] {
        while (!run_finished.load(std::memory_order_relaxed))
        {
            pollfd fd{STDIN_FILENO, POLLIN, 0};
            if (poll(&fd, 1, 100) > 0)
            {
                char discard[64];
                (void)!read(STDIN_FILENO, discard, sizeof(discard));
                abort_requested.store(true, std::memory_order_relaxed);
                std::cout << "\nABORT requested - retracting." << std::endl;
                return;
            }
        }
    }};

    std::thread console_thread{[&] {
        while (!run_finished.load(std::memory_order_relaxed))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            std::printf("  [%-11s] hole %6.1f mm   force %7.1f N\n",
                        cr20_drill::phaseName(static_cast<Phase>(live_phase.load(std::memory_order_relaxed))),
                        live_hole_depth_mm.load(std::memory_order_relaxed),
                        live_force_n.load(std::memory_order_relaxed));
            std::fflush(stdout);
        }
    }};

    int exit_code = 0;
    try
    {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        rtCon->setControlLoop(callback, kRtThreadPriority, /*useStateDataInLoop=*/true);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        rtCon->startMove(RtControllerMode::cartesianImpedance);
        rtCon->startLoop(/*blocking=*/false);
        std::cout << "Running. Press Enter to abort." << std::endl;

        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(kWatchdogSec));
        const auto hard_deadline = deadline + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                  std::chrono::duration<double>(kWatchdogGraceSec));

        while (!run_finished.load(std::memory_order_relaxed))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            const auto now = std::chrono::steady_clock::now();
            if (now > deadline && !watchdog_expired.load(std::memory_order_relaxed))
            {
                std::cerr << "\nWATCHDOG: " << kWatchdogSec << " s elapsed - retracting." << std::endl;
                watchdog_expired.store(true, std::memory_order_relaxed);
            }
            if (now > hard_deadline)
            {
                std::cerr << "WATCHDOG GRACE EXPIRED: stopping the loop regardless." << std::endl;
                break;
            }
        }

        // Short settle only. Once the callback has returned setFinished() the
        // controller stops moving, but the callback keeps being invoked until
        // stopLoop() tears it down - so waiting longer just re-sends a pose
        // the controller may already consider finished.
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        rtCon->stopLoop();
        std::cout << "Loop stopped." << std::endl;
    }
    catch (const std::exception& e)
    {
        // Falls through to the trace dump rather than returning: a faulted
        // run is exactly the one whose log matters.
        std::cerr << "RT error: " << e.what() << std::endl;
        exit_code = 1;
    }

    run_finished.store(true, std::memory_order_relaxed);
    if (abort_thread.joinable())
    {
        abort_thread.join();
    }
    if (console_thread.joinable())
    {
        console_thread.join();
    }

    robot.setPowerState(false, ec);

    // -- write the trace ----------------------------------------------------

    const std::string log_path = writeTrace("completed");
    if (log_path.empty())
    {
        return exit_code == 0 ? 1 : exit_code;
    }

    std::cout << "\nRun finished in phase " << cr20_drill::phaseName(cycle.phase()) << ", abort reason "
              << cr20_drill::abortReasonName(cycle.abortReason()) << ".\n"
              << "Contact datum " << cycle.contactDatum() * 1000.0 << " mm, settle residual "
              << cycle.settleResidualN() << " N.\n"
              << "Trace: " << log_path << " (" << trace.size() << " rows)\n"
              << "Pull it with:\n"
              << " scp crest@crest-LattePanda-Sigma.local:/home/crest/hyperdrill-logs/'*.csv' logs/test-logs/ "
              << std::endl;

    return exit_code;
}
