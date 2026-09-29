/**
 * @file admittance_drill_test.cpp
 * @brief Drills one blind hole in the CR20 with a position-mode admittance
 * feed - or, with kFreeAir, runs the free-air stroke that checks the force
 * signal first.
 *
 * Design: docs/drill-anchor-control-design.md. This file only talks to the
 * controller. The two pieces that decide where the bit goes are provable
 * off-robot:
 *
 *   drill_frames.hpp      turns a scalar depth into a commanded pose
 *   admittance_feed.hpp   decides what that scalar should be
 *
 * Run cr20_admittance_checks before every session.
 *
 * HOW IT DIFFERS FROM drill_test.cpp
 *
 *  - RtControllerMode::cartesianPosition, not cartesianImpedance. The five
 *    non-feed axes are held rigidly by the position servos, so the tool
 *    cannot tilt in the hole the way it did at 300 Nm/rad.
 *  - Push is MEASURED (tauExt_inStiff) and fed back into the feed rate; it is
 *    never commanded. No preload, no impedance, no feedforward wrench.
 *  - Collision detection applies again (position control only), so it is
 *    configured here, least sensitive first.
 *  - stopMove() is called before stopLoop() on every exit path, and a stalled
 *    RT callback goes straight to teardown (both from docs/drill-test-todo.md).
 *
 * CONFIRMED by the free-air stroke of 2026-09-25 (admittance-20260925-043850):
 *
 *  - setFcCoor is honoured: tool-frame fz equals the base-frame force
 *    projected on the bit axis exactly.
 *  - Noise is 1.5-2.7 N sd per sample.
 *  - tauExt carries the tool's FULL weight at rest in this mode (26.3 N
 *    against m*g = 26.6 N), so the feed tares all six axes.
 *  - A motion-direction error of about +8 N moving in and -12 N moving out,
 *    with nothing touching. Probably unmodelled joint friction.
 *
 * STILL UNCONFIRMED: the force sign in this mode (hand-push check), and
 * whether the hammer trips collision detection.
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

#include "admittance_feed.hpp"
#include "drill_frames.hpp"

using namespace rokae;

using cr20_drill::Pose;
using cr20_drill::admittance::AbortReason;
using cr20_drill::admittance::AdmittanceConfig;
using cr20_drill::admittance::AdmittanceFeed;
using cr20_drill::admittance::Feedback;
using cr20_drill::admittance::Phase;

// ---------------------------------------------------------------------------
// Test parameters. The ladder: kFreeAir = true first (spindle OFF), then a
// touch-off, then a real hole. See docs/drill-anchor-control-design.md,
// section 8.

/// Free-air stroke: approach kFreeAirStrokeM, hold, retract. Any contact aborts. Left
/// true by default so a fresh build cannot cut.
constexpr bool kFreeAir = false;

/// Free-air stroke length. 100 mm so the motion is obvious by eye; at the
/// 2 mm/s approach and retract feeds that is ~50 s each way.
constexpr double kFreeAirStrokeM = 0.010;

/// Time at the bottom of the stroke (or hole) before retracting. 10 s for the
/// hand-push sign check (test plan step 2): push the bit firmly INTO the work
/// during the hold and push_raw must read positive. Back to 1 s after.
constexpr double kHoldSec = 1.0;

/// Blind-hole depth, measured from the contact depth (bit seated).
constexpr double kHoleDepthM = 0.055;

/// F_des: the push at zero feed. With no feed bias, the steady push while
/// cutting is F_des - B * v_cut - about 50 N at the ~2.7 mm/s seen so far.
constexpr double kDesiredPushN = 100.0;

/// Chain stiffness between the command and the push, from the touch-off of
/// 2026-09-25 (admittance-20260925-052426): least-squares slope of push
/// against commanded travel, 10-45 N, is 25.0 N/mm (r = 0.993); other fits
/// span 22-28 N/mm. The give is outside the arm - probably the bit's buffer in
/// the drill. Used to PRINT the damping ratio, and to choose kDampingNsPerM.
constexpr double kMeasuredChainStiffnessNPerM = 2.5e4;

/// B. zeta = 0.5 * sqrt(B / (K * tau_f)); at the stiff end of the measured
/// range (2.8e4) and tau_f = 53 ms this is 1.30, and zeta stays >= 0.7 up to
/// K = 9.6e4, about 4x the measurement. Halves the default's response time
/// (B/K ~ 0.4 s) and its push shortfall while cutting (B * v = 27 N at
/// 2.7 mm/s, against 54 N).
constexpr double kDampingNsPerM = 1.0e4;

/// Collision thresholds, per joint. The SDK's documented maxima for 6-axis
/// arms other than xMateErPro - the LEAST sensitive setting - because whether
/// the hammer trips detection in position mode is not yet known. Tighten once
/// a run shows the margin.
constexpr std::array<double, 6> kCollisionThresholds{75, 75, 45, 30, 30, 20};

/// The tool and workobject to ACTIVATE, by their pendant names. See
/// drill_test.cpp: an identity toolset makes tcpPose_m report the flange, and
/// "tool Z" becomes flange Z.
constexpr const char* kToolName = "g_tool_0";
constexpr const char* kWorkobjectName = "g_wobj_0";

/// What g_tool_0 is expected to contain on this arm. Same values as
/// drill_test.cpp; update both if the tool changes.
constexpr std::array<double, 3> kExpectedToolTransM{0.014003, 0.008381, 0.280403};
constexpr std::array<double, 3> kExpectedToolRpyRad{0.512429, -0.0205949, 1.50884};
constexpr double kExpectedToolMassKg = 2.714;
constexpr double kToolTransToleranceM = 0.002;
constexpr double kToolRpyToleranceRad = 0.02;
constexpr double kToolMassToleranceKg = 0.05;

/// How far the TCP may move between the plan and the confirmation. A
/// tripwire, not a tolerance.
constexpr double kBaselineDriftToleranceM = 0.001;

constexpr int kRtThreadPriority = 80;

/// Wall-clock backstop, independent of anything the cycle believes.
constexpr double kWatchdogSec = 300.0;
constexpr double kWatchdogGraceSec = 30.0;

/// No new RT cycle for this long means the callback has stopped being called
/// (as after the E-stop on 2026-09-24). Go straight to teardown rather than
/// waiting out the watchdog.
constexpr double kCallbackStallSec = 1.0;

/// Trace buffer. Preallocated and never grown, so the RT callback does not
/// allocate; appends past capacity are dropped.
constexpr std::size_t kTraceCapacity = 400000;

/// Every Nth cycle is written out (50 Hz). Phase changes are always kept.
constexpr int kTraceDecimation = 20;

/// Bind-mounted from the host - see docker-compose.yml.
constexpr const char* kLogDir = "/logs";

namespace
{

std::atomic<bool> g_interrupt_requested{false};

void handleSignal(int signal)
{
    if (g_interrupt_requested.exchange(true))
    {
        // Second Ctrl+C: restore the default and re-raise so the process
        // actually dies even if the retract is stuck.
        std::signal(signal, SIG_DFL);
        std::raise(signal);
        return;
    }
}

/**
 * @class MotionModeGuard
 * @brief Returns the controller to Idle when it goes out of scope.
 *
 * MotionControlMode outlives this process; a run that exits in RtCommand
 * breaks the next run's task-space calls with ec -18. See drill_test.cpp and
 * docs/fault-logs/cr20-calibrate-force-sensor-task-space-unsupported.md.
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
    double commanded_depth_m;
    double measured_depth_m;
    double hole_depth_m;
    double feed_m_per_s;
    double push_raw_n;
    double push_control_n;
    double push_fast_n;
    double lateral_x_m;
    double lateral_y_m;
    double axis_drift_rad;
    std::array<double, 6> wrench_stiff;
    std::array<double, 6> wrench_base;
    std::array<double, 6> joints;
};

/// Plain-language decode of a unit direction in the base frame.
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

    return std::string{kLogDir} + "/admittance-" + stamp + ".csv";
}

/// stopMove() is the SDK's documented way to end motion started with
/// startMove(); setFinished() alone never happens on a fault path. It throws
/// on failure, and this runs during teardown - often while another failure
/// is already being reported - so it only reports.
void stopMoveQuietly(const std::shared_ptr<RtMotionControlCobot<6>>& rt_con)
{
    try
    {
        rt_con->stopMove();
    }
    catch (const std::exception& e)
    {
        std::cerr << "WARNING: stopMove() failed: " << e.what() << std::endl;
    }
}

void stopLoopQuietly(const std::shared_ptr<RtMotionControlCobot<6>>& rt_con)
{
    try
    {
        rt_con->stopLoop();
    }
    catch (const std::exception& e)
    {
        std::cerr << "WARNING: stopLoop() failed: " << e.what() << std::endl;
    }
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

    // Task-space setup (setToolset, calibrateForceSensor) must happen while
    // the controller is Idle; RtCommand only afterwards. See drill_test.cpp.
    robot.setRtNetworkTolerance(50, ec);
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);
    if (ec)
    {
        std::cerr << "Automatic mode / power on failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "Automatic mode + power on OK." << std::endl;

    // The controller REMEMBERS MotionControlMode across sessions; set it
    // explicitly rather than inheriting a previous run's RtCommand.
    robot.setMotionControlMode(MotionControlMode::Idle, ec);
    if (ec)
    {
        std::cerr << "setMotionControlMode(Idle) failed: " << ec.message() << "\n"
                  << "If this persists, restart the controller." << std::endl;
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
                      << "An identity or wrong toolset makes tcpPose_m report the FLANGE pose, so the feed\n"
                      << "axis becomes flange Z rather than the bit. Re-teach the TCP on the pendant, or\n"
                      << "update the expected values in this file if the tool has genuinely changed."
                      << std::endl;
            robot.setPowerState(false, ec);
            return 1;
        }
        std::cout << "Toolset matches the calibrated drill TCP." << std::endl;
    }

    // -- zero the force sensor, with the bit unloaded ----------------------

    std::cout << "Calibrating force sensor (bit unloaded)..." << std::endl;
    robot.calibrateForceSensor(true, 0, ec);
    if (ec)
    {
        std::cerr << "calibrateForceSensor failed: ec " << ec.value() << " - " << ec.message() << std::endl;
        robot.setPowerState(false, ec);
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::cout << "calibrateForceSensor OK." << std::endl;

    // -- switch to real-time control ---------------------------------------

    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    if (ec)
    {
        std::cerr << "setMotionControlMode(RtCommand) failed: " << ec.message() << std::endl;
        robot.setPowerState(false, ec);
        return 1;
    }

    // From here on every exit path puts the controller back to Idle.
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

    // -- position-mode configuration ---------------------------------------

    std::array<double, 16> ref_to_world{};
    std::array<double, 16> tool_to_flange{};
    Utils::toolsetCalcPos(active_toolset, ref_to_world, tool_to_flange);

    // Puts tauExt_inStiff in TOOL axes, so fz is along the bit. It is a force
    // control setting; whether cartesianPosition honours it is one of the
    // things the free-air stroke confirms (fz should track a hand push along
    // the bit). If the controller refuses it outright, do not guess the frame.
    rtCon->setFcCoor(tool_to_flange, FrameType::tool, ec);
    if (ec)
    {
        std::cerr << "REFUSING TO RUN: setFcCoor failed in RT mode: " << ec.message() << "\n"
                  << "Without it the frame of tauExt_inStiff is unknown, and the feed's force sign\n"
                  << "cannot be trusted." << std::endl;
        robot.setPowerState(false, ec);
        return 1;
    }

    rtCon->setCollisionBehaviour(kCollisionThresholds, ec);
    if (ec)
    {
        std::cerr << "setCollisionBehaviour failed: " << ec.message() << std::endl;
        robot.setPowerState(false, ec);
        return 1;
    }

    rtCon->setFilterFrequency(50, 50, 50, ec);
    if (ec)
    {
        std::cerr << "setFilterFrequency failed: " << ec.message() << std::endl;
        robot.setPowerState(false, ec);
        return 1;
    }
    std::cout << "Position mode configured." << std::endl;

    // -- state stream and baseline -----------------------------------------

    try
    {
        robot.startReceiveRobotState(std::chrono::milliseconds(1),
                                     {RtSupportedFields::tcpPose_m, RtSupportedFields::tauExt_inStiff,
                                      RtSupportedFields::tauExt_inBase, RtSupportedFields::jointPos_m});
    }
    catch (const std::exception& e)
    {
        std::cerr << "startReceiveRobotState failed: " << e.what() << std::endl;
        robot.setPowerState(false, ec);
        return 1;
    }
    while (robot.updateRobotState(std::chrono::steady_clock::duration::zero()))
    {
    }

    Pose baseline{};
    robot.getStateData(RtSupportedFields::tcpPose_m, baseline);

    // -- the feed -----------------------------------------------------------

    AdmittanceConfig config;
    config.free_air = kFreeAir;
    config.free_air_stroke_m = kFreeAirStrokeM;
    config.hold_sec = kHoldSec;
    config.damping_n_s_per_m = kDampingNsPerM;
    config.hole_depth_m = kHoleDepthM;
    config.desired_push_n = kDesiredPushN;

    AdmittanceFeed feed{config};

    std::vector<Sample> trace;
    trace.reserve(kTraceCapacity);

    // -- trace writing, reachable from every exit path after this point ----

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
        csv << "# controller,cartesian_position_admittance\n"
            << "# free_air," << (config.free_air ? 1 : 0) << '\n'
            << "# free_air_stroke_m," << config.free_air_stroke_m << '\n'
            << "# hole_depth_m," << config.hole_depth_m << '\n'
            << "# desired_push_n," << config.desired_push_n << '\n'
            << "# feed_bias_m_per_s," << config.feed_bias_m_per_s << '\n'
            << "# damping_n_s_per_m," << config.damping_n_s_per_m << '\n'
            << "# approach_feed_m_per_s," << config.approach_feed_m_per_s << '\n'
            << "# max_feed_m_per_s," << config.max_feed_m_per_s << '\n'
            << "# max_backoff_m_per_s," << config.max_backoff_m_per_s << '\n'
            << "# max_accel_m_per_s2," << config.max_accel_m_per_s2 << '\n'
            << "# control_filter_hz," << config.control_filter_hz << '\n'
            << "# abort_filter_hz," << config.abort_filter_hz << '\n'
            << "# contact_push_n," << config.contact_push_n << '\n'
            << "# retract_feed_m_per_s," << config.retract_feed_m_per_s << '\n'
            << "# retract_pull_cap_n," << config.retract_pull_cap_n << '\n'
            << "# axial_force_abort_n," << config.axial_force_abort_n << '\n'
            << "# collision_thresholds," << kCollisionThresholds[0] << ',' << kCollisionThresholds[1] << ','
            << kCollisionThresholds[2] << ',' << kCollisionThresholds[3] << ',' << kCollisionThresholds[4]
            << ',' << kCollisionThresholds[5] << '\n'
            << "# wrench_tare," << feed.wrenchTare()[0] << ',' << feed.wrenchTare()[1] << ','
            << feed.wrenchTare()[2] << ',' << feed.wrenchTare()[3] << ',' << feed.wrenchTare()[4] << ','
            << feed.wrenchTare()[5] << '\n'
            << "# contact_found," << (feed.contactFound() ? 1 : 0) << '\n'
            << "# contact_depth_m," << feed.contactDepth() << '\n'
            << "# tool_name," << kToolName << '\n'
            << "# tool_trans_m," << active_toolset.end.trans[0] << ',' << active_toolset.end.trans[1] << ','
            << active_toolset.end.trans[2] << '\n'
            << "# tool_rpy_rad," << active_toolset.end.rpy[0] << ',' << active_toolset.end.rpy[1] << ','
            << active_toolset.end.rpy[2] << '\n'
            << "# tool_mass_kg," << active_toolset.load.mass << '\n';

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
            << "# exit_phase," << cr20_drill::admittance::phaseName(feed.phase()) << '\n'
            << "# abort_reason," << cr20_drill::admittance::abortReasonName(feed.abortReason()) << '\n'
            << "# cycles_captured," << trace.size() << '\n'
            << "# trace_truncated," << (trace.size() >= trace.capacity() ? 1 : 0) << '\n';

        csv << "t_s,dt_ms,phase,cmd_depth_mm,meas_depth_mm,hole_depth_mm,feed_mm_s,push_raw_n,push_ctrl_n,"
               "push_fast_n,lateral_x_mm,lateral_y_mm,axis_drift_deg,fx_n,fy_n,fz_n,tx_nm,ty_nm,tz_nm,"
               "fx_base_n,fy_base_n,fz_base_n,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg\n";

        csv << std::setprecision(4);
        for (const Sample& sample : trace)
        {
            csv << sample.t_s << ',' << sample.dt_ms << ',' << cr20_drill::admittance::phaseName(sample.phase)
                << ',' << sample.commanded_depth_m * 1000.0 << ',' << sample.measured_depth_m * 1000.0 << ','
                << sample.hole_depth_m * 1000.0 << ',' << sample.feed_m_per_s * 1000.0 << ','
                << sample.push_raw_n << ',' << sample.push_control_n << ',' << sample.push_fast_n << ','
                << sample.lateral_x_m * 1000.0 << ',' << sample.lateral_y_m * 1000.0 << ','
                << sample.axis_drift_rad * 180.0 / M_PI;
            for (const double value : sample.wrench_stiff)
            {
                csv << ',' << value;
            }
            for (int i = 0; i < 3; ++i)
            {
                csv << ',' << sample.wrench_base[i];
            }
            for (const double value : sample.joints)
            {
                csv << ',' << value * 180.0 / M_PI;
            }
            csv << '\n';
        }
        csv.close();

        // Root inside the container; 0644 lets the host user scp it.
        chmod(log_path.c_str(), 0644);
        return log_path;
    };

    // -- the plan block -----------------------------------------------------

    {
        // Computed with the SAME function the loop commands with, so the
        // printed direction cannot disagree with the motion.
        const auto bit_axis = cr20_drill::bitAxisInParentForDisplay(baseline);
        const Pose probe = cr20_drill::advanceAlongBit(baseline, 0.010);
        const auto from = cr20_drill::translationForDisplay(baseline);
        const auto to = cr20_drill::translationForDisplay(probe);

        const double tau_f = 1.0 / (2.0 * M_PI * config.control_filter_hz);
        const double zeta =
            0.5 * std::sqrt(config.damping_n_s_per_m / (kMeasuredChainStiffnessNPerM * tau_f));

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "\n=== Run plan: position-mode admittance ===\n"
                  << "  mode              "
                  << (config.free_air ? "FREE-AIR STROKE (any contact aborts)" : "REAL HOLE") << "\n";
        if (config.free_air)
        {
            std::cout << "  stroke            " << config.free_air_stroke_m * 1000.0 << " mm at "
                      << config.approach_feed_m_per_s * 1000.0 << " mm/s, then hold and retract\n";
        }
        else
        {
            std::cout << "  hole depth        " << config.hole_depth_m * 1000.0 << " mm, from where the bit SEATS\n"
                      << "  approach          " << config.approach_feed_m_per_s * 1000.0
                      << " mm/s until push > " << config.contact_push_n << " N (max "
                      << config.max_approach_m * 1000.0 << " mm)\n";
        }
        std::cout << "  law               v = V_des + (F_des - push) / B\n"
                  << "  F_des             " << config.desired_push_n << " N  (push at ZERO feed)\n"
                  << "  V_des             " << config.feed_bias_m_per_s * 1000.0 << " mm/s\n"
                  << "  B                 " << config.damping_n_s_per_m << " N.s/m  ("
                  << config.damping_n_s_per_m / 1000.0 << " N per mm/s of feed)\n"
                  << "  feed limits       +" << config.max_feed_m_per_s * 1000.0 << " / -"
                  << config.max_backoff_m_per_s * 1000.0 << " mm/s, accel "
                  << config.max_accel_m_per_s2 * 1000.0 << " mm/s^2\n"
                  << "  filters           control " << config.control_filter_hz << " Hz, aborts "
                  << config.abort_filter_hz << " Hz\n"
                  << "  damping ratio     " << zeta << "  at the MEASURED chain stiffness of "
                  << kMeasuredChainStiffnessNPerM << " N/m (touch-off 2026-09-25)\n"
                  << "  aborts            |push| > " << config.axial_force_abort_n << " N, push < -"
                  << config.reversed_push_abort_n << " N, lateral > " << config.lateral_force_abort_n
                  << " N, tz > " << config.reaction_torque_abort_nm << " Nm\n"
                  << "  retract           " << config.retract_feed_m_per_s * 1000.0 << " mm/s, stops if pull > "
                  << config.retract_pull_cap_n << " N\n"
                  << "  collision         thresholds [" << kCollisionThresholds[0] << ", "
                  << kCollisionThresholds[1] << ", " << kCollisionThresholds[2] << ", "
                  << kCollisionThresholds[3] << ", " << kCollisionThresholds[4] << ", "
                  << kCollisionThresholds[5] << "] (least sensitive)\n";

        std::cout << "\n  Feed direction\n"
                  << "      bit axis (tool +Z) in base: [" << bit_axis[0] << ", " << bit_axis[1] << ", "
                  << bit_axis[2] << "]\n"
                  << "      -> " << describeDirection(bit_axis) << "\n"
                  << "      a 10 mm advance moves the TCP\n"
                  << "         from [" << from[0] * 1000.0 << ", " << from[1] * 1000.0 << ", "
                  << from[2] * 1000.0 << "] mm\n"
                  << "           to [" << to[0] * 1000.0 << ", " << to[1] * 1000.0 << ", " << to[2] * 1000.0
                  << "] mm  (base frame)\n";

        std::cout << "\n  The force tare is measured in the first " << config.settle_sec
                  << " s of the loop. Keep the bit UNLOADED until then.\n";
    }

    if (!isatty(STDIN_FILENO))
    {
        std::cerr << "\nREFUSING TO RUN: stdin is not a TTY, so Ctrl+C would not reach this process and\n"
                  << "the abort path would be unavailable. Run the container with a TTY - see\n"
                  << "docs/drill-test-quickstart.md." << std::endl;
        robot.setPowerState(false, ec);
        return 1;
    }

    if (config.free_air)
    {
        std::cout << "\nFREE AIR: do NOT start the spindle. Check nothing is within "
                  << config.free_air_stroke_m * 1000.0 + 10.0 << " mm of the bit.\n";
    }
    else
    {
        std::cout << "\nCheck the feed direction above against what you can see.\n"
                  << "Then START THE SPINDLE.\n";
    }
    std::cout << "Press Enter to begin. Ctrl+C aborts at any time." << std::endl;
    std::cin.get();

    if (g_interrupt_requested.load())
    {
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
                      << " mm between the plan and the confirmation." << std::endl;
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

    /// Incremented every RT cycle; the main thread watches it for a stall.
    std::atomic<std::uint64_t> cycle_count{0};

    // Snapshots for the console thread.
    std::atomic<double> live_hole_depth_mm{0.0};
    std::atomic<double> live_push_n{0.0};
    std::atomic<double> live_feed_mm_s{0.0};
    std::atomic<int> live_phase{0};

    const auto loop_start = std::chrono::steady_clock::now();
    auto previous_tick = loop_start;
    bool first_cycle = true;
    Phase previous_phase = Phase::kSettle;

    std::function<CartesianPosition(void)> callback = [&]() -> CartesianPosition {
        const auto now = std::chrono::steady_clock::now();
        const double dt_s = first_cycle ? cr20_drill::admittance::kCyclePeriodSec
                                        : std::chrono::duration<double>(now - previous_tick).count();
        previous_tick = now;
        first_cycle = false;

        Pose measured{};
        robot.getStateData(RtSupportedFields::tcpPose_m, measured);
        std::array<double, 6> wrench_stiff{};
        robot.getStateData(RtSupportedFields::tauExt_inStiff, wrench_stiff);
        std::array<double, 6> wrench_base{};
        robot.getStateData(RtSupportedFields::tauExt_inBase, wrench_base);
        std::array<double, 6> joints{};
        robot.getStateData(RtSupportedFields::jointPos_m, joints);

        // Depth, wander and tilt, all in the frozen baseline's own axes.
        const cr20_drill::ToolDelta delta = cr20_drill::toolDeltaBetween(baseline, measured);

        Feedback feedback{};
        feedback.wrench = wrench_stiff;
        feedback.measured_depth_m = delta.z_m;
        feedback.lateral_m = std::hypot(delta.x_m, delta.y_m);
        feedback.axis_drift_rad = cr20_drill::bitAxisAngleRad(baseline, measured);
        feedback.operator_abort = abort_requested.load(std::memory_order_relaxed) ||
                                  g_interrupt_requested.load(std::memory_order_relaxed);
        feedback.watchdog_expired = watchdog_expired.load(std::memory_order_relaxed);

        const cr20_drill::admittance::Command command = feed.step(feedback);

        const bool phase_changed = command.phase != previous_phase;
        previous_phase = command.phase;

        const std::uint64_t count = cycle_count.fetch_add(1, std::memory_order_relaxed) + 1;
        if (trace.size() < trace.capacity() && (count % kTraceDecimation == 0 || phase_changed))
        {
            trace.push_back(Sample{std::chrono::duration<double>(now - loop_start).count(),
                                   dt_s * 1000.0,
                                   command.phase,
                                   command.depth_m,
                                   delta.z_m,
                                   feed.holeDepth(command.depth_m),
                                   command.feed_m_per_s,
                                   cr20_drill::admittance::kFzToPushSign *
                                       (wrench_stiff[2] - feed.wrenchTare()[cr20_drill::admittance::kFz]),
                                   command.push_control_n,
                                   command.push_fast_n,
                                   delta.x_m,
                                   delta.y_m,
                                   feedback.axis_drift_rad,
                                   wrench_stiff,
                                   wrench_base,
                                   joints});
        }

        live_hole_depth_mm.store(feed.holeDepth(command.depth_m) * 1000.0, std::memory_order_relaxed);
        live_push_n.store(command.push_control_n, std::memory_order_relaxed);
        live_feed_mm_s.store(command.feed_m_per_s * 1000.0, std::memory_order_relaxed);
        live_phase.store(static_cast<int>(command.phase), std::memory_order_relaxed);

        // THE only place a commanded pose is produced: always on the baseline
        // line, never a base-frame translation component.
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
            std::printf("  [%-8s] hole %6.1f mm   push %7.1f N   feed %5.2f mm/s\n",
                        cr20_drill::admittance::phaseName(
                            static_cast<Phase>(live_phase.load(std::memory_order_relaxed))),
                        live_hole_depth_mm.load(std::memory_order_relaxed),
                        live_push_n.load(std::memory_order_relaxed),
                        live_feed_mm_s.load(std::memory_order_relaxed));
            std::fflush(stdout);
        }
    }};

    int exit_code = 0;
    const char* exit_stage = "completed";
    bool move_started = false;
    try
    {
        // Same pauses as drill_test.cpp, which is known to work.
        std::this_thread::sleep_for(std::chrono::seconds(3));
        rtCon->setControlLoop(callback, kRtThreadPriority, /*useStateDataInLoop=*/true);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        rtCon->startMove(RtControllerMode::cartesianPosition);
        move_started = true;
        rtCon->startLoop(/*blocking=*/false);
        std::cout << "Running. Press Enter to abort." << std::endl;

        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(kWatchdogSec));
        const auto hard_deadline = deadline + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                  std::chrono::duration<double>(kWatchdogGraceSec));

        std::uint64_t last_count = 0;
        auto last_progress = std::chrono::steady_clock::now();

        while (!run_finished.load(std::memory_order_relaxed))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            const auto now = std::chrono::steady_clock::now();

            // Stalled callback: the controller has stopped calling us (an
            // E-stop does exactly this). Nothing the feed decides can take
            // effect any more, so go straight to teardown.
            const std::uint64_t count = cycle_count.load(std::memory_order_relaxed);
            if (count != last_count)
            {
                last_count = count;
                last_progress = now;
            }
            else if (std::chrono::duration<double>(now - last_progress).count() > kCallbackStallSec)
            {
                std::cerr << "\nRT CALLBACK STALLED: no cycle for " << kCallbackStallSec
                          << " s (E-stop or controller fault?). Tearing down." << std::endl;
                exit_stage = "callback_stalled";
                exit_code = 1;
                break;
            }

            if (now > deadline && !watchdog_expired.load(std::memory_order_relaxed))
            {
                std::cerr << "\nWATCHDOG: " << kWatchdogSec << " s elapsed - retracting." << std::endl;
                watchdog_expired.store(true, std::memory_order_relaxed);
            }
            if (now > hard_deadline)
            {
                std::cerr << "WATCHDOG GRACE EXPIRED: stopping the loop regardless." << std::endl;
                exit_stage = "watchdog_grace_expired";
                exit_code = 1;
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    catch (const std::exception& e)
    {
        // Falls through to teardown and the trace dump: a faulted run is
        // exactly the one whose log matters.
        std::cerr << "RT error: " << e.what() << std::endl;
        exit_stage = "rt_error";
        exit_code = 1;
    }

    // -- teardown: stopMove BEFORE stopLoop, on every path ------------------

    if (move_started)
    {
        stopMoveQuietly(rtCon);
    }
    stopLoopQuietly(rtCon);
    std::cout << "Motion and loop stopped." << std::endl;

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

    const std::string log_path = writeTrace(exit_stage);
    if (log_path.empty())
    {
        return exit_code == 0 ? 1 : exit_code;
    }

    std::cout << "\nRun finished in phase " << cr20_drill::admittance::phaseName(feed.phase()) << ", abort reason "
              << cr20_drill::admittance::abortReasonName(feed.abortReason()) << ".\n"
              << "Wrench tare [" << feed.wrenchTare()[0] << ", " << feed.wrenchTare()[1] << ", "
              << feed.wrenchTare()[2] << "] N, tz " << feed.wrenchTare()[5] << " Nm";
    if (feed.contactFound())
    {
        std::cout << ", contact at " << feed.contactDepth() * 1000.0 << " mm of travel";
    }
    std::cout << ".\n"
              << "Trace: " << log_path << " (" << trace.size() << " rows)\n"
              << "Pull it with:\n"
              << " scp crest@crest-LattePanda-Sigma.local:/home/crest/hyperdrill-logs/'*.csv' logs/test-logs/ "
              << std::endl;

    return exit_code;
}
