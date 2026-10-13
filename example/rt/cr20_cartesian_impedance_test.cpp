/**
 * @file cr20_cartesian_impedance_test.cpp
 * @brief Minimal, CR20-adapted copy of cartesian_impedance_control.cpp, used
 * to isolate whether startMove(cartesianImpedance) succeeds on this specific
 * hardware/controller at all, independent of RtCartesianStreamer entirely.
 *
 * Differences from the reference example: xMateRobot (6-axis) instead of
 * xMateErProRobot (7-axis), this LattePanda/CR20's real IPs, no initial
 * MoveAbsJCommand (holds wherever the arm currently is), no setCartesianLimit,
 * and the control loop holds exactly at the baseline pose - zero commanded
 * motion - rather than running the reference's automatic 200mm Z oscillation.
 *
 * First run (priority=0, unelevated, matching the reference example's
 * default) got past startMove() but faulted after ~5s just holding still:
 * kActualCartesianPositionLimitsViolation. Not chasing that via a
 * setCartesianLimit()-configured safety box - operator judgment plus the
 * E-stop is the safety mechanism here, not a software boundary. Instead,
 * this run requests real-time scheduling for the control loop the same way
 * RtCartesianStreamer already does (see its kRtThreadPriority) - the likely
 * actual cause is jittery, non-RT-scheduled command delivery on this
 * non-dedicated-RT container being misread as an erratic actual position,
 * not a genuine workspace boundary issue. Hold duration bumped from 5s to
 * 30s to get a real confidence check once that's in place.
 *
 * Every step prints success/failure explicitly so a run pinpoints exactly
 * which call (if any) is rejected.
 *
 * Push extension: press along the tool's own +Z axis (not base Z).
 * setFcCoor's `frame` argument is the tool frame expressed RELATIVE TO THE
 * FLANGE - the reference example names its equivalent `toolToFlange`.
 * Passing identity there means "tool frame == flange frame", which made an
 * earlier run push along flange +Z instead of TCP +Z. FrameType::tool only
 * selects the category of task frame (one that moves with the end-effector,
 * vs. world-fixed or path-following); it does NOT make the controller pick
 * up the pendant-configured TCP by itself. The real transform comes from
 * Toolset::end via Utils::toolsetCalcPos().
 *
 * Note the ramp direction was already correct: tcpPose_m appears to be
 * TCP-referenced, so its rotation Z column is true tool Z. Only the
 * force-control frame was wrong. Target of "100N" can't be guaranteed by this API at a shallow
 * depth: setCartesianImpedanceDesiredTorque's feedforward is documented as
 * capped at +-60N per axis, and max Cartesian stiffness is 3000 N/m, so the
 * spring term alone needs ~13.3mm of real position error against a rigid
 * surface to supply the remaining 40N (100 - 60 feedforward). Commanding
 * 20mm of penetration (instead of a literal 10mm) leaves ~6.7mm of margin
 * over that theoretical minimum to actually get close to 100N on real
 * hardware/surface stiffness, at the cost of the exact "10mm" spec. Logs
 * tauExt_inStiff (external wrench in the force-control frame, i.e. tool
 * frame here) alongside position so the actual achieved push force is
 * visible, not just assumed.
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <thread>

#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace rokae;

// Matches RtCartesianStreamer's kRtThreadPriority - requests SCHED_FIFO-style
// scheduling for the SDK's own control-loop thread. Needs the container to
// have CAP_SYS_NICE/an rtprio ulimit; if it can't be set, the SDK only prints
// a console message and continues at normal priority (see setControlLoop()'s
// doc comment), so there's no exception to catch for that specifically.
constexpr int kRtThreadPriority = 80;

int main()
{
    xMateRobot robot;
    const std::string robot_ip = "192.168.2.160";
    const std::string local_ip = "192.168.2.22";
    std::error_code ec;

    try
    {
        robot.connectToRobot(robot_ip, local_ip);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Connect to robot failed: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "Connected." << std::endl;

    robot.setRtNetworkTolerance(50, ec);

    robot.setOperateMode(OperateMode::automatic, ec);
    
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);

    robot.setPowerState(true, ec);
    if (ec)
    {
        std::cerr << "NRT automatic/power-on setup failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "NRT automatic mode + power on OK." << std::endl;

    std::shared_ptr<RtMotionControlCobot<6>> rtCon;
    try
    {
        rtCon = robot.getRtMotionController().lock();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Get RT motion controller failed: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "Got RT motion controller." << std::endl;

    // Toolset::end is "机器人末端坐标系相对法兰坐标系转换" - the TCP frame
    // relative to the flange, which is exactly what setFcCoor wants. Aborting
    // rather than falling back to identity: identity silently pushes along
    // flange Z, which is the bug this replaced.
    Toolset active_toolset = robot.toolset(ec);
    if (ec)
    {
        std::cerr << "toolset() query failed: " << ec.message() << std::endl;
        return 1;
    }
    std::array<double, 16> refToWorld{};
    std::array<double, 16> toolToFlange{};
    Utils::toolsetCalcPos(active_toolset, refToWorld, toolToFlange);
    std::cout << "Toolset end trans (m): [" << active_toolset.end.trans[0] << ", "
              << active_toolset.end.trans[1] << ", " << active_toolset.end.trans[2] << "], rpy (rad): ["
              << active_toolset.end.rpy[0] << ", " << active_toolset.end.rpy[1] << ", "
              << active_toolset.end.rpy[2] << "]" << std::endl;

    rtCon->setFcCoor(toolToFlange, FrameType::tool, ec);
    if (ec)
    {
        std::cerr << "setFcCoor failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "setFcCoor OK (tool-to-flange from toolset)." << std::endl;

    // Impedance factor order is [X, Y, Z, Rx, Ry, Rz] in the fc frame set
    // above (tool). Z is the push axis - maxed at the documented ceiling
    // (3000 N/m) so the position error above actually produces force.
    // X/Y left compliant so the tool can conform laterally on contact.
    rtCon->setCartesianImpedance({200, 200, 3000, 150, 150, 150}, ec);
    if (ec)
    {
        std::cerr << "setCartesianImpedance failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "setCartesianImpedance OK." << std::endl;

    // Feedforward force bias, same [X,Y,Z,Rx,Ry,Rz] fc-frame order. +60N
    // along tool Z is the documented max for this field - see header
    // comment for why the remaining ~40N comes from stiffness * penetration
    // instead.
    rtCon->setCartesianImpedanceDesiredTorque({0, 0, 60, 0, 0, 0}, ec);
    if (ec)
    {
        std::cerr << "setCartesianImpedanceDesiredTorque failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "setCartesianImpedanceDesiredTorque OK." << std::endl;

    // Never set before, on this class or in production RtCartesianStreamer -
    // "used to smooth the command", allowed range 1-1000Hz, recommended
    // 10-100Hz. Same shape of gap as setFcGain() on the non-RT side: an
    // unconfigured smoothing filter left at whatever its default is, which
    // could plausibly make the controller's own filtered view of even a
    // frozen command stream look like a spurious position transient.
    rtCon->setFilterFrequency(/*jointFrequency=*/50, /*cartesianFrequency=*/50, /*torqueFrequency=*/50, ec);
    if (ec)
    {
        std::cerr << "setFilterFrequency failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "setFilterFrequency(50, 50, 50) OK." << std::endl;

    try
    {
        robot.startReceiveRobotState(std::chrono::milliseconds(1),
                                      {RtSupportedFields::tcpPose_m, RtSupportedFields::tauExt_inStiff});
    }
    catch (const std::exception& e)
    {
        std::cerr << "startReceiveRobotState failed: " << e.what() << std::endl;
        return 1;
    }
    while (robot.updateRobotState(std::chrono::steady_clock::duration::zero()))
    {
    }

    std::array<double, 16> baseline{};
    robot.getStateData(RtSupportedFields::tcpPose_m, baseline);
    std::cout << "Got baseline pose. translation (m): [" << baseline[3] << ", " << baseline[7] << ", "
              << baseline[11] << "]" << std::endl;

    // The fault was never at shutdown or after seconds of holding - it fires
    // on literally the first RT cycle (confirmed by the contactors clicking
    // off the instant startLoop()'s first callback runs; the printed message
    // was just stuck behind buffered stdout, same as the docker log-buffering
    // issue seen earlier this session). Commanding a large offset `target`
    // from cycle 1 is an instantaneous jump the robot obviously can't make
    // in one 1ms cycle - that's almost certainly what's tripping
    // kActualCartesianPositionLimitsViolation immediately, not a stop-sequence
    // or drift issue. This is exactly what CartesianForceLimiter already
    // guards against in production code (a bounded per-cycle step, never a
    // jump) - this diagnostic skipped that originally. Ramping here at
    // 5mm/s so cycle 1 moves a negligible 0.005mm.

    // Push direction is the tool's own +Z axis in its CURRENT orientation,
    // not base Z - baseline is a row-major 4x4 (tcpPose_m), so the tool's Z
    // axis expressed in base frame is the third rotation column: indices
    // [2, 6, 10]. Target = baseline translation + depth * that axis.
    const std::array<double, 3> toolZInBase{baseline[2], baseline[6], baseline[10]};
    constexpr double kCommandedPenetrationM = 0.02; // see header comment: 20mm, not 10mm, for force margin
    constexpr double kJogSpeedMPerS = 0.05;
    constexpr double kControlCyclePeriodSec = 0.001;
    constexpr double kMaxStepM = kJogSpeedMPerS * kControlCyclePeriodSec;
    std::cout << "Target pose offset by " << (kCommandedPenetrationM * 1000.0)
              << "mm along tool +Z from baseline, ramped at " << kJogSpeedMPerS
              << " m/s. Push axis in base frame: [" << toolZInBase[0] << ", " << toolZInBase[1] << ", "
              << toolZInBase[2] << "]" << std::endl;

    std::cout << "Press Enter to attempt startMove(cartesianImpedance) (will ramp toward the offset target)..."
              << std::endl;
    std::cin.get();

    std::atomic<bool> keep_running{true};
    double current_depth = 0.0;
    std::function<CartesianPosition(void)> callback = [&]() -> CartesianPosition
    {
        const double remaining = kCommandedPenetrationM - current_depth;
        const double step = std::clamp(remaining, -kMaxStepM, kMaxStepM);
        current_depth += step;

        CartesianPosition output{};
        output.pos = baseline;
        output.pos[3] += current_depth * toolZInBase[0];
        output.pos[7] += current_depth * toolZInBase[1];
        output.pos[11] += current_depth * toolZInBase[2];
        if (!keep_running.load())
        {
            output.setFinished();
        }
        return output;
    };

    try
    {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        rtCon->setControlLoop(callback, kRtThreadPriority, /*useStateDataInLoop=*/true);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        rtCon->startMove(RtControllerMode::cartesianImpedance);
        std::cout << "startMove(cartesianImpedance) SUCCEEDED." << std::endl;
        rtCon->startLoop(/*blocking=*/false);
        std::cout << "Pushing for 10 seconds..." << std::endl;
        for (int i = 0; i < 10; ++i)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // Reading robot_/rtCon from the main thread while the SDK's own
            // RT thread is concurrently calling into it too - unlike
            // RtCartesianStreamer's production code, this quick diagnostic
            // isn't holding to the "one thread owns robot_" rule. Acceptable
            // for a one-off read during a supervised hardware test; not a
            // pattern to carry into production code.
            std::array<double, 16> measured{};
            robot.getStateData(RtSupportedFields::tcpPose_m, measured);
            const double dx = measured[3] - baseline[3];
            const double dy = measured[7] - baseline[7];
            const double dz = measured[11] - baseline[11];
            const double depth_along_tool_z_mm =
                (dx * toolZInBase[0] + dy * toolZInBase[1] + dz * toolZInBase[2]) * 1000.0;

            // tauExt_inStiff is in the fc frame (tool, per setFcCoor above):
            // [Fx, Fy, Fz, Tx, Ty, Tz]. Index 2 is force along the push axis.
            std::array<double, 6> wrench{};
            robot.getStateData(RtSupportedFields::tauExt_inStiff, wrench);
            std::cout << "[t=" << (i + 1) << "s] depth_along_tool_z_mm: " << depth_along_tool_z_mm
                      << ", force_along_tool_z_N: " << wrench[2] << std::endl;
        }

        std::cout << "Control loop requested to stop." << std::endl;
        keep_running.store(false);
        // Short settle, not long: once the callback returns setFinished(),
        // the SDK's own doc says the robot stops moving right then - the
        // callback keeps getting invoked every cycle regardless until
        // stopLoop() tears the loop down, so waiting much longer than a
        // couple of cycles here just means repeatedly re-sending the same
        // frozen baseline position to a session the controller may already
        // consider finished. That mismatch (real position vs. stale resent
        // baseline) is the likely actual cause of the previous
        // kActualCartesianPositionLimitsViolation at shutdown - matches
        // RtCartesianStreamer::stop()'s existing kStopSettleDuration (20ms),
        // not an arbitrarily long wait.
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        rtCon->stopLoop();
        std::cout << "Stopped cleanly." << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "RT move error: " << e.what() << std::endl;
        return 1;
    }

    robot.setPowerState(false, ec);
    robot.setOperateMode(OperateMode::manual, ec);
    robot.setMotionControlMode(MotionControlMode::Idle, ec);

    return 0;
}
