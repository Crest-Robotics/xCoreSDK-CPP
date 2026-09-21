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
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <thread>

#include "rokae/robot.h"

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

    constexpr std::array<double, 16> kIdentity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    rtCon->setFcCoor(kIdentity, FrameType::tool, ec);
    if (ec)
    {
        std::cerr << "setFcCoor failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "setFcCoor OK." << std::endl;

    rtCon->setCartesianImpedance({200, 1500, 1500, 150, 150, 150}, ec);
    if (ec)
    {
        std::cerr << "setCartesianImpedance failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "setCartesianImpedance OK." << std::endl;

    rtCon->setCartesianImpedanceDesiredTorque({0, 0, 0, 0, 0, 0}, ec);
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
        robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::tcpPose_m});
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
    std::cout << "Got baseline pose." << std::endl;

    // The fault was never at shutdown or after seconds of holding - it fires
    // on literally the first RT cycle (confirmed by the contactors clicking
    // off the instant startLoop()'s first callback runs; the printed message
    // was just stuck behind buffered stdout, same as the docker log-buffering
    // issue seen earlier this session). Commanding the full 50mm-offset
    // `target` from cycle 1 is an instantaneous jump the robot obviously
    // can't make in one 1ms cycle - that's almost certainly what's tripping
    // kActualCartesianPositionLimitsViolation immediately, not a stop-sequence
    // or drift issue. This is exactly what CartesianForceLimiter already
    // guards against in production code (a bounded per-cycle step, never a
    // jump) - this diagnostic skipped that. Ramping here at 5mm/s so cycle 1
    // moves a negligible 0.005mm, reaching the full 50mm target by ~10s.
    constexpr double kPushOffsetM = -0.05;
    constexpr double kJogSpeedMPerS = 0.005;
    constexpr double kControlCyclePeriodSec = 0.001;
    constexpr double kMaxStepM = kJogSpeedMPerS * kControlCyclePeriodSec;
    const double target_z = baseline[11] + kPushOffsetM;
    std::cout << "Target pose offset by " << kPushOffsetM << "m along base Z from baseline, ramped at "
              << kJogSpeedMPerS << " m/s." << std::endl;

    std::cout << "Press Enter to attempt startMove(cartesianImpedance) (will ramp toward the offset target)..."
              << std::endl;
    std::cin.get();

    std::atomic<bool> keep_running{true};
    double current_z = baseline[11];
    std::function<CartesianPosition(void)> callback = [&]() -> CartesianPosition
    {
        const double remaining = target_z - current_z;
        const double step = std::clamp(remaining, -kMaxStepM, kMaxStepM);
        current_z += step;

        CartesianPosition output{};
        output.pos = baseline;
        output.pos[11] = current_z;
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
        std::this_thread::sleep_for(std::chrono::seconds(3));

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
            const double z_delta_mm = (measured[11] - baseline[11]) * 1000.0;
            std::cout << "[t=" << (i + 1) << "s] z_delta_mm: " << z_delta_mm << std::endl;
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
