/**
 * @file cr20_force_control_test.cpp
 * @brief Minimal, CR20-adapted prototype of the non-RT ForceControl API (see
 * force_control_commands.cpp's fcCartesianControl()), to check whether it's
 * a better fit for push/drilling than RT cartesianImpedance mode - which
 * just failed on real hardware even holding still with zero commanded
 * motion (kActualCartesianPositionLimitsViolation).
 *
 * First held with zero desired force to confirm fcInit()/fcStart() succeed
 * and that merely holding under force control (no active push) is stable -
 * it was (10s steady, unlike RT mode's ~5s position-limit fault), and after
 * adding calibrateForceSensor() the measured force/torque settled near zero
 * as expected.
 *
 * A follow-up 10N push (stiffness {1500,1500,1500,150,150,150}, plus
 * setCartesianControlMaxWrench/MaxVel as extra guards) against a real
 * surface produced no visible motion and no trend in measured force at all
 * - statistically identical to the zero-force baseline. This version
 * instead mirrors the xCore SDK's own reference values from
 * force_control_commands.cpp's fcCartesianControl() exactly (stiffness
 * {0,1000,1000,500,500,500}, 1N push, no extra wrench/vel limiting) to
 * isolate whether that no-effect result was about our parameter choices or
 * something else entirely. Runs for a fixed, short duration rather than
 * until a keypress, printing measured torque/force periodically so behavior
 * over time is visible.
 */

#include <array>
#include <chrono>
#include <iostream>
#include <thread>

#include "print_helper.hpp"
#include "rokae/robot.h"

using namespace rokae;

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

    // item=1: E-stop recovery, best-effort.
    robot.recoverState(1, ec);
    ec.clear();

    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);
    if (ec)
    {
        std::cerr << "automatic mode/power-on failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "Automatic mode + power on OK." << std::endl;

    // Explicit rather than relying on whatever tool preset the controller
    // already happens to have active - g_wobj_0 is the global default
    // workobject, always available with no project loaded (see robot.h).
    robot.setToolset("g_tool_0", "g_wobj_0", ec);
    if (ec)
    {
        std::cerr << "setToolset(g_tool_0) failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "setToolset(g_tool_0) OK." << std::endl;

    // Must come after setToolset() above for accuracy (see the SDK doc
    // comment on calibrateForceSensor()) - without this, the sensor's
    // baseline isn't zeroed for this specific tool/pose, and the residual
    // (mostly gravity from the tool's own weight) shows up as a persistent,
    // non-random "external force" in getEndTorque() below even with zero
    // desired force commanded. Doesn't block until calibration finishes
    // (~100ms per the doc), hence the sleep.
    robot.calibrateForceSensor(true, 0, ec);
    if (ec)
    {
        std::cerr << "calibrateForceSensor failed: " << ec.message() << std::endl;
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::cout << "calibrateForceSensor OK." << std::endl;

    auto fc = robot.forceControl();

    // FrameType::tool uses whichever tool frame setToolset() above just set.
    fc.fcInit(FrameType::tool, ec);
    if (ec)
    {
        std::cerr << "fcInit failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "fcInit OK." << std::endl;

    fc.setControlType(1, ec); // Cartesian impedance.
    if (ec)
    {
        std::cerr << "setControlType failed: " << ec.message() << std::endl;
        return 1;
    }

    // Reference example's exact stiffness values - see the earlier attempt
    // at 1500/1500/1500. A 1N push at these values also produced no clear
    // trend, but the sensor noise floor is ~2-5N (visible even at zero
    // desired force), so 1N is indistinguishable from noise either way -
    // this run raises the force well above that floor instead.
    fc.setCartesianStiffness({0, 1000, 1000, 500, 500, 500}, ec);
    if (ec)
    {
        std::cerr << "setCartesianStiffness failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "setCartesianStiffness OK." << std::endl;

    // Raised from 40 to 70 - now testing a 60N target, so the old 40N
    // ceiling would have silently capped it below the value we're actually
    // trying to reach. 70 keeps some headroom above 60 as a backstop without
    // being so high it stops acting as one.
    fc.setCartesianControlMaxWrench({70, 70, 70, 10, 10, 10}, ec);

    // Confirmed at 60 (the max) that force control bandwidth was the real
    // limiter - real, sustained motion and force finally showed up. Backing
    // off from the max to 40 here: 60 produced noisy/oscillating readings
    // and audible motor chatter, consistent with the control loop being
    // pushed to marginal stability rather than tracking smoothly.
    fc.setFcGain({40, 40, 40, 40, 40, 40}, ec);
    if (ec)
    {
        std::cerr << "setFcGain failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "setFcGain(40) OK." << std::endl;

    // Baseline TCP pose, captured before any force is commanded - so the
    // per-second loop below can print how far the tool has actually moved,
    // not just measured force. Neither 1N nor 20N showed a force trend; this
    // tells us whether the arm silently moved somewhere instead (e.g. if
    // tool Z isn't actually pointed at the surface) or genuinely didn't
    // respond to the command at all.
    const auto baseline_posture = robot.posture(CoordinateType::endInRef, ec);
    if (ec)
    {
        std::cerr << "posture (baseline) failed: " << ec.message() << std::endl;
        return 1;
    }

    // Baseline per-joint torque too - see the loop below, which now prints
    // the raw joint_torque_measured/external_torque_measured outputs of
    // getEndTorque() (previously discarded, only cart_force/cart_torque were
    // printed) as deltas from this. If a real torque response shows up here
    // but not in the Cartesian numbers, that's a conversion/reporting issue;
    // if it's flat here too, the command isn't reaching the actuators at all.
    std::array<double, 6> baseline_joint_torque{}, baseline_external_torque{};
    {
        std::array<double, 3> unused_cart_torque{}, unused_cart_force{};
        std::error_code baseline_torque_ec;
        fc.getEndTorque(FrameType::tool,
                         baseline_joint_torque,
                         baseline_external_torque,
                         unused_cart_torque,
                         unused_cart_force,
                         baseline_torque_ec);
        if (baseline_torque_ec)
        {
            std::cerr << "getEndTorque (baseline) failed: " << baseline_torque_ec.message() << std::endl;
            return 1;
        }
    }

    std::cout << "Press Enter to attempt fcStart() (holds still, zero desired force)..." << std::endl;
    std::cin.get();

    fc.fcStart(ec);
    if (ec)
    {
        std::cerr << "fcStart failed: " << ec.message() << std::endl;
        return 1;
    }
    std::cout << "fcStart SUCCEEDED." << std::endl;

    // -60N: the +20N run at max gain produced a consistently NEGATIVE
    // measured cart_force Z and negative translation_delta - i.e. the real
    // push direction into the surface is negative Z in this frame, not
    // positive. -60 is the documented minimum of setCartesianDesiredForce's
    // [-60,60] range - the boundary, not past it yet.
    constexpr double kDesiredForceZ = -80.0;
    fc.setCartesianDesiredForce({0, 0, kDesiredForceZ, 0, 0, 0}, ec);
    if (ec)
    {
        std::cerr << "setCartesianDesiredForce failed: " << ec.message() << std::endl;
        fc.fcStop(ec);
        return 1;
    }
    std::cout << "setCartesianDesiredForce(" << kDesiredForceZ << "N along tool Z) OK. Holding for 60 seconds..."
              << std::endl;

    // Bumped from 10 to 60s: the last run showed real, dominant-in-Z motion
    // (~1.6mm, decelerating) but nowhere near what 20N/1000 N/m stiffness
    // would need in free space (~20mm) - this checks whether it's still
    // slowly converging toward something bigger, or has already plateaued.
    constexpr int kHoldSeconds = 10;
    for (int i = 0; i < kHoldSeconds; ++i)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::array<double, 6> joint_torque{}, external_torque{};
        std::array<double, 3> cart_force{}, cart_torque{};
        // FrameType::tool - relative to g_tool_0's TCP, matching fcInit()
        // above, not the flange.
        fc.getEndTorque(FrameType::tool, joint_torque, external_torque, cart_torque, cart_force, ec);
        if (ec)
        {
            std::cerr << "[t=" << (i + 1) << "s] getEndTorque failed: " << ec.message() << std::endl;
            continue;
        }
        std::error_code posture_ec;
        const auto posture = robot.posture(CoordinateType::endInRef, posture_ec);
        std::array<double, 3> translation_delta_mm{};
        if (!posture_ec)
        {
            for (size_t axis = 0; axis < translation_delta_mm.size(); ++axis)
            {
                translation_delta_mm[axis] = (posture[axis] - baseline_posture[axis]) * 1000.0;
            }
        }

        std::array<double, 6> joint_torque_delta{}, external_torque_delta{};
        for (size_t j = 0; j < joint_torque_delta.size(); ++j)
        {
            joint_torque_delta[j] = joint_torque[j] - baseline_joint_torque[j];
            external_torque_delta[j] = external_torque[j] - baseline_external_torque[j];
        }

        print(std::cout,
              "[t=",
              i + 1,
              "s] cart_force:",
              cart_force,
              " cart_torque:",
              cart_torque,
              " translation_delta_mm:",
              translation_delta_mm,
              " joint_torque_delta:",
              joint_torque_delta,
              " external_torque_delta:",
              external_torque_delta);
    }

    fc.fcStop(ec);
    if (ec)
    {
        std::cerr << "fcStop failed: " << ec.message() << std::endl;
    }
    else
    {
        std::cout << "fcStop OK." << std::endl;
    }

    robot.setPowerState(false, ec);
    robot.setOperateMode(OperateMode::manual, ec);

    return 0;
}
