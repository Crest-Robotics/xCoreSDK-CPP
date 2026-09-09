/**
 * @file cartesian_impedance_control.cpp
 * @brief Real-time mode - Cartesian impedance control
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>
#include "rokae/robot.h"
#include "../print_helper.hpp"
#include "rt_funtion_helper.hpp"
#include "../function_helper.hpp"

using namespace rokae;

/**
 * @brief main program
 */
int main() {
  using namespace std;
  rokae::xMateErProRobot robot; // ****   xMate 7-axis
  std::string robot_ip = "192.168.0.160"; // robot address
  std::string local_ip = "192.168.0.100"; // local address
  std::error_code ec;

  try {
    robot.connectToRobot(robot_ip, local_ip);
  } catch (const std::exception &e) {
    std::cerr << "Connect to robot failed " << e.what() << std::endl;
    return 0;
  }

  // Disable real-time mode
  robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
  if(ec) {
    std::cerr << "Set motion control mode failed " << ec.message() << std::endl;
    return 0;
  }
  // Automatic mode, power on
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setPowerState(true, ec);

  // Move to the starting position first, the drag-teaching pose for xMate Pro models
  MoveAbsJCommand start_joint({0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0}, 200, 0);
  std::string id;
  robot.moveAppend(start_joint, id, ec);
  robot.moveStart(ec);
  if(ec) {
    std::cerr << "Move failed " << ec.message() << std::endl;
    return 0;
  }
  // Wait for motion to finish
  helper::waitRobot(robot);

  // Set the real-time mode network tolerance to 50%. Must be set before starting real-time mode
  robot.setRtNetworkTolerance(50, ec);
  // Switch to real-time mode, power on
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  robot.setOperateMode(rokae::OperateMode::automatic,ec);
  robot.setPowerState(true, ec);

  std::shared_ptr<RtMotionControlCobot<7>> rtCon;

  try {
    rtCon = robot.getRtMotionController().lock();
  } catch (const std::exception &e) {
    std::cerr << "Get rt motion controller failed " << e.what() << std::endl;
    return 0;
  }

  // Set the force control coordinate system to the tool frame, the end-effector frame relative to the flange
  std::array<double, 16> toolToFlange = {0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1};
  rtCon->setFcCoor(toolToFlange, FrameType::tool, ec);
  // Set the Cartesian impedance coefficients
  rtCon->setCartesianImpedance({1200, 1200, 0, 100, 100, 0}, ec);
  // Set the desired force of 3N in the X and Z directions
  rtCon->setCartesianImpedanceDesiredTorque({3, 0, 3, 0, 0, 0}, ec);

  try {
    // Start receiving state data only after switching to real-time control mode, to ensure the data read is from real-time mode
    robot.startReceiveRobotState(std::chrono::milliseconds(8), {RtSupportedFields::tcpPose_m});
  } catch (const std::exception &e) {
    std::cerr << "Start receive robot state failed " << e.what() << std::endl;
    return 0;
  }
  // Get the current Cartesian pose for real-time mode, as the starting point
  std::array<double, 16> init_position = helper::getCurrentPose_matrix(robot);
  std::cout << "Initial position: " << init_position << std::endl;

  // Track which control cycle the planning has reached (i.e., which millisecond)
  double time = 0;

  std::atomic<bool> stopManually {true};
  // Define the callback function, containing the computation to perform each cycle, returning the computed Cartesian command
  std::function<CartesianPosition(void)> callback = [&, rtCon]()->CartesianPosition{
    time += 0.001; // Since the control period is fixed at 1kHz, this time is incremented by a fixed 0.001s each cycle

    // Example: smooth S-curve displacement planning based on a cosine function
    constexpr double kRadius = 0.2; // Maximum displacement amplitude is 0.2m

    // Initial velocity is 0, final velocity is 0; the intermediate acceleration-deceleration process is smooth, conforming to trajectory planning with continuous cosine jerk.
    double angle = M_PI / 4 * (1 - std::cos(M_PI / 2 * time));
    // As angle changes, it traces a half-period cosine waveform, forming smooth up-and-down motion.
    double delta_z = kRadius * (std::cos(angle) - 1);

    CartesianPosition output{};
    output.pos = init_position;
    // Add the per-cycle change onto the starting pose
    output.pos[7] += delta_z;

    // Run continuously for 40 seconds
    if(time > 40){
      std::cout << "Motion finished" <<std::endl;
      output.setFinished(); // Indicates that the returned cmd is the last command
      stopManually.store(false); // The loop is non-blocking, sync the stop state with the main thread
    }
    return output;
  };

  try {
    rtCon->setControlLoop(callback);
    // Start Cartesian impedance control
    rtCon->startMove(RtControllerMode::cartesianImpedance);
    rtCon->startLoop(false);
    while (stopManually.load());
    // Non-blocking loop, need to call stop loop once
    rtCon->stopLoop();
  } catch (const std::exception &e) {
    std::cerr << "RT move error occur " << e.what() << std::endl;
  }

  // Power off, disable real-time mode
  robot.setPowerState(false, ec);
  robot.setOperateMode(OperateMode::manual, ec);
  robot.setMotionControlMode(MotionControlMode::Idle, ec);

  return 0;
}
