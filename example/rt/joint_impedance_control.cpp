/**
 * @file joint_impedance_control.cpp
 * @brief Real-time mode - joint-space impedance control. Applicable robot model: xMateER7 Pro
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "../print_helper.hpp"
#include "../function_helper.hpp"
#include "rt_funtion_helper.hpp"

using namespace rokae;

/**
 * @brief main program
 */
int main() {
  using namespace std;
  rokae::xMateErProRobot robot;
  std::string robot_ip = "192.168.0.160"; // robot address
  std::string local_ip = "192.168.0.100"; // local machine address
  std::error_code ec;
  try {
    robot.connectToRobot(robot_ip, local_ip);
  } catch (const std::exception &e) {
    std::cout << "Connection error: " << e.what();
    return 0;
  }

  // Move to the start point using the MoveAbsJ command
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  if(ec) {
    std::cerr << "Switch MotionControlMode error: " << ec << std::endl;
    return 0;
  }
  robot.setOperateMode(rokae::OperateMode::automatic,ec);
  robot.setPowerState(true, ec);

  std::vector<double> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
  std::string id;
  // Speed 100mm/s, blend zone 0
  MoveAbsJCommand absj (q_drag_xm7p, 100, 0);
  robot.moveAppend(absj, id, ec);
  robot.moveStart(ec);
  if( ec) {
    std::cerr << "MoveAbsJ error: " << ec << std::endl;
    return 0;
  }
  helper::waitRobot(robot); // wait for the motion to finish

  // Then switch to real-time control
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  robot.setOperateMode(rokae::OperateMode::automatic,ec);
  robot.setPowerState(true, ec);

  std::shared_ptr<RtMotionControlCobot<7>> rtCon;
  try {
    rtCon = robot.getRtMotionController().lock();
    // Set up the data to receive
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});
  } catch (const std::exception &e) {
    std::cout << e.what();
    return 0;
  }

  double time = 0; // cycle counter [seconds]
  std::array<double, 7> jntPos {};

  // Callback function
  std::function<JointPosition(void)> callback = [&jntPos, rtCon, &time] {
    time += 0.001;
    double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI/4 * time));

    JointPosition cmd(7);
    for(unsigned i = 0; i < cmd.joints.size(); ++i) {
      cmd.joints[i] = jntPos[i] + delta_angle;
    }

    if(time > 60) {
      cmd.setFinished(); // finish after 60 seconds
    }
    return cmd;
  };

  // Set the joint-space impedance coefficients
  rtCon->setJointImpedance({500, 500, 500, 500, 50, 50, 50}, ec);
  // Set the callback function
  rtCon->setControlLoop(callback);
  // Update the starting position to the current position
  jntPos = helper::getCurrentJointPos(robot);

  try {
    // Start joint-space impedance motion
    rtCon->startMove(RtControllerMode::jointImpedance);
    // Blocking loop
    rtCon->startLoop(true);
    print(std::cout, "Control finished");
  } catch (const std::exception &e) {
    std::cout << "Error during motion: " << e.what();
  }

  // Power off, switch to non-real-time mode
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setPowerState(false, ec);
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);

  return 0;
}
