/**
 * @file joint_position_control.cpp
 * @brief Real-time mode - joint angle control. Applicable robot model: xMateER3
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

using namespace rokae;

/**
 * @brief main program
 */
int main() {
  using namespace std;

  // Create the robot object
  rokae::xMateRobot robot;
  std::error_code ec;

  // Connect to the robot
  try {
    robot.connectToRobot("192.168.0.160", "192.168.0.100"); // local machine address 192.168.0.100
  } catch (const std::exception &e) {
    print(std::cerr, e.what());
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

  std::vector<double> q_drag_xm3 = {0, M_PI/6, M_PI/3, 0, M_PI/2, 0};
  std::string id;
  MoveAbsJCommand absj (q_drag_xm3, 100, 0);
  robot.moveAppend(absj, id, ec);
  robot.moveStart(ec);
  if( ec) {
    std::cerr << "MoveAbsJ error: " << ec << std::endl;
    return 0;
  }
  helper::waitRobot(robot);

  // Switch to real-time mode
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setPowerState(true, ec);

  try {
    auto rtCon = robot.getRtMotionController().lock();

    // Optional: set the filter cutoff frequency. If abnormal noise, slight vibration, or similar issues occur
    // during motion, after ruling out causes such as network jitter or unsuitable trajectory planning, filtering
    // can be set to smooth the commands and reduce the noise. The lower the value, the smoother the effect.
    rtCon->setFilterLimit(true, 10);
    rtCon->setFilterFrequency(10, 10, 10, ec);

    // Define and initialize the motion control time and angle
    double time = 0;

    std::array<double, 6> jntPos{};

    // Define the callback function
    std::function<JointPosition()> callback = [&, rtCon](){
      time += 0.001;
      // Cosine-interpolation S-curve example
      double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI / 2.5 * time));
      JointPosition cmd = {{jntPos[0] + delta_angle, jntPos[1] + delta_angle,
                            jntPos[2] - delta_angle,
                            jntPos[3] + delta_angle, jntPos[4] - delta_angle,
                            jntPos[5] + delta_angle}};

      if(time > 60) {
        cmd.setFinished(); // finish after 60 seconds
      }
      return cmd;
    };

    // Set the callback function
    rtCon->setControlLoop(callback);
    // Update the starting angle to the current angle
    jntPos = robot.jointPos(ec);
    // Start joint-space position control
    rtCon->startMove(RtControllerMode::jointPosition);
    // Blocking loop, start motion
    rtCon->startLoop(true);
    print(std::cout, "Control finished");

    // Set the control mode to idle and power off
    robot.setMotionControlMode(MotionControlMode::Idle, ec);
    robot.setPowerState(false, ec);

  } catch (const std::exception &e) {
    // Catch the exception and print the error message
    print(std::cerr, e.what());
    robot.setMotionControlMode(MotionControlMode::Idle, ec);
    robot.setPowerState(false,ec);
  }

  return 0;
}
