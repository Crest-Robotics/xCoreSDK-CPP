/**
 * @file joint_s_line.cpp
 * @brief Real-time mode - joint-space S-curve planning. Applicable robot model: xMateER7 Pro
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

using namespace rokae;

/**
 * @brief main program
 */
int main() {
  rokae::xMateErProRobot robot;
  try {
    robot.connectToRobot("192.168.0.160", "192.168.0.100"); // local machine address 192.168.0.100
  } catch(const std::exception &e) {
    std::cerr << "Connection failed " << e.what();
    return -1;
  }

  std::error_code ec;

  robot.setOperateMode(rokae::OperateMode::automatic,ec);
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  robot.setPowerState(true, ec);
  std::shared_ptr<RtMotionControlCobot<7>> rtCon;
  try {
    rtCon = robot.getRtMotionController().lock();
    // Set up the data to receive
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});
  } catch(const std::exception &e) {
    std::cerr << "Failed to initialize real-time control" << e.what();
    return -1;
  }

  std::array<double,7> jntPos{}, delta{};
  JointPosition cmd(7);

  double time = 0;

  // 6 target points
  std::vector<std::array<double, 7>> jntTargets = {
    {0, M_PI/6, 0, M_PI/3, 0, M_PI_2, 0},
    {0, -0.078, 0, 1.836, 0, 1.003, 0},
    {0, -0.054, 0, 1.331, 0, 1.485, 0},
    {0, 0.330, 0, 0.887, 0, 1.544, 0},
    {0, 0.150, 0, 1.201, 0, 1.156, 0},
    {0, 0.354, 0, 1.328, 0, 0.824, 0}
  };
  auto it = jntTargets.begin();

  std::function<JointPosition(void)> callback = [&, rtCon]() {
    time += 0.001; // plan with a 1ms control cycle

    JointMotionGenerator joint_s(0.8, *it);
    joint_s.calculateSynchronizedValues(jntPos);

    // Get the angle offset computed for each control cycle
    if (!joint_s.calculateDesiredValues(time, delta)) {
      for(unsigned i = 0; i < cmd.joints.size(); ++i) {
        cmd.joints[i] = jntPos[i] + delta[i];
      }
    } else {
      // Reached one target point, start moving to the next target point
      if (++it == jntTargets.end()) {
        cmd.setFinished();
      }
      time = 0;
      // Use the final angle value as the starting point for the next planning
      std::copy(cmd.joints.begin(), cmd.joints.end(), jntPos.begin());
    }
    return cmd;
  };

  try {
    rtCon->setControlLoop(callback);
    // Update the callback's starting position
    jntPos = robot.jointPos(ec);
    // Set to joint-space position control before starting motion
    rtCon->startMove(RtControllerMode::jointPosition);
    rtCon->startLoop(true);
    print(std::cout, "Control finished");

  } catch (const std::exception &e) {
    print(std::cerr, "Real-time motion error", e.what());
  }

  return 0;
}
