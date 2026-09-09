/**
 * @file read_robot_state.cpp
 * @brief Example of reading robot state data
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#include <thread>
#include <atomic>
#include <fstream>
#include "rokae/utility.h"
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace std;
using namespace rokae;

void WaitRobot(BaseRobot *robot);

/**
 * @brief main program
 */
int main() {
  try {
    using namespace RtSupportedFields;
    xMateRobot robot("192.168.0.160", "192.168.0.100");
    error_code ec;
    std::ostream &os = std::cout;
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);

    // Set the state data push interval to 1s, receiving the robot's end-effector pose, joint torque, and joint angles
    robot.startReceiveRobotState(chrono::seconds(1), {tcpPoseAbc_m, tau_m, jointPos_m});
    std::array<double, 6> tcpPose{};
    std::array<double, 6> arr6{};

    std::atomic_bool running{true};

    // The queue holding received state data is not automatically overwritten with old data cleared; loop-reading can be used to clear out the old data
    while (robot.updateRobotState(chrono::steady_clock::duration::zero()));
    // Output to file
    std::ofstream ofs;
    ofs.open(("read_" + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count()) + ".csv"), std::ios::out);

    // Print the end-effector pose and joint angles to the console
    std::thread readState([&] {
      while (running) {
        // Periodically retrieve the current state data; the timeout parameter should ideally match the configured data push interval
        // or read according to the push frequency
        robot.updateRobotState(chrono::seconds(1));
        robot.getStateData(tcpPoseAbc_m, tcpPose);
        robot.getStateData(jointPos_m, arr6);
        ofs << tcpPose[0] << "," << tcpPose[1] << "," << tcpPose[2] << ","
          << tcpPose[3] << "," << tcpPose[4] << "," << tcpPose[5] << ",,"
         << arr6[0] << ","<< arr6[1] << ","<< arr6[2] << ","
          << arr6[3] << ","<< arr6[4] << ","<< arr6[5] <<std::endl;
//        print(os, "Ts:", ts, "TCP pose:", pose, "\nJoint:", Utils::radToDeg(jnt));
      }
    });

    // Start a motion thread
    std::thread moveThread([&]{
      robot.setOperateMode(rokae::OperateMode::automatic, ec);
      robot.setPowerState(true, ec);
      MoveAbsJCommand p1({0,0,0,0,0,0}), p2({0, M_PI/6, M_PI/3, 0, M_PI_2, 0});
      std::string id;
      robot.moveAppend({p1, p2}, id, ec);
      robot.moveStart(ec);
      WaitRobot(&robot);
    });

    // Wait for the motion to finish
    moveThread.join();
    running = false;
    readState.join();

    // Stop the controller from sending state data
    robot.stopReceiveRobotState();

  } catch(const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}

/**
 * @brief Wait for the robot to stop
 */
void WaitRobot(BaseRobot *robot) {
  bool checking = true;
  while (checking) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    error_code ec;
    auto st = robot->operationState(ec);
    if(st == OperationState::idle || st == OperationState::unknown){
      checking = false;
    }
  }
}