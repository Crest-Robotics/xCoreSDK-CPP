/**
 * @file follow_cart_position.cpp
 * @brief Real-time mode - Cartesian waypoint following feature
 * This feature requires the xMateModel model library; please set the build option XCORE_USE_XMATE_MODEL=ON
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#include <thread>
#include <atomic>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "../print_helper.hpp"
#include "../function_helper.hpp"

using namespace rokae;

namespace {
std::atomic_bool running = true; ///< running state flag
std::ostream &os = std::cout; ///< print to console
std::vector<double> q_drag_xm7p = { 0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0 }; ///< xMateER Pro drag-teaching pose
std::vector<double> q_drag_er3 = { 0, M_PI/6, M_PI/3, 0, M_PI/2, 0 }; ///< xMateER drag-teaching pose
std::vector<double> q_drag_sr_cr = { 0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0 }; ///< SR and CR drag-teaching pose

xMateRobot robot;
}

template <unsigned short DoF>
void updatePose(rokae::FollowPosition<DoF> &fp, const Eigen::Transform<double, 3, Eigen::Isometry> &start){
  auto rtCon = robot.getRtMotionController().lock();
  using namespace std::chrono;
  double count = 0;
  // Scale factor of 0.2
  fp.setScale(0.2);
  auto transform = start;
  while(running) {
    count += 3;
    // Simulate updating the position once per second
    std::this_thread::sleep_for(std::chrono::seconds(1));
    transform.translation().y() = start.translation().y() + 0.4 * sin(M_PI / 2 * count);
    fp.update(transform);

    if(rtCon->hasMotionError()) {
      print(std::cerr, "An error occurred during motion");
      running = false;
    }
  }
}

/**
 * @brief Waypoint following example
 */
void followCart_Example() {
  using namespace rokae::RtSupportedFields;

  error_code ec;
  std::thread updater;
  std::thread inputer;

  auto model = robot.model();
  std::shared_ptr<RtMotionControlCobot<6>> rtCon;
  try {
    rtCon = robot.getRtMotionController().lock();
  } catch (const std::exception &e) {
    std::cerr << e.what();
    return;
  }

  // Optional: set smoothing filter. Recommended when running under Windows
  rtCon->setFilterFrequency(10, 10, 10, ec);
  rtCon->setFilterLimit(true, 10);

  // Cartesian starting point
  auto cart_pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
  print(std::cout, "Start from", cart_pose);
  // Compute the quaternion
  auto quaternion = Utils::eulerToQuaternion(cart_pose.rpy);

  FollowPosition follow_pose(robot, model);
  Eigen::Transform<double, 3, Eigen::Isometry> start_pose = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
  start_pose.rotate(Eigen::Quaterniond(quaternion[0], quaternion[1], quaternion[2], quaternion[3]));
  start_pose.pretranslate(Eigen::Vector3d(cart_pose.trans[0], cart_pose.trans[1], cart_pose.trans[2]));

  running = true;
  print(os, "Start following");
  follow_pose.start(start_pose);
  updater = std::thread([&]() {
    updatePose(follow_pose, start_pose);
  });

  inputer = std::thread([]{
    // press 'q' to stop
    print(os, "Enter 'q' to stop following");
    while (getchar() != 'q');
    running = false;
  });
  inputer.detach();

  // Wait to finish (either due to an error or a manual stop)
  while(running);

  try {
    follow_pose.stop();
  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }

  updater.join();

}

/**
 * @brief main program
 */
int main() {
  using namespace rokae;
  using namespace rokae::RtSupportedFields;

  std::string remoteIP = "192.168.0.160";
  std::string localIP = "192.168.0.100";
  error_code ec;
  std::thread updater;

  try {
    robot.connectToRobot(remoteIP, localIP);
  } catch (const std::exception &e) {
    std::cerr << "Connection failed" << e.what();
    return 0;
  }

  // Move to a suitable starting point first
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  if(ec) {
    std::cerr << "Switch MotionControlMode error: " << ec << std::endl;
    return 0;
  }
  // Move to a different starting joint angle depending on the robot model name
  std::string robot_name = robot.robotInfo(ec).type;
  std::vector<double> start_joint {};
  if(robot_name.find("CR") != std::string::npos || robot_name.find("XMC") != std::string::npos ||
    robot_name.find("SR") != std::string::npos || robot_name.find("XMS") != std::string::npos) {
    start_joint = q_drag_sr_cr;
  } else if(robot_name.find("xMate3") != std::string::npos || robot_name.find("xMate7") != std::string::npos) {
    start_joint = q_drag_er3;
  } else {
    std::cerr << "Unsupported robot type: " << robot_name << std::endl;
    return 0;
  }

  // Power on
  robot.setOperateMode(OperateMode::automatic, ec);
  robot.setPowerState(true, ec);

  // Use the MoveAbsJ command to reach the starting point
  std::string id;
  MoveAbsJCommand absj (start_joint, 100, 0);
  robot.moveAppend(absj, id, ec);
  robot.moveStart(ec);
  if( ec) {
    std::cerr << "MoveAbsJ error: " << ec << std::endl;
    return 0;
  }
  helper::waitRobot(robot);

  // Switch to real-time control mode
  robot.setRtNetworkTolerance(60, ec);
  robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setPowerState(true, ec);

  // Start real-time data reception
  try {
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {jointPos_m});
  } catch (const std::exception &e) {
    std::cerr << e.what();
    return 0;
  }

  // Run the Cartesian waypoint following example
  followCart_Example();

  // Motion finished, disable real-time mode, power off
  robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
  robot.setPowerState(false, ec);
  robot.setOperateMode(rokae::OperateMode::manual, ec);

  return 0;
}