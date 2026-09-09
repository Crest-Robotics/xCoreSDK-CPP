/**
 * @file cartesian_s_line.cpp
 * @brief Real-time mode - Cartesian space S-curve planning. Applicable model: xMateER7 Pro
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

  try {
    // Robot address 192.168.0.160, local address 192.168.0.100
    robot.connectToRobot("192.168.0.160", "192.168.0.100");
  } catch(const rokae::Exception &e) {
    std::cerr << "Connection failed " << e.what();
    return 0;
  }

  std::error_code ec;

  robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
  if(ec) {
    std::cerr << "Set motion control mode failed " << ec.message() << std::endl;
    return 0;
  }
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setPowerState(true, ec);
  // Move to the starting position first, the drag-teaching pose for xMate Pro models
  MoveAbsJCommand start_joint({0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0}, 200, 0);
  std::string id;
  robot.moveAppend(start_joint, id, ec);
  robot.moveStart(ec);
  helper::waitRobot(robot);

  // Switch to real-time mode, power on
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  robot.setOperateMode(rokae::OperateMode::automatic,ec);
  robot.setPowerState(true, ec);

  std::shared_ptr<RtMotionControlCobot<7>> rtCon;
  try {
    rtCon = robot.getRtMotionController().lock();
  } catch (const std::exception &e) {
    std::cerr << "Get rt motion controller failed " << e.what();
    return 0;
  }

  try {
    // Start receiving state data only after switching to real-time mode
    robot.startReceiveRobotState(std::chrono::milliseconds(1),
                                 {RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m});
  } catch (const std::exception &e) {
    std::cerr << "Receive rt state data failed " << e.what();
    return 0;
  }

  std::array<double, 16> init_pos{}, end_pos{};
  Eigen::Quaterniond rot_cur;
  Eigen::Matrix3d mat_cur;
  double delta_s;

  static bool init = true;
  double time = 0;

  std::function<CartesianPosition()> callback = [&, rtCon]() {
    time += 0.001; // Plan with a 1ms period
    if(init) {
      // Read the current position
      // Note: only the first cycle may send the position that was read, so the robot starts moving from its current position. Subsequent cycles must not send the read position as a command.
      robot.getStateData(RtSupportedFields::tcpPose_m, init_pos);
      end_pos = init_pos;
      end_pos[11] -= 0.2; // Set the target of this motion segment to be 0.2m in the negative Z direction from the starting position
      init = false;
    }

    std::array<double, 16> pose_start = init_pos;

    // Extract the starting position pos_1 and target position pos_2
    Eigen::Vector3d pos_1(pose_start[3], pose_start[7], pose_start[11]);
    Eigen::Vector3d pos_2(end_pos[3], end_pos[7], end_pos[11]);

    // Compute the total path vector pos_delta and path length s.
    Eigen::Vector3d pos_delta = pos_2 - pos_1;
    double s = pos_delta.norm();
    Eigen::Vector3d pos_delta_vector = pos_delta.normalized();

    // s -> total path length
    CartMotionGenerator cart_s(0.05, s);
    // Synchronize the arc length already traveled, which is 0
    cart_s.calculateSynchronizedValues(0);

    // Extract the rotation part from the starting and target homogeneous matrices and convert to a Quaternion.
    // Quaternion slerp will be used afterward for smooth rotation interpolation
    Eigen::Matrix3d mat_start, mat_end;
    mat_start << pose_start[0], pose_start[1], pose_start[2], pose_start[4], pose_start[5], pose_start[6],
      pose_start[8], pose_start[9], pose_start[10];

    mat_end << end_pos[0], end_pos[1], end_pos[2], end_pos[4], end_pos[5], end_pos[6], end_pos[8],
      end_pos[9], end_pos[10];

    Eigen::Quaterniond rot_start(mat_start);
    Eigen::Quaterniond rot_end(mat_end);

    Eigen::Vector3d pos_cur;
    CartesianPosition cmd;

    // Compute the distance traveled delta_s based on time
    // If the endpoint has not been reached, return false
    // If the endpoint has been reached, return true and set the command as the last command
    if (!cart_s.calculateDesiredValues(time, &delta_s)) {
      // Position interpolation: move along the straight line between start and end, by the fraction delta_s.
      pos_cur = pos_1 + pos_delta * delta_s / s;
      // Orientation interpolation: use quaternion slerp for a smooth rotation transition
      Eigen::Quaterniond rot_cur = rot_start.slerp(delta_s / s, rot_end);
      mat_cur = rot_cur.normalized().toRotationMatrix();

      // Finally build the 4x4 homogeneous matrix and write it into cmd.pos
      std::array<double, 16> new_pose = {
        {mat_cur(0, 0), mat_cur(0, 1), mat_cur(0, 2), pos_cur(0), mat_cur(1, 0), mat_cur(1, 1),
         mat_cur(1, 2), pos_cur(1), mat_cur(2, 0), mat_cur(2, 1), mat_cur(2, 2), pos_cur(2), 0, 0, 0, 1}};
      cmd.pos = new_pose;
    } else {
      cmd.setFinished();
    }
    return cmd;
  };

  try {
    // Since getStateData() is used in the callback, the parameter useStateDataInLoop=true
    rtCon->setControlLoop(callback, 0, true);
    // Set to Cartesian space position control before starting motion
    rtCon->startMove(RtControllerMode::cartesianPosition);
    rtCon->startLoop(true);
    print(std::cout, "Control finished");
  } catch (const std::exception &e) {
    std::cerr << "Error occurred during motion " << e.what();
  }

  // Power off, disable real-time mode
  robot.setPowerState(false, ec);
  robot.setOperateMode(OperateMode::manual, ec);
  robot.setMotionControlMode(MotionControlMode::Idle, ec);

  return 0;
}
