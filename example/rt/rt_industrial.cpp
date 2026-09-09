/**
 * @file rt_industrial.cpp
 * @brief Real-time mode - position control support for six-axis industrial robot models
 * @attention MoveJ/MoveL/MoveC in real-time mode are no longer recommended; please use
 * MoveAbsJCommand/MoveLCommand/MoveCCommand in non-real-time mode instead.
 * This example only demonstrates how to call these functions.
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#include <cmath>
#include <thread>
#include <iterator>
#include "../print_helper.hpp"
#include "Eigen/Geometry"
#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace rokae;

std::ostream &os = std::cout; ///< print to console

/**
 * @brief main program
 */
int main() {
  using namespace std;
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::StandardRobot robot(ip, "192.168.0.180");
    robot.setOperateMode(rokae::OperateMode::automatic,ec);
    robot.setRtNetworkTolerance(20, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);

    robot.setPowerState(true, ec);
    auto rtCon = robot.getRtMotionController().lock();

    // Reset the end-effector frame to coincide with the flange
    std::array<double, 16> _end{};
    Utils::postureToTransArray({0,0,0,0,0,0}, _end);
    rtCon->setEndEffectorFrame(_end, ec);

    // Example program uses robot model: XB7h-R707
    // ***** 1. MoveJ from the current position to the shipping position *****
    rtCon->MoveJ(0.4, robot.jointPos(ec), Utils::degToRad(std::array<double, 6>({0, -15, 60, 0, 45, 0})));

    // ***** 2. Circular arc motion (on the X-Y plane) *****
    CartesianPosition start, aux, target;
    Utils::postureToTransArray(robot.posture(rokae::CoordinateType::endInRef, ec), start.pos);
    Eigen::Matrix3d rot_start;
    Eigen::Vector3d trans_start, trans_aux, trans_end;
    Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
    trans_end = trans_start;
    trans_aux = trans_start;
    // Auxiliary point X+0.28, Y-0.05; target point Y-0.15
    trans_aux[0] += 0.28;
    trans_aux[1] -= 0.05;
    trans_end[1] -= 0.15;

    Utils::transMatrixToArray(rot_start, trans_aux, aux.pos);
    Utils::transMatrixToArray(rot_start, trans_end, target.pos);
    rtCon->MoveC(0.2, start, aux, target);

#if 0
    // ***** Example: set the safety zone *****
    // Using the current pose as the center of the safety zone: X length 1m, Y length 0.8m, Z length 0.1m
    // If set successfully, the robot will power off if the Z value of the subsequent MoveL target point exceeds the safety zone
    std::array<double, 16> _centre{};
    Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), _centre);
    rtCon->setCartesianLimit({1, 0.8, 0.1}, _centre, ec);
#endif

    // ***** 3. Linear motion *****
    auto _pose_start = robot.posture(rokae::CoordinateType::endInRef, ec);
    auto _pose_target = _pose_start;
    // Along Z+0.2m, rotate about Ry+60°
    _pose_target[2] += 0.2;
    _pose_target[4] += Utils::degToRad(60);
    Utils::postureToTransArray(_pose_start, start.pos);
    Utils::postureToTransArray(_pose_target, target.pos);

    print(os, "MoveL start position:", start.pos, "Target:", target.pos);

    rtCon->MoveL(0.3, start, target);

    // ***** 4. Set the end-effector frame in real-time mode *****
    Utils::postureToTransArray(std::array<double, 6>({0.1, 0, 0, 0, M_PI_2, 0}), _end);
    rtCon->setEndEffectorFrame(_end, ec);
    rtCon->MoveJ(0.4, robot.jointPos(ec), Utils::degToRad(std::array<double, 6>({0, -15, 60, 0, 45, 0})));

    // Note: the tool frame setting in real-time mode is independent, so the robot.posture(CoordinateType::endInRef)
    // interface cannot be used to get the end-effector pose;
    // the example below directly gives the starting pose after setting the end-effector frame;
    // alternatively, real-time state data can be received and obtained via robot.getStateData(RtSupportedFields::tcpPose_m, start.pos)
    Utils::postureToTransArray({0.1036, 0,0.415, 0.042, -M_PI_2, -0.0424}, start.pos);
    target.pos = start.pos;
    target.pos[3] += 0.35;
    print(os, "MoveL start position:", start.pos, "Target:", target.pos);
    rtCon->MoveL(0.3, start, target);

    // ***** 5. Turn off real-time mode *****
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec);

  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}