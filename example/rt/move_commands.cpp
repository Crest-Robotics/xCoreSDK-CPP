/**
 * @file move_commands.cpp
 * @brief Real-time mode - S-curve planning MoveJ & MoveL & MoveC
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
#include <iostream>
#include <thread>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "../print_helper.hpp"

using namespace rokae;

/**
 * @brief main program
 */
int main() {
  using namespace std;
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateErProRobot robot(ip, "192.168.0.180"); // ****   XMate 7-axis
    robot.setOperateMode(rokae::OperateMode::automatic,ec);
    // If the controller is already in real-time mode when the program runs, switch to non-real-time mode
    // first before changing the network latency threshold, otherwise it will not take effect
    robot.setRtNetworkTolerance(20, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    auto rtCon = robot.getRtMotionController().lock();

    // Example program uses robot model: xMateER7 Pro
    // 1. MoveJ from the current position to the drag-teach position
    std::array<double, 7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
    rtCon->MoveJ(0.5, robot.jointPos(ec), q_drag_xm7p);

    // 2. Circular arc motion (on the X-Y plane)
    CartesianPosition start, aux, target;
    Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
    Eigen::Matrix3d rot_start;
    Eigen::Vector3d trans_start, trans_aux, trans_end;
    Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
    trans_end = trans_start; trans_aux = trans_start;
    trans_aux[0] -= 0.28;
    trans_aux[1] -= 0.05;
    trans_end[1] -= 0.15;

    Utils::transMatrixToArray(rot_start, trans_aux, aux.pos);
    Utils::transMatrixToArray(rot_start, trans_end, target.pos);
    rtCon->MoveC(0.2, start, aux, target);

    // 3. Linear motion
    Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
    Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);

    trans_end = trans_start;
    // Along x-0.1m, y-0.3m, z-0.25
    trans_end[0] -= 0.1;
    trans_end[1] -= 0.3;
    trans_end[2] -= 0.25;
    Utils::transMatrixToArray(rot_start, trans_end, target.pos);

    print(std::cout, "MoveL start position:", start.pos, "Target:", target.pos);
    rtCon->MoveL(0.3, start, target);

    // 4. Turn off real-time mode
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec);

  } catch (const std::exception &e) {
    std::cerr << e.what();
  }
  return 0;
}