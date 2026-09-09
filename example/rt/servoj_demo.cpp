/**
 * @file servoj_demo.cpp
 * @brief Real-time mode - servoj feature demo
 * This example only demonstrates how to call it.
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#include <iostream>
#include <cmath>
#include <thread>
#include <ctime>
#include <cerrno>
#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace rokae;

// User command dispatch period (s)
constexpr double planPeriod = 0.02;

// Busy-wait delay function
void busy_wait(int milliseconds) {
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start + std::chrono::milliseconds(milliseconds);
    while (std::chrono::high_resolution_clock::now() < end) {
    }
}

int main() {
    using namespace std;
    rokae::xMateRobot robot;
    std::error_code ec;
    try {
        robot.connectToRobot("192.168.2.160", "192.168.2.161");//robot IP, host PC IP
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 0;
    }
    
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    // Required: enable real-time mode
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    try {
        auto rtCon = robot.getRtMotionController().lock();
        // Set up the data to receive
        robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});

        std::array<double, 6> jntPos{};
        std::array<double, 6> q_drag_xm3 = { M_PI/3, M_PI/6,  M_PI/6,  M_PI/6, M_PI/2,  M_PI/2};
        std::array<double, 6> q2_drag_xm3 ={0, 0,  0, 0, 0, 0};

        while(robot.updateRobotState(std::chrono::steady_clock::duration::zero()));
        
        jntPos = robot.jointPos(ec);
        // Move to the drag-teach position
        rtCon->MoveJ(0.3, robot.jointPos(ec), q_drag_xm3);
        // Required: enable the servoj feature
        rtCon->setServoJoint(planPeriod,planPeriod*3,1,ec);
        jntPos = robot.jointPos(ec);
        // Required: set the motion mode, note this must come after enabling the servoj feature
        rtCon->startMove(RtControllerMode::jointPosition);
        
        auto start = std::chrono::steady_clock::now();
        
        while(true) {
            robot.updateRobotState(std::chrono::milliseconds(1));
            // Get the current time
            auto now = std::chrono::steady_clock::now();
            double elapsed_seconds = std::chrono::duration<double>(now - start).count();
            // Compute the target position
            double delta_angle = M_PI / 30.0 * (1 - std::cos(M_PI / 2.5 * elapsed_seconds));
            JointPosition cmd = {{jntPos[0] + delta_angle, jntPos[1] + delta_angle,
                                  jntPos[2] - delta_angle,
                                  jntPos[3] + delta_angle, jntPos[4] - delta_angle,
                                  jntPos[5] + delta_angle}};
            // Required: send the computed joint position
            rtCon->sendCommand(cmd);
            // Required: send at intervals of the command period
            busy_wait((int)(planPeriod*1000));

            // Check whether it's finished
            if ( elapsed_seconds > 30) {
                cmd.setFinished();
                rtCon->sendCommand(cmd);
                // Required: disable the servoj feature
                rtCon-> stopServoJoint();
                break;
            }
        }

    while(robot.updateRobotState(std::chrono::steady_clock::duration::zero()));
    rtCon->MoveJ(0.3, robot.jointPos(ec), q2_drag_xm3);
    std::cout << "Control finished" << std::endl;

    // Turn off real-time mode
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec);

    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}
