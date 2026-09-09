/**
 * @file modbusRTU_endtool_control.cpp
 * @brief Transparent-transmission protocol for reading/writing control of the end-effector tool
 * @note  Different vendors' end-effector tools send different data; please modify the data structure according to this demo
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;

namespace DhGripParams {
 std::vector<uint8_t> send_data = { 0x01,0x06,0x01,0x00,0x00,0xA5,0x48,0x4D };//initialization raw-transmission data
 std::vector<uint8_t> rev_data = { 0,0,0,0,0,0,0,0 };//received raw-transmission response data
 std::vector<int>  init_set = { 0xA5 };//initialization data
 std::vector<int>  init_get = { 100 };//get initialization status  0-2
 std::vector<int>  grip_status_get = { 0 };//get grip status  0-3
 std::vector<int>  trq_set = { 100 };//set force 20-100, can be changed
 std::vector<int>  vel_set = { 2 };//set speed 1-100, can be changed
 std::vector<int>  pos_set = { 100 };//set position 0-1000, can be changed
 std::vector<int>  trq_get = { 0 };//stores the retrieved force
 std::vector<int>  vel_get = { 0 };//stores the retrieved speed
 std::vector<int>  pos_get = { 0 };//stores the retrieved position
 std::vector<int>  pos_now_get = { 0 };//stores the retrieved real-time position
}

/**
 * @brief Initialize the DH gripper
 */
void DHGripInit(xMateRobot &robot) {
  error_code ec;
  // Raw-transmission initialization
  std::vector<int>  init_set = { 0xA5 };//initialization data
  robot.XPRWModbusRTUReg(1, 0x06, 0x0100, "int16", 1, init_set, false, ec);
  std::cout << "DH initialization query result: " << ec << std::endl;
}

/**
 * @brief Move the DH gripper to a target
 */
void DHGripMove(xMateRobot &robot, int& trq_set, int& vel_set, int& pos_set) {
  error_code ec;
  //set force
  std::vector<int>  trq_set_vec;//set force 20-100, can be changed
  trq_set_vec.push_back(trq_set);
  robot.XPRWModbusRTUReg(0x01, 0x06, 0x0101, "int16", 0x01, trq_set_vec, false, ec);
  std::cout << "Set force result: " << ec << std::endl;
  //set speed
  std::vector<int>  vel_set_vec;//set speed 1-100, can be changed
  vel_set_vec.push_back(vel_set);
  robot.XPRWModbusRTUReg(0x01, 0x06, 0x0104, "int16", 0x01, vel_set_vec, false, ec);
  std::cout << "Set speed result: " << ec << std::endl;
  //set position
  std::vector<int>  pos_set_vec;//set position 0-1000, can be changed
  pos_set_vec.push_back(pos_set);
  robot.XPRWModbusRTUReg(0x01, 0x06, 0x0103, "int16", 0x01, pos_set_vec, false, ec);
  std::cout << "Set position result: " << ec << std::endl;
}

/**
 * @brief Get the DH gripper's initialization status
 */
void DHGripGetInitStatus(xMateRobot& robot,int& init_get) {
  error_code ec;
  std::vector<int>  init_get_vec = { -1 };//get initialization status  0-2
  robot.XPRWModbusRTUReg(0x01, 0x03, 0x0200, "int16", 0x01, init_get_vec, false, ec);
  init_get = init_get_vec[0];
  std::cout << "Get initialization status result: " << ec << std::endl;
}

/**
 * @brief Get the DH gripper's grip status
 */
void DHGripGetStatus(xMateRobot& robot, int& grip_status_get) {
  error_code ec;
  std::vector<int>  grip_status_get_vec = { -1 };//get grip status  0-3
  robot.XPRWModbusRTUReg(0x01, 0x03, 0x0201, "int16", 0x01, grip_status_get_vec, false, ec);
  grip_status_get = grip_status_get_vec[0];
  std::cout << "Get initialization status result " << ec << std::endl;
}

/*
 * @brief Get the DH gripper's torque, speed, and position information
 */
void DHGripGetInfo(xMateRobot& robot, int& trq_get, int& vel_get, int& pos_get) {
  error_code ec;
  //get force
  std::vector<int>  trq_get_vec = { 0 };//buffer for the retrieved force
  robot.XPRWModbusRTUReg(0x01, 0x03, 0x0101, "int16", 0x01, trq_get_vec, false, ec);
  trq_get = trq_get_vec[0];
  std::cout << "Get force result: " << ec << std::endl;
  //get speed
  std::vector<int>  vel_get_vec = { 0 };//buffer for the retrieved speed
  robot.XPRWModbusRTUReg(0x01, 0x03, 0x0104, "int16", 0x01, vel_get_vec, false, ec);
  vel_get = vel_get_vec[0];
  std::cout << "Get speed result: " << ec << std::endl;
  //get position
  std::vector<int>  pos_get_vec = { 0 };//buffer for the retrieved position
  robot.XPRWModbusRTUReg(0x01, 0x03, 0x0103, "int16", 0x01, pos_get_vec, false, ec);
  pos_get = pos_get_vec[0];
  std::cout << "Get position result: " << ec << std::endl;
}

/**
 * @brief Get the DH gripper's real-time position
 */
void DHGripGetNewPos(xMateRobot& robot, int& pos_now_get) {
  error_code ec;
  std::vector<int> pos_now_get_vec = { 0 };
  robot.XPRWModbusRTUReg(1, 0x03, 0x0202, "int16", 1, pos_now_get_vec, false, ec);
  pos_now_get = pos_now_get_vec[0];
  std::cout << "Get real-time position result: " << ec << std::endl;
}

/**
 * @brief Coil test interface, for suction cups
 */
void CoilTest(xMateRobot& robot) {
  error_code ec;
  std::vector<bool> bool_data_len1 = { 0 };
  std::vector<bool> bool_data_len4 = { 0,0,0,1 };
  std::vector<bool> bool_data_len10 = { 1,1,1,1,1,1,1,1,1,1 };

  //0x01 and 0x02 are similar
  robot.XPRWModbusRTUCoil(0x01, 0x01, 0x0001, 1, bool_data_len1, false, ec);
  std::cout << "coiltest 0x01 code:" << ec << std::endl;

  robot.XPRWModbusRTUCoil(0x01, 0x02, 0x0001, 1, bool_data_len1, false, ec);
  std::cout << "coiltest 0x02 code:" << ec << std::endl;

  //0x05 writes a single coil; parameter 3's length can only be 1
  robot.XPRWModbusRTUCoil(0x01, 0x05, 0x0001, 1, bool_data_len4, false, ec);
  std::cout << "coiltest 0x05 code:" << ec << std::endl;

  //0x10 writes multiple coils
  robot.XPRWModbusRTUCoil(0x01, 0x0F, 0x0001, 4, bool_data_len4, false, ec);
  std::cout << "coiltest 0x10 code:" << ec << std::endl;

  robot.XPRWModbusRTUCoil(0x01, 0x0F, 0x0001, 10, bool_data_len10, false, ec);
  std::cout << "coiltest 0x10 code:" << ec << std::endl;
}

/**
 * @brief func_code register test interface
 */
void RegFuncCodeTest(xMateRobot& robot) {
  error_code ec;
  std::vector<int> int_data_len1 = { 0 };
  std::vector<int> int_data_len3 = { 0,0,1 };
  std::vector<int> int_data_len4 = { 0,0,0,1 };
  int ret = 0;

  //0x03 and 0x04 are similar
  robot.XPRWModbusRTUReg(0x01, 0x03, 0x0001, "int16", 1, int_data_len1, false, ec);
  std::cout << "regtest 0x03 code:" << ec << std::endl;

  robot.XPRWModbusRTUReg(0x01, 0x04, 0x0001, "int16", 1, int_data_len1, false, ec);
  std::cout << "regtest 0x04 code:" << ec << std::endl;

  //0x06 writes a single register; parameter 3's length can only be 1
  robot.XPRWModbusRTUReg(0x01, 0x06, 0x0001, "int16", 1, int_data_len3, false, ec);
  std::cout << "regtest 0x06 code:" << ec << std::endl;


  //0x10 writes multiple registers
  robot.XPRWModbusRTUReg(0x01, 0x10, 0x0001, "int16", 1, int_data_len3, false, ec);
  std::cout << "regtest 0x10 code:" << ec << std::endl;

  robot.XPRWModbusRTUReg(0x01, 0x10, 0x0001, "int32", 1, int_data_len1, false, ec);
  std::cout << "regtest 0x10 code:" << ec << std::endl;

}

/**
 * @brief data_type register test interface, for the RM gripper
 */
void RegDataTypeTest(xMateRobot& robot) {
  error_code ec;
  std::vector<int> int16_data_len1 = { 0 };
  std::vector<int> int16_data_len2 = { 0 };
  std::vector<int> int32_data_len1_1 = { 65535 };
  std::vector<int> int32_data_len1_2 = { 255 };

  //Send: 01 03 02 00 00 01 85 B2
  robot.XPRWModbusRTUReg(1, 0x03, 0x0200, "int16", 1, int16_data_len1, false, ec);
  std::cout << "regdate type test int16:" << ec << std::endl;

  //Send: 01 03 02 00 00 01 85 B2
  robot.XPRWModbusRTUReg(1, 0x03, 0x0200, "uint16", 1, int16_data_len1, false, ec);
  std::cout << "regdate type test uint16:" << ec << std::endl;

  //Send: 01 10 02 00 00 02 04 FF FF 00 00 EA EB
  robot.XPRWModbusRTUReg(1, 0x10, 0x0200, "int32", 1, int32_data_len1_1, false, ec);
  std::cout << "regdate type test int32:" << ec << std::endl;

  //Send: 01 10 02 00 00 02 04 00 FF 00 00 DA FF
  robot.XPRWModbusRTUReg(1, 0x10, 0x0200, "int32", 1, int32_data_len1_2, false, ec);
  std::cout << "regdate type test int32:" << ec << std::endl;

  //Send: 01 10 02 00 00 02 04 FF FF 00 00 EA EB
  robot.XPRWModbusRTUReg(1, 0x10, 0x0200, "uint32", 1, int32_data_len1_1, false, ec);
  std::cout << "regdate type test uint32:" << ec << std::endl;

  //Send: 01 10 02 00 00 02 04 00 FF 00 00 DA FF
  robot.XPRWModbusRTUReg(1, 0x10, 0x0200, "uint32", 1, int32_data_len1_2, false, ec);
  std::cout << "regdate type test uint32:" << ec << std::endl;
}

/**
 * @brief Raw-transmission data interface
 */
void RWData_Test(xMateRobot& robot) {
  error_code ec;
  //Raw-transmission initialization: directly initialize the Dahuan gripper using raw-transmission data
  std::vector<uint8_t> send_data = { 0x01,0x06,0x01,0x00,0x00,0xA5,0x48,0x4D };//initialization raw-transmission data
  std::vector<uint8_t> rev_data = {};//received raw-transmission data
  robot.XPRS485SendData((int)send_data.size(), (int)rev_data.size(), send_data, DhGripParams::rev_data,ec);
  //std::vector<int>  init_set = { 0xA5 };//initialization data, non-raw-transmission
  //robot.XPRWModbusRTUReg(1, 0x06, 0x0100, "int16", 1, init_set, false, ec);
  std::cout << "DH gripper initialization result via raw transmission: " << ec << std::endl;
}

/**
 * @brief main program
 */
int main() {

  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateRobot robot(ip);

    std::cout << "Establishing connection with the robot" << std::endl;

    //coil interface test
    //CoilTest(robot);
    //register function code test
    //RegFuncCodeTest(robot);

    //register data type test
    //RegDataTypeTest(robot);


    ////------------------------------------------DH gripper demo-----------------------------------------------
    //
    //1. Turn on the end-effector xpanel's 24V power supply and RS485
    std::cout << "Configuring xpanel..." << std::endl;
    robot.setxPanelRS485(xPanelOpt::Vout::supply24v, true, ec);
    std::cout << "xpanel configuration result: " << ec << std::endl;
    //2. ---------------------------------Run the sample program--------------------------------------------
    //(1) Initialize
    DHGripInit(robot);

    int i = 14;
    while (i--) {
      //(2) Get initialization status
      int init_status = -1;
      DHGripGetInitStatus(robot, init_status);
      std::cout << "Get initialization status: "<< init_status << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));//5s used to check initialization; the DH gripper takes about 7s to initialize
    }
    //(3) Set force, speed, position
    int trq_set = 100;
    int vel_set = 2;
    int pos_set = 0;
    //(4) Move
    DHGripMove(robot, trq_set, vel_set, pos_set);
    //(5) Get force, speed, position
    int trq_get = 0;
    int vel_get = 0;
    int pos_get = 0;
    DHGripGetInfo(robot, trq_get, vel_get, pos_get);
    std::cout << "Retrieved force: " << trq_get << std::endl;
    std::cout << "Retrieved speed: " << vel_get << std::endl;
    std::cout << "Retrieved position: " << pos_get << std::endl;
    while (true) {
      //(6) Get grip status
      int grip_status_get = -1;
      DHGripGetStatus(robot, grip_status_get);
      std::cout << "Get grip status: "<< grip_status_get << std::endl;
      //(7) Get real-time position
      int pos_now_get = -1;
      DHGripGetNewPos(robot, pos_now_get);
      std::cout << "Get real-time position: " <<pos_now_get << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(500)); //retrieve the gripper's current position status every 0.5s
    }
  }
  catch (const std::exception& e) {
    std::cout << e.what();
  }
  return 0;
}