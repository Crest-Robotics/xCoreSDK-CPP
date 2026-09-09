/**
 * @file get_keypad_state.cpp
 * @brief Read the end-effector keypad state
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
#include "print_helper.hpp"

using namespace rokae;

namespace {
 xMateRobot g_robot; ///< Robot object
}

/**
 * @brief Example - Read the end-effector keypad state
 */
void example_ReadKeyPadValue() {
  error_code ec;
  KeyPadState state = g_robot.getKeypadState(ec);
  std::cout << "Current end-effector keypad state, key1: " << state.key1_state << ",key2:"<<state.key2_state
            << ",key3:" << state.key3_state << ",key4:" << state.key4_state << ",key5:" << state.key5_state
            << ",key6:" << state.key6_state << ",key7:" << state.key7_state << std::endl;


  // Configure the data to be received; keypads is what this example program uses
  g_robot.startReceiveRobotState(std::chrono::milliseconds(1), { RtSupportedFields::keypads });

  std::array<bool, 7> keypad{};
  g_robot.getStateData(RtSupportedFields::keypads, keypad);

  // Run 50 times
  int count = 50;

  std::thread readKeyPad([&] {
    while (count--) {
      // Read the end-effector keypad state once every cycle
      g_robot.updateRobotState(std::chrono::milliseconds(1));
      g_robot.getStateData(RtSupportedFields::keypads, keypad);
      std::cout << "Current end-effector keypad state, key1: " << keypad[0] << ",key2:" << keypad[1]
                << ",key3:" << keypad[2] << ",key4:" << keypad[3] << ",key5:" << keypad[4]
                << ",key6:" << keypad[5] << ",key7:" << keypad[6] << std::endl;
    }
  });

  readKeyPad.join();
}

/**
 * @brief Example - Read/write IO and registers
 */
void example_io_register(BaseRobot *robot) {
  error_code ec;
  print(std::cout, "Current value of DO1_0:", robot->getDO(1,0,ec));
  robot->setSimulationMode(true, ec); // DI can only be set when input simulation mode is enabled
  robot->setDI(0, 2, true, ec);
  print(std::cout, "Current value of DI0_2:", robot->getDI(0, 2, ec));
  robot->setSimulationMode(false, ec); // Disable simulation mode

  // Read a single register, of type float
  // Assume "register0" is a register array of length 10
  float val_f;
  std::vector<float> val_af;
  // Read the 1st element, i.e. register0[1] in the state monitor; the result is assigned to val_f
  robot->readRegister("register0", 0, val_f, ec);
  // Read the 10th element, i.e. register0[10] in the state monitor; the result is assigned to val_f
  robot->readRegister("register0", 9, val_f, ec);
  // Read the entire array and assign it to val_af; val_af's length also becomes 10. In this case, the index parameter's value doesn't matter
  robot->readRegister("register0", 9, val_af, ec);

  // Read an int-type register array
  std::vector<int> val_ai;
  robot->readRegister("register1", 1, val_ai, ec);
  // Write a bool/bit-type register
  robot->writeRegister("register0", 0, true, ec);
  // Write a bool-type register array
  std::vector<bool> val_bool_array = { false,true,false,true,false,true,false };
  robot->writeRegister("register2", 0, val_bool_array, ec);
}

/**
 * @brief main program
 */
int main() {
  std::string remote_ip = "192.168.0.160";
  std::string local_ip = "192.168.0.100";
  try {
    // This example uses real-time state data, so the local machine address needs to be set
    g_robot.connectToRobot(remote_ip, local_ip);
  }
  catch (const std::exception& e) {
    std::cout << "Connection error: " << e.what();
    return -1;
  }


  return 0;
}