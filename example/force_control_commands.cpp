/**
 * @file force_control_commands.cpp
 * @brief Force control commands
 * @note Force control command parameter settings are usually related to the robot model.
 * Each example specifies the tested robot model; if your model differs, please refer to the "xCore Control System Manual" to confirm the parameters are appropriate before running the example.
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
void waitRobot(BaseRobot *robot);

/**
 * @brief Example - Cartesian space force control. Applicable model: xMateCR
 * Example - cartesian space force control
 */
void fcCartesianControl(xMateRobot &robot) {
  error_code ec;

  // Set the hand-held tool frame, Ry rotated 90°
  Toolset toolset1;
  toolset1.end.rpy[1] = M_PI_2;
  robot.setToolset(toolset1, ec);

  auto fc = robot.forceControl();

  // Force control initialization, using the tool frame
  // force control initialization, use tool frame
  fc.fcInit(FrameType::tool, ec);
  // Cartesian control mode
  // set force control type
  fc.setControlType(1, ec);
  // Set Cartesian stiffness. This example uses the tool frame, so the tool frame's x-direction has zero impedance, while the other directions have higher impedance.
  fc.setCartesianStiffness({0, 1000, 1000, 500, 500, 500}, ec);
  // Start force control
  print(std::cout , "Starting Cartesian mode force control");
  fc.fcStart(ec);

#if 0
  // Set the load; please configure according to actual conditions to ensure safety
  Load load;
  load.mass = 1;
  fc.setLoad(load, ec);
#endif

  // Set desired force
  // set desired force along Z axis
  fc.setCartesianDesiredForce({0, 0, 1, 0, 0, 0}, ec);

  // Press Enter to stop force control
  while(getchar() != '\n');
  fc.fcStop(ec);
}

/**
 * @brief Example - Joint space force control. Applicable model: xMateCR
 */
 template <unsigned short DoF>
void fcJointControl(ForceControl_T<DoF> &fc) {
  error_code ec;

  fc.fcInit(FrameType::base, ec);
  fc.setControlType(0, ec);
  // Set the stiffness for each joint. Joints 2 and 4 have low impedance, while the other joints have higher impedance.
  fc.setJointStiffness({1000, 10, 1000, 5, 50, 50}, ec);

  print(std::cout, "Starting joint mode force control");
  fc.fcStart(ec);
  // Set desired force
  fc.setJointDesiredTorque({1,1,3,0,0,0}, ec);

  // Press Enter to stop force control
  while(getchar() != '\n');
  fc.fcStop(ec);
}

/**
 * @brief Example - Search motion & force control monitoring. Tested model: xMateER3
 */
void fcOverlay(xMateRobot &robot){
  error_code ec;

  // Set the hand-held tool frame, Ry rotated 90°
  Toolset toolset1;
  toolset1.end.rpy[1] = M_PI_2;
  robot.setToolset(toolset1, ec);

  auto fc = robot.forceControl();

  // Optional: set force control monitoring parameters; the values used in this example are the default thresholds for the xMateER3 model
  // Set maximum joint velocity
  fc.setJointMaxVel({3.0, 3.0, 3.5, 3.5, 3.5, 4.0}, ec);
  // Set maximum joint momentum
  fc.setJointMaxMomentum({0.1, 0.1, 0.1, 0.055, 0.055, 0.055}, ec);
  // Set maximum joint kinetic energy
  fc.setJointMaxEnergy({250, 250, 250, 150, 150, 100}, ec);
  // Set maximum Cartesian space velocity
  fc.setCartesianMaxVel({1.0, 1.0, 1.0, 2.5, 2.5, 2.5}, ec);
  // Start monitoring
  fc.fcMonitor(true, ec);

  // Force control initialization
  fc.fcInit(FrameType::tool, ec);
  // Search motion must use Cartesian impedance control
  fc.setControlType(1, ec);

  // Set a sinusoidal search motion around the Z axis (since the force control frame was set to the tool frame above, this is the tool Z axis)
  fc.setSineOverlay(2, 6, 1, M_PI, 1, ec);
  // Start force control
  fc.fcStart(ec);
  // Overlay a Lissajous search motion in the XZ plane
  fc.setLissajousOverlay(1, 5, 1, 10, 5, 0, ec);
  // Start search motion
  print(std::cout, "Starting search motion");
  fc.startOverlay(ec);

#if 0
  // Pause and restart search motion
  fc.pauseOverlay(ec);
  fc.restartOverlay(ec);
#endif

  // Press Enter to stop force control
  while(getchar() != '\n');
  fc.stopOverlay(ec);

  // Restore monitoring parameters to default values
  fc.fcMonitor(false, ec);
  // Stop force control
  fc.fcStop(ec);
}

/**
 * @brief Example - Set force control termination conditions. Tested model: xMateER3
 */
void fcCondition(xMateRobot &robot) {
  auto fc = robot.forceControl();
  error_code ec;
  Toolset toolset;
  toolset.ref.trans[2] = 0.1;
  robot.setToolset(toolset, ec);

  fc.fcInit(FrameType::world, ec);
  fc.setControlType(1, ec);
  fc.fcStart(ec);
  // Set force limits
  fc.setForceCondition({-20, 20, -15, 15, -15, 15}, true, 20, ec);
  // Set a cuboid region limit; isInside=false means waiting terminates when inside this region
  // The frame the cuboid is defined in will be superimposed on the external workpiece frame
  Frame supvFrame;
  supvFrame.trans[2] = -0.1;
  fc.setPoseBoxCondition(supvFrame, {-0.6, 0.6, -0.6, 0.6, 0.2, 0.3}, false, 20, ec);

  // Block and wait until the termination condition is met
  print(std::cout, "Starting to wait");
  fc.waitCondition(ec);

  print(std::cout, "Wait finished, stopping force control");
  fc.fcStop(ec);
}

/**
 * @brief Read end-effector torque information
 */
template <unsigned short DoF>
void readTorqueInfo(ForceControl_T<DoF> &fc) {
  error_code ec;
  std::array<double, DoF> joint_torque{}, external_torque{};
  std::array<double, 3> cart_force{}, cart_torque{};

  // Read current torque information
  fc.getEndTorque(FrameType::flange, joint_torque, external_torque, cart_torque, cart_force, ec);
  print(std::cout, "End-effector torque");
  print(std::cout, "Measured force per joint -", joint_torque);
  print(std::cout, "External force per joint -", external_torque);
  print(std::cout, "Cartesian torque -", cart_torque);
  print(std::cout, "Cartesian force  -", cart_force);
}

/**
 * @brief Example - Force/torque sensor calibration
 */
void example_CalibrateForceSensor(xMateRobot &robot) {
  error_code ec;
  // Calibrate all joints
  robot.calibrateForceSensor(true, 0, ec);
  // Single-axis (joint 4) calibration
  robot.calibrateForceSensor(false, 3, ec);
}

/**
 * @brief main program
 */
int main() {
  using namespace rokae;
  xMateRobot robot;
  try {
    robot.connectToRobot("192.168.0.160");
  } catch(const Exception &e) {
    std::cerr << e.what();
    return 1;
  }
  error_code ec;

  // Force control instance
  auto fc = robot.forceControl();
  readTorqueInfo(fc);

  // Power on
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setPowerState(true, ec);

  // First move to the drag-teaching pose; make sure to select the correct robot model
  std::vector<double> drag_cr = {0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0},
  drag_er = {0, M_PI/6, M_PI/3, 0, M_PI_2, 0};
  robot.executeCommand({MoveAbsJCommand(drag_cr)}, ec);
  waitRobot(&robot);

  // Run example program
  fcCartesianControl(robot);
  fcJointControl(fc);
//  fcCondition(robot);

  robot.setPowerState(false, ec);
  robot.setOperateMode(OperateMode::manual, ec);

  return 0;
}

/**
 * @brief Wait for the robot to come to a stop
 * @param robot robot pointer
 */
void waitRobot(BaseRobot *robot) {
  bool running = true;
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    error_code ec;
    auto st = robot->operationState(ec);
    if(st == OperationState::idle || st == OperationState::unknown){
      running = false;
    }
  }}