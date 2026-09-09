/**
 * @file sdk_example.cpp
 * @brief Usage examples for the various SDK interfaces
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#include <iostream>
#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"
#include "rokae/utility.h"
#include "rokae/model.h"

using namespace rokae;
std::ostream &os = std::cout; ///< print to console

namespace Workflow {

 /**
  * @brief Example - tool/workpiece/base frame calibration
  */
 template<WorkType Wt, unsigned short DoF>
 class CalibrateFrame {
  public:
   /**
    * @brief Constructor
    * @param robot An already-created robot instance
    * @param type The frame type to calibrate
    * @param point_num The number of positions to be provided, corresponding to the N-point method
    * @param is_held Whether the robot is hand-held
    * @param base_aux Auxiliary points for base frame calibration
    */
   CalibrateFrame(Robot_T<Wt,DoF> &robot, FrameType type, int point_num, bool is_held, const std::array<double, 3> &base_aux = {})
     : robot_(&robot), type_(type), point_list_(point_num), is_held_(is_held), base_aux_(base_aux) {}

   /**
    * @brief Set a calibration point
    */
   void setPoint(unsigned point_index) {
     if(point_index >= point_list_.size()) {
       // Add your own exception handling here
       print(std::cerr, "Calibration point index out of range");
       return;
     }
     error_code ec;
     // please guarantee robot is valid before calling
     point_list_[point_index] = robot_->jointPos(ec);
   }

   /**
    * @brief All calibration positions have been confirmed; obtain the calibration result
    * @param ec Calibration result error code
    * @return Calibration result
    */
   FrameCalibrationResult confirm(error_code &ec) {
     return robot_->calibrateFrame(type_, point_list_, is_held_, ec, base_aux_);
   }

  private:
   Robot_T<Wt, DoF> *robot_;
   FrameType type_;
   std::vector<std::array<double, DoF>> point_list_;
   bool is_held_;
   std::array<double, 3> base_aux_;
 };

 /**
  * @brief Example - Jog the arm back within its soft limits after it exceeds them
  */
 template<WorkType Wt, unsigned short DoF>
 void recoveryFromOverJointRange(Robot_T<Wt, DoF> *robot) {
   error_code ec;
   auto curr_joint = robot->jointPos(ec);
   std::array<double[2], DoF> soft_limits {};
   // Read the current soft limit settings
   robot->getSoftLimit(soft_limits, ec);

   std::array<double, DoF> jog_steps {};
   bool outofRange = false;
   // Jog any axis that exceeds its limit back within the soft limit by ±0.08 rad (about 5 degrees)
   double margin = 0.08;
   for(unsigned i = 0; i < DoF; ++i) {
     if(curr_joint[i] > soft_limits[i][1]) {
       jog_steps[i] = soft_limits[i][1] - curr_joint[i] - margin;
       outofRange = true;
     }
     if(curr_joint[i] < soft_limits[i][0]) {
       jog_steps[i] = soft_limits[i][0] - curr_joint[i] + margin;
       outofRange = true;
     }
   }
   if(!outofRange) {
     print(std::cout, "Current joint angles are within the soft limits; no recovery needed");
     return;
   }

   // After powering off, disable the soft limits
   robot->setPowerState(false, ec);
   robot->setOperateMode(OperateMode::manual, ec);
   robot->setSoftLimit(false, ec);
   robot->setPowerState(true, ec);

   // Jog each axis in turn
   double rate = 0.2; // Jog rate
   for(unsigned i = 0; i < DoF; ++i) {
     if(jog_steps[i] != 0) {
       robot->startJog(JogOpt::Space::jointSpace, rate, Utils::radToDeg(abs(jog_steps[i])), i,
                       jog_steps[i] > 0, ec);
       bool running = true;
       while (running) {
         std::this_thread::sleep_for(std::chrono::milliseconds(100));
         auto st = robot->operationState(ec);
         print(std::cout, st);
         if(st == OperationState::jog){
           running = false;
         }
       }
     }
   }
   robot->stop(ec);
   robot->setPowerState(false, ec);
   // Re-enable the soft limits
   robot->setSoftLimit(true, ec);
 }
}

/**
 * @brief Example - calibrate the tool/workpiece frame
 */
template<WorkType Wt, unsigned short DoF>
void example_calibrateFrame(Robot_T<Wt, DoF> *robot) {
  int point_count = 4;
  Workflow::CalibrateFrame calibrate_frame(*robot, FrameType::tool, point_count, true);
  for(int i = 0; i < point_count; i++) {
    print(std::cout, "Jog the robot to the calibration point, then press Enter to confirm");
    while(getchar() != '\n');
    calibrate_frame.setPoint(i);
  }
  error_code ec;
  FrameCalibrationResult calibrate_result = calibrate_frame.confirm(ec);
  if(ec) {
    print(std::cerr, "Calibration failed:", ec);
  } else {
    print(std::cout, "Calibration succeeded, result -", calibrate_result.frame, "\nDeviation:", calibrate_result.errors);
  }
}

/**
 * @brief Example - compute forward/inverse kinematics
 */
template <WorkType wt, unsigned short dof>
void example_coordinateCalculation(Robot_T<wt, dof> *robot){
  error_code ec;
  auto tcp_xyzabc = robot->posture(CoordinateType::endInRef, ec);
  // *** Compute inverse & forward kinematics ***
  //set the tool frame
  Toolset toolset1;
  toolset1 = robot->toolset(ec);
  print(os, "Tool/workpiece frame read from the controller:", toolset1);
  auto model = robot->model();

  // Compute the inverse kinematics under the currently configured tool/workpiece frame
  model.calcIk(tcp_xyzabc, ec);
  // Compute the inverse kinematics under toolset1
  auto ik = model.calcIk(tcp_xyzabc, toolset1, ec);
  // Compute the forward kinematics under the currently configured tool/workpiece frame
  model.calcFk(ik, ec);
  // Compute the forward kinematics under toolset1
  auto fk_ret = model.calcFk(ik, toolset1, ec);
  print(os, "Current inverse kinematics solution:", ik);
  print(os, "Forward kinematics solution:", fk_ret);

  //*** Frame conversion: end-effector relative to external reference & flange relative to base ***
  //query the base frame configuration
  auto base_in_world = robot->baseFrame(ec);
  auto flan_in_base =Utils::EndInRefToFlanInBase(base_in_world, toolset1, tcp_xyzabc);
  auto flan_pos = robot->posture(CoordinateType::flangeInBase, ec);
  auto end_in_ref = Utils::FlanInBaseToEndInRef(base_in_world, toolset1, flan_pos);

  print(os, "Input end-effector pose relative to the external reference frame", tcp_xyzabc);
  print(os, "Computed end-effector pose relative to the external reference frame", end_in_ref);
  print(os, "Input flange pose relative to the base frame", flan_pos);
  print(os, "Computed flange pose relative to the base frame", flan_in_base);

  // Example of computing all inverse kinematics solutions
  // Offset the current flange-relative-to-base pose and compute all inverse kinematics solutions for the offset pose
  auto cart_pos = robot->cartPosture(CoordinateType::flangeInBase, ec);
  cart_pos.trans[1] += 0.05;
  cart_pos.rpy[0] += Utils::degToRad(20);
  std::vector<std::vector<int>> calc_confs;
  auto ik_solutions = model.calcAllIkSolutions(cart_pos, calc_confs, ec);
  if(ec) {
    print(os, "Failed to compute inverse kinematics:", ec);
  } else {
    print(os, "Number of inverse kinematics solutions found", ik_solutions.size());
    for(size_t i = 0; i < ik_solutions.size(); ++i) {
      print(os, " ->", ik_solutions[i], "| conf:", calc_confs[i]);
    }
  }
}

/**
 * @brief Example - basic information queries
 */
template <WorkType wt, unsigned short dof>
void example_basicOperation(Robot_T<wt, dof> *robot){
  error_code ec;
  // *** Query information ***
  auto robotinfo = robot->robotInfo(ec);
  print(os, "Controller version:", robotinfo.version, "Model:", robotinfo.type);
  print(os, "xCore-SDK version:", robot->sdkVersion());

  // *** Get the robot's current pose, joint angles, base frame, and other information ***
  auto joint_pos = robot->jointPos(ec); // joint angles [rad]
  auto joint_vel = robot->jointVel(ec); // joint velocities [rad/s]
  auto joint_torque = robot->jointTorque(ec); // joint torques [Nm]
  auto tcp_xyzabc = robot->posture(CoordinateType::endInRef, ec);
  auto flan_cart = robot->cartPosture(CoordinateType::flangeInBase, ec);
  robot->baseFrame(ec); // base frame
  print(os, "End-effector pose relative to the external reference frame", tcp_xyzabc);
  print(os, "Flange relative to the base frame -", flan_cart);

#if 0
  // Set the base frame. Requires restarting the control cabinet PC to take effect
  Frame base_frame_headstand = {0, 0, 0, M_PI, 0, 0}; // upside-down mount, A = 180 deg
  robot->setBaseFrame(base_frame_headstand, ec);
#endif

  // Query the 5 most recent error-level controller log entries
  print(os, "Querying the 5 most recent error-level controller log entries");
  auto controller_logs = robot->queryControllerLog(5, {LogInfo::error}, ec);
  for(const auto &log: controller_logs) {
    print(os, log.content);
  }

  // Query controller log entries 10-15 of all levels
  print(os, "Querying controller log entries 10-15 of all levels");
  controller_logs = robot->queryControllerLog(5, {LogInfo::error}, ec, 10);
  for(const auto &log: controller_logs) {
    print(os, log.content);
  }
}

/**
 * @brief Example - enable/disable drag (hand-guiding)
 */
void example_drag(BaseCobot *robot) {
  error_code ec;
  robot->setOperateMode(rokae::OperateMode::manual, ec);
  robot->setPowerState(false, ec); // Before enabling drag, the arm must be powered off in manual mode
  // Cartesian space, free drag
  robot->enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec);
  print(os, "Enable drag", ec, "press Enter to continue");
  std::this_thread::sleep_for(std::chrono::seconds(2)); //wait for the control mode switch
  while(getchar() != '\n');
  robot->disableDrag(ec);
  std::this_thread::sleep_for(std::chrono::seconds(2)); //wait for the control mode switch
}

/**
 * @brief Example - jog the robot
 * @param robot
 */
void example_jog(BaseRobot *robot) {
  error_code ec;
  robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
  robot->setOperateMode(rokae::OperateMode::manual, ec); // jog requires manual mode
  print(os, "Preparing to jog the robot. Manual-mode power-on is required; please confirm power is on, then press Enter");
  // If an external enable switch is connected, hold the switch to power on manually
  robot->setPowerState(true, ec);

  print(os, "-- Starting to jog the robot -- \nIn the world frame, move 50mm along Z+, rate 50%. Wait for the robot to stop moving, then press Enter to continue");
  robot->startJog(JogOpt::world, 0.5, 50, 2, true, ec);
  while(getchar() != '\n');
  print(os, "Joint space, continuous negative rotation of axis 6, rate 5%, press Enter to stop jogging");
  robot->startJog(JogOpt::jointSpace, 0.05, 5000, 5, false, ec);
  while(getchar() != '\n'); // press Enter to stop
  robot->stop(ec); // stop() must be called to end jogging
}

/**
 * @brief Example - singularity-avoidance jog, applicable to the xMateSR and xMateCR series
 */
void example_avoidSingularityJog(xMateRobot &robot) {
  error_code ec;
  robot.setOperateMode(rokae::OperateMode::manual, ec); // jog requires manual mode
  print(os, "Preparing to jog the robot. Manual-mode power-on is required; please confirm power is on, then press Enter");
  // If an external enable switch is connected, hold the switch to power on manually
  robot.setPowerState(true, ec);
  while(getchar() != '\n');

  print(os, "-- Starting to jog the robot -- \nSingularity-avoidance mode, move 50mm along Y+, rate 20%. Wait for the robot to stop moving, then press Enter to continue");
  robot.startJog(JogOpt::singularityAvoidMode, 0.2, 50, 1, true, ec);
  while(getchar() != '\n'); // press Enter to stop
  robot.stop(ec); // stop() must be called to end jogging
}

/**
 * @brief Example - NTP configuration. Note: NTP is not a standard feature and requires an additional robot upgrade
 */
void example_ConfigNtp(BaseRobot *robot) {
  error_code ec;
  // Set the NTP server address
  robot->configNtp("192.168.0.170", ec);
  if(ec) {
    print(os, "Failed to set the NTP server address:", ec);
  }
  // Sync the time once
  robot->syncTimeWithServer(ec);
  if(ec) {
    print(os, "Time sync failed:", ec);
  }
}

/**
 * @brief Example - enable and disable collision detection
 */
template <unsigned short dof>
void example_setCollisionDetection(Cobot<dof> *robot) {
  error_code ec;
  // Set the sensitivity for each axis, range 0.01 ~ 2.0, equivalent to 1% ~ 200% as set in RobotAssist
  // Trigger behavior: safety stop; retreat distance 0.01m
  robot->enableCollisionDetection({1.0, 1.0, 0.01, 2.0, 1.0, 1.0, 1.0}, StopLevel::stop1, 0.01, ec);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  // Disable collision detection
  robot->disableCollisionDetection(ec);
}

/**
 * @brief Emergency stop reset
 */
void example_emergencyStopReset(BaseRobot *robot) {
  error_code ec;
  print(os, "Emergency stop reset");
  robot->recoverState(1, ec);
  if (ec) {
    print(os, "Reset failed:", ec);
  } else {
    print(os, "Reset succeeded");
  }
}

/**
 * @brief Enable/disable parallel-base mode (5-axis collaborative robot)
 */
void example_CompletePostureLerp(xMateCr5Robot* robot) {
    error_code ec;
    print(os, "Enable parallel-base mode");
    robot->enableCompletePostureLerp(true, ec); // true: enable, false: failure
    if (ec) {
        print(os, "Failed to enable parallel-base mode:", ec);
    }
    else {
        print(os, "Parallel-base mode enabled successfully");
    }
}

/**
 * @brief Example - set the teach pendant mode
 */
void example_setTpMode(BaseRobot *robot) {
  error_code ec;
  // Use without a teach pendant
  robot->setTeachPendantMode(false, ec);
  if(ec) {
    print(os, "Configuration failed:", ec);
    return;
  }
  print(os, "Successfully configured to operate without a teach pendant");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  // When not connected to a teach pendant, the robot can be powered on in manual mode
  robot->setOperateMode(rokae::OperateMode::manual, ec);
  robot->setPowerState(true, ec); // could be powered on without teach pendant
}

/**
 * @brief Reboot/shut down the control cabinet PC
 */
void example_reboot(BaseRobot* robot) {
  error_code ec;
  robot->rebootSystem(ec);
  print(os, "Rebooting the control cabinet PC");
  if (ec) print(os, "Reboot failed:", ec);
  else print(os, "Reboot succeeded");

  // Shut down the control cabinet PC
  robot->shutdownSystem(ec);
  if (ec) print(os, "Shutdown failed:", ec);
  else print(os, "Shutdown succeeded");
}

/**
 * @brief Example - set the connection/disconnection callback function
 */
void example_setConnectionHandler(BaseRobot *robot) {
  auto handler = [](bool connected){
    print(os, "Detect", connected ? "connection" : "disconnection");
  };
  robot->setConnectionHandler(handler);
}

/**
 * @brief main program
 */
int main() {
  try {
    // *** 1. Connect to the robot ***
    std::string ip = "192.168.2.160";
    std::cout << "Connecting to the robot with " << ip << "..." << std::endl;
    xMateRobot robot(ip);  // this connects to a 6-axis collaborative robot model
    std::cout << "Built robot object!" << std::endl;

    // Check whether an emergency stop (or safety gate) is currently active before doing anything else
    error_code power_ec;
    print(os, "Current power state:", robot.powerState(power_ec));

    example_emergencyStopReset(&robot); // reset emergency stop if needed

    print(os, "Current power state after reset:", robot.powerState(power_ec));

    // Other models
//    xMateErProRobot robot; // 7-axis collaborative robot model
//    StandardRobot robot; // connect to a 6-axis industrial robot model
//    PCB4Robot robot; // connect to a PCB 4-axis model
//    PCB3Robot robot; // connect to a PCB 3-axis model
//    xMateCr5Robot; // 5-axis collaborative robot model
std::cout << "Running basic operation example..." << std::endl;
example_basicOperation(&robot);

  } catch (const rokae::Exception &e) {
    std::cerr << e.what();
  }

  return 0;
}