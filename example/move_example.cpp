/**
 * @file move_example.cpp
 * @brief Non-real-time motion commands. Depending on the robot model and coordinate system, the waypoints in
 * each example may not be reachable; they are provided only as a reference for API usage.
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#include <iostream>
#include <array>
#include <thread>
#include <cmath>
#include <chrono>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "print_helper.hpp"

using namespace std;
using namespace rokae;
std::ostream &os = std::cout; ///< print to console

namespace Predefines {
 // ******   Drag teaching posture   ******
 const std::vector<double> ErDragPosture = {0, M_PI/6, M_PI/3, 0, M_PI_2, 0}; ///< xMateEr3, xMateEr7
 const std::vector<double> ErProDragPosture = {0, M_PI/6, 0, M_PI/3, 0, M_PI_2, 0}; ///< xMateEr3 Pro, xMateEr7 Pro
 const std::vector<double> CrDragPosture {0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0}; ///< xMateCR
 const std::vector<double> Cr5DragPostre = { 0, M_PI / 6, -M_PI_2, -M_PI / 3, 0}; ///< CR5 axis configuration

 Toolset defaultToolset; ///< Default tool/workpiece
}
/**
 * @brief Print motion execution information
 */
void printInfo(const rokae::EventInfo &info) {
  using namespace rokae::EventInfoKey::MoveExecution;
  print(std::cout, "[Motion execution info] ID:", std::any_cast<std::string>(info.at(ID)), "Index:", std::any_cast<int>(info.at(WaypointIndex)),
        "Reached: ", std::any_cast<bool>(info.at(ReachTarget)) ? "YES": "NO", std::any_cast<error_code>(info.at(Error)),
          std::any_cast<std::string>(info.at(Remark)));
  // If custom info was set, print it
  if(info.count(CustomInfo)) {
    auto custom_info =  std::any_cast<std::string>(info.at(CustomInfo));
    if(!custom_info.empty()) print(std::cout, "Custom info: ",custom_info);
  }
}

/** Clear the Cartesian point's confData to avoid MoveJ reporting error -50021 */
static void clearCartConf(CartesianPosition &p) {
  p.confData.clear();
}

void waitForFinish(BaseRobot &robot, const std::string &traj_id, int index) {
  using namespace rokae::EventInfoKey::MoveExecution;
  error_code ec;
  while(true) {
    auto info = robot.queryEventInfo(Event::moveExecution, ec);
    auto _id = std::any_cast<std::string>(info.at(ID));
    auto _index = std::any_cast<int>(info.at(WaypointIndex));
    if(auto _ec = std::any_cast<error_code>(info.at(Error))) {
      print(std::cout, "Path", _id, ":", _index, "error:", _ec.message());
      return;
    }
    if(_id == traj_id && _index == index) {
      if(std::any_cast<bool>(info.at(ReachTarget))) {
        print(std::cout, "Path", traj_id, ":", index, "completed");
      }
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

/**
 * @brief Wait for motion to finish - by polling whether the robot is in motion
 */
void waitRobot(BaseRobot &robot, bool &running) {
  running = true;
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    error_code ec;
    auto st = robot.operationState(ec);
    if(st == OperationState::idle || st == OperationState::unknown){
      running = false;
    }
  }
}

/**
 * @brief Event handling - after a simulated collision, wait 20 seconds, power on, and resume
 * After a collision occurs, the robot controller immediately begins logging diagnostic data; this
 * takes about 10 seconds, and motion can only resume once logging is complete.
 */
void recoverFromCollision(BaseRobot &robot, const rokae::EventInfo &info) {
  using namespace rokae::EventInfoKey;
  bool isCollided = std::any_cast<bool>(info.at(Safety::Collided));
  print(std::cout, "Collided:", isCollided);
  if(isCollided) {
    std::this_thread::sleep_for(std::chrono::seconds(20));
    error_code ec;
    robot.setPowerState(true, ec);
    robot.moveStart(ec);
    print(std::cout, "Recovered from collision");
  }
}

/**
 * @brief Linear motion that strictly follows joint configuration data (conf data). Waypoints apply to the xMateER3 model.
 */
void moveWithForcedConf(xMateRobot &robot) {
  Toolset default_toolset;
  error_code ec;
  bool running;
  std::string id;
  robot.setToolset(default_toolset, ec);
  robot.setDefaultSpeed(200,ec);
  robot.setDefaultZone(5, ec);

  print(std::cout, "Moving to drag teaching posture");
  robot.executeCommand({MoveAbsJCommand(Predefines::ErDragPosture)}, ec);
  waitRobot(robot, running);

  CartesianPosition cartesian_position0({0.786, 0, 0.431, M_PI, 0.6, M_PI});
  CartesianPosition cartesian_position1({0.786, 0, 0.431, M_PI, 0.98, M_PI});


  MoveJCommand j0({cartesian_position0}), j1({cartesian_position1});
  MoveLCommand l0({cartesian_position0}), l1({cartesian_position1});

  // Not strictly following joint configuration data: MoveL & MoveJ pick the IK solution closest to the current joint angles
  print(std::cout, "Starting MoveJ");
  robot.moveAppend({j0, j1}, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 1);

  // Following joint configuration data: use conf to compute the IK solution; MoveL may fail to find an IK solution here
  robot.setDefaultConfOpt(true, ec);
  print(std::cerr, ec);
  cartesian_position1.confData = {-1,1,-1,0,1,0,0,2};
  l1.target = cartesian_position1;
  j1.target = cartesian_position1;

  print(std::cout, "Starting MoveJ");
  robot.moveAppend({j0, j1}, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 1);

  print(std::cout, "Moving to drag teaching posture");
  robot.executeCommand({MoveAbsJCommand(Predefines::ErDragPosture)}, ec);
  waitRobot(robot, running);

  print(std::cout, "Starting MoveL");
  robot.moveAppend({l0, l1}, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 1);
  robot.setDefaultConfOpt(false, ec);
}

/**
 * @brief Example - setting an offset on a Cartesian waypoint & pausing/resuming during motion; waypoints apply to the xMateEr7 Pro model
 */
void cartesianPointWithOffset(BaseRobot &robot) {
  error_code ec;

  std::array<double, 6> pos = {0.631, 0, 0.38, M_PI, 0, M_PI};
  std::array<double,6> offset_z = {0, 0, 0.2, 0, 0, 0};

  MoveLCommand moveL1(pos, 500, 5), moveL2(pos, 800, 0);
  // Example: set the rotational speed to 100 deg/s; the default is 200 deg/s if not set
  moveL1.rotSpeed = 100 / 180.0 * M_PI;

  // Offset +0.2m along Z relative to the workpiece frame
  moveL2.offset = { CartesianPosition::Offset::offs, offset_z};

  MoveJCommand moveJ1(pos, 200, 0), moveJ2(pos, 1000, 80);
  // Offset +0.2m along Z relative to the tool frame
  moveJ2.offset = {CartesianPosition::Offset::relTool, offset_z};

  // First move to the starting position, then execute these 4 waypoints
  robot.executeCommand({MoveAbsJCommand(Predefines::ErProDragPosture)}, ec);
  robot.executeCommand({moveL1, moveL2}, ec);
  robot.executeCommand({moveJ1, moveJ2}, ec);

  std::thread input([&]{
    int c{};
    print(os, "[p] pause [c] resume [q] quit");
    while(c != 'q') {
      c = getchar();
      switch(c) {
        case 'p':
          robot.stop(ec);
          print(std::cerr, ec); break;
        case 'c':
          robot.moveStart(ec);
          print(std::cerr, ec); break;
        default: break;
      }
    }
  });
  input.join();
  robot.moveReset(ec);
}

/**
 * @brief Spiral motion, applies to model: xMate3
 */
void spiralMove(rokae::xMateRobot &robot) {
  error_code ec;
  std::string id;
  rokae::Toolset default_toolset = {};
  robot.setToolset(default_toolset, ec);

  // Spiral end posture; only rpy is used, xyz values are arbitrary
  rokae::CartesianPosition cart_target({0, 0, 0, 2.967, -0.2, 3.1415}),
  cart_target1({0, 0, 0, -2.787577,0.1639,-2.9});
  rokae::MoveAbsJCommand absjcmd({0.0,0.22150561307150393,1.4779577696969546,0.0,1.2675963456219013,0.0});

  // Spiral 1: initial radius 0.01m, radius change step 0.0005m/rad, counterclockwise 720 deg, speed v500
  rokae::MoveSPCommand spcmd1({cart_target, 0.01, 0.0005, M_PI * 4, false, 500}),
  // Spiral 2: initial radius 0.05m, radius change step 0.001m/rad, clockwise 360 deg, speed v100
  spcmd2({cart_target1, 0.05, 0.001, M_PI * 2, true, 100});

  std::vector<rokae::MoveSPCommand> spcmds = {spcmd1, spcmd2};
  robot.moveAppend({absjcmd}, id, ec);
  robot.moveAppend(spcmds, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, (int)spcmds.size() - 1);
}

/**
 * @brief Example - seven-axis redundant motion & resuming motion after collision detection, waypoints apply to the xMateER3 Pro model
 */
void redundantMove(xMateErProRobot &robot) {
  error_code ec;
  std::string id;

  // This example uses the default tool/workpiece, speed v500, turning zone fine
  Toolset defaultToolset;
  robot.setToolset(defaultToolset, ec);
  robot.setDefaultSpeed(500, ec);
  robot.setDefaultZone(0, ec);

  // Optional: set a callback for collision detection events
  robot.setEventWatcher(Event::safety, [&](const EventInfo &info){
    recoverFromCollision(robot, info);
  }, ec);


  MoveAbsJCommand moveAbsj({0, M_PI/6, 0, M_PI/3, 0, M_PI_2, 0});
  // ** 1) Elbow-angle-changing motion **
  MoveLCommand moveL1({0.562, 0, 0.432, M_PI, 0, -M_PI});
  moveL1.target.elbow = 1.45;
  robot.moveAppend({moveAbsj}, id, ec);
  robot.moveAppend({moveL1}, id, ec);
  moveL1.target.elbow = -1.51;
  robot.moveAppend({moveL1}, id, ec);
  robot.moveStart(ec);
  // The last moveAppend() sends one command, so index = 0
  waitForFinish(robot, id, 0);

  // ** 2) 60-degree elbow-angle arc **
  CartesianPosition circle_p1({0.472, 0, 0.342, M_PI, 0, -M_PI}),
  circle_p2({0.602, 0, 0.342, M_PI, 0, -M_PI}),
  circle_a1({0.537, 0.065, 0.342, M_PI, 0, -M_PI}),
  circle_a2({0.537, -0.065, 0.342, M_PI, 0, -M_PI});
  // All elbow angles are 60 degrees
  circle_p1.elbow = M_PI/3;
  circle_p2.elbow = M_PI/3;
  circle_a1.elbow = M_PI/3;
  circle_a2.elbow = M_PI/3;

  MoveLCommand moveL2(circle_p1);
  robot.moveAppend({moveL2}, id, ec);
  MoveCCommand moveC1(circle_p2, circle_a1), moveC2(circle_p1, circle_a2);
  std::vector<MoveCCommand> movec_cmds = {moveC1, moveC2};
  robot.moveAppend(movec_cmds, id, ec);
  robot.moveStart(ec);
  // The last moveAppend() sends 2 commands, so we must wait for the second point to finish before returning; index is the second point's index
  waitForFinish(robot, id, (int)movec_cmds.size() - 1);
}

/**
 * @brief Example - full-circle motion, waypoints apply to the XMC20 model
 */
void fullCircleMove(xMateRobot &robot) {
  error_code ec;

  // This example uses the default tool/workpiece
  Toolset defaultToolset;
  robot.setToolset(defaultToolset, ec);

  // Starting angle
  std::array<double, 6> start_angle = {0, 0.557737,-1.5184888, 0,-1.3036738, 0};

  auto robot_model = robot.model();
  // Posture corresponding to the starting angle
  auto cart_pose = robot_model.calcFk(start_angle, ec);

  MoveAbsJCommand abs_j({start_angle[0], start_angle[1], start_angle[2],
                         start_angle[3], start_angle[4], start_angle[5]}, 1000, 0);

  // Full-circle command, executes 360 degrees
  MoveCFCommand move_cf1(cart_pose, cart_pose, M_PI * 2, 100, 10);

  // Aux point 1: starting posture offset by Y+10mm
  move_cf1.auxOffset.type = CartesianPosition::Offset::offs;
  move_cf1.auxOffset.frame.trans[1] = 0.01;
  // Aux point 2: starting posture offset by X+5mm, Y-10mm
  move_cf1.targetOffset.type = CartesianPosition::Offset::offs;
  move_cf1.targetOffset.frame.trans[0] = 0.005;
  move_cf1.targetOffset.frame.trans[1] = -0.01;

  MoveCFCommand move_cf2 = move_cf1, move_cf3 = move_cf1;

  // Set the three rotation posture types respectively
  move_cf1.rotType = MoveCFCommand::constPose;
  move_cf2.rotType = MoveCFCommand::rotAxis;
  move_cf3.rotType = MoveCFCommand::fixedAxis;

  std::string id;
  // Execute the three full-circle motions
  // Note: move to the starting angle before each execution, otherwise a joint-limit-exceeded error may occur
  robot.moveAppend({abs_j}, id, ec);
  robot.moveAppend({ move_cf1 }, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 0);

  robot.moveAppend({abs_j}, id, ec);
  robot.moveAppend({ move_cf2 }, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 0);

  robot.moveAppend({abs_j}, id, ec);
  robot.moveAppend({ move_cf3 }, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 0);
}

/**
 * @brief Lock-axis-4 singularity avoidance method. Example applies to the xMateCR7 model.
 */
void avoidSingularityMove_Lock4(rokae::xMateRobot &robot) {
  error_code ec;
  std::string id;
  bool running;
  robot.setToolset(Predefines::defaultToolset, ec);

  // First move to the starting posture
  robot.executeCommand({MoveAbsJCommand({0.453,0.539,-1.581,0.0,0.026,0})}, ec);
  waitRobot(robot, running);

  std::vector<rokae::MoveLCommand> cmds = {
    MoveLCommand({0.66675437164302165, -0.23850040314585069, 0.85182031,-3.1415926535897931, 1.0471975511965979, 3.01151}),
    MoveLCommand({0.66675437164302154, 0.15775146321850292, 0.464946,-3.1415926535897931, 1.0471975511965979, -2.6885547129789127})
  };

  // Without singularity avoidance enabled, this reports an out-of-range error
  robot.setAvoidSingularity(AvoidSingularityMethod::lockAxis4, false, 0, ec);
  robot.moveAppend(cmds, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, (int)cmds.size() - 1);

  // Enable singularity avoidance, so the waypoints become reachable
  // Note: moveReset() disables all singularity avoidance features
  robot.moveReset(ec);
  robot.setAvoidSingularity(AvoidSingularityMethod::lockAxis4, true, 0, ec);
  std::cerr << ec;
  print(std::cout, "Lock-axis-4 singularity avoidance", robot.getAvoidSingularity(AvoidSingularityMethod::lockAxis4, ec) ? "enabled" : "disabled");

  robot.moveAppend(cmds, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, (int)cmds.size() - 1);
  robot.setAvoidSingularity(AvoidSingularityMethod::lockAxis4, false, 0, ec);
}

/**
 * @brief Example - using tool/workpiece frames, waypoints apply to the xMateCR7 model
 */
void moveInToolsetCoordinate(BaseRobot &robot) {
  error_code ec;
  std::string id;
  // Default tool/workpiece frame
  robot.setToolset(Predefines::defaultToolset, ec);

  MoveAbsJCommand moveAbs({0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0});
  robot.moveAppend({moveAbs}, id, ec);

  MoveLCommand movel1({0.563, 0, 0.432, M_PI, 0, M_PI}, 1000, 100);
  MoveLCommand movel2({0.33467, -0.095, 0.51, M_PI, 0, M_PI}, 1000, 100);
  robot.moveAppend({movel1, movel2}, id, ec);
  robot.moveStart(ec);
  bool moving = true;
  waitRobot(robot, moving);

  // Example: after movel1 and movel2 finish, switch to a different tool/workpiece, then run the subsequent motion commands
  // Setting the tool/workpiece, method 1: set it directly
  Toolset toolset1;
  toolset1.ref = {{0.1, 0.1, 0}, {0, 0, 0}}; // External reference frame, X+0.1m, Y+0.1m
  toolset1.end = {{ 0, 0, 0.01}, {0, M_PI/6, 0}}; // End-effector coordinates, Z+0.01m, Ry+30 deg
#if 0
  toolset1.load.mass = 2; // Payload 2kg
#endif
  robot.setToolset(toolset1, ec);

#if 0
  // Setting the tool/workpiece, method 2: use previously created tool1, wobj1
  robot.setToolset("tool1", "wobj1", ec);
#endif
  MoveLCommand movel3({0.5, 0, 0.4, M_PI, 0, M_PI}, 1000, 100);
  robot.moveAppend({movel3}, id, ec);
  robot.moveStart(ec);
  waitRobot(robot, moving);
}

/**
 * @brief Example - adjusting the velocity scale during motion
 */
void adjustSpeed(BaseRobot &robot) {
  error_code ec;
  std::string id;
  double scale = 0.5;
  robot.adjustSpeedOnline(scale, ec); // Set the starting speed ratio to 50%

  // For this example: move back and forth between waypoints cmd1 and cmd2
  rokae::MoveAbsJCommand cmd1({0, 0, 0, 0, 0, 0}), cmd2({1.5, 1.5,1.5,1.5,1.5,1.5});
  robot.moveAppend({cmd1, cmd2,cmd1,cmd2,cmd1,cmd2,cmd1,cmd2}, id, ec);
  robot.moveStart(ec);
  bool running = true;

  // Read keyboard input
  std::thread readInput([&]{
    while(running) {
      auto ch = std::getchar();
      if(ch == 'a') {
        if(scale < 0.1) { print(std::cerr, "Already at 1%"); continue; }
        scale -= 1e-1;
      } else if(ch == 'd'){
        if(scale > 1) { print(std::cerr, "Already at 100%"); continue; }
        scale += 1e-1;
      } else { continue; }
      robot.adjustSpeedOnline(scale, ec);
      print(os, "Adjusted to", scale);
    }
  });
  print(os, "Robot motion started, press [a] to decrease speed, [d] to increase speed, step size 10%");

  // Wait for motion to finish
  waitRobot(robot, running);
  readInput.join();
}

/**
 * @brief Example - using joint configuration data (confData) to resolve multiple IK solutions, waypoints apply to the xMateCR7 model
 * Example - use joint configure data to get the desired IK result
 */
void multiplePosture(xMateRobot &robot) {
  error_code ec;
  std::string id;

  // This example uses the default tool/workpiece
  // use default tool and wobj frame
  Toolset defaultToolset;
  robot.setToolset(defaultToolset, ec);
  // Set the robot to use confData when computing the IK solution
  robot.setDefaultConfOpt(true, ec);

  MoveJCommand moveJ({0.2434, -0.314, 0.591, 1.5456, 0.314, 2.173});
  // The same end-effector posture but different confData produces different joint angles
  // the target posture is same, but give different joint configure data
  moveJ.target.confData =  {-67, 100, 88, -79, 90, -120, 0, 0};
  // Example: set the joint speed percentage to 10%. If not set, joint speed is derived from the end-effector's linear speed
  moveJ.jointSpeed = 0.1;
  robot.moveAppend({moveJ}, id, ec);

  moveJ.target.confData =  {-76, 8, -133, -106, 103, 108, 0, 0};
  robot.moveAppend({moveJ}, id, ec);
  moveJ.target.confData =  {-70, 8, -88, 90, -105, -25, 0, 0};
  robot.moveAppend({moveJ}, id, ec);

  robot.moveStart(ec);
  waitForFinish(robot, id, 0);
  robot.setDefaultConfOpt(false, ec);
}

/**
 * @brief Example - motion with an external axis (rail). Waypoints apply to the xMateSR4 model
 */
 template <WorkType wt, unsigned short dof>
void moveWithRail(Robot_T<wt, dof> *robot) {
  error_code ec;
  bool is_rail_enabled;
  robot->getRailParameter("enable", is_rail_enabled, ec);
  if(!is_rail_enabled) {
    print(os, "External axis (rail) not enabled");
    return;
  }

  // Enable/disable the rail, set rail parameters
  // Setting rail parameters and the base frame requires a controller restart to take effect; this only demonstrates the API calls
  robot->setRailParameter("enable", true, ec); // Enable the rail
  robot->setRailParameter("maxSpeed", 1, ec); // Set max speed to 1 m/s
  robot->setRailParameter("softLimit", std::vector<double>({-0.8, 0.8}), ec); // Set soft limits to +-0.8m
  robot->setRailParameter("reductionRatio", 1.0, ec); // Set the reduction ratio

  auto curr = robot->BaseRobot::jointPos(ec);
  print(os, "Current joint angles", robot->BaseRobot::jointPos(ec));

  // *** Jog rail example ***
  // Power on in manual mode
  robot->setOperateMode(OperateMode::manual, ec);
  robot->setPowerState(true, ec);
  std::vector<double> soft_limit;
  robot->getRailParameter("softLimit", soft_limit, ec);
  // Jog within the soft limits
  double step = (curr.back() - soft_limit[0] > 0.1 ? 0.1 : (curr.back() - soft_limit[0])) * 1000.0;
  // Using six-axis joint-space jogging as an example: index 0~5 represent axes 1-6, index=6 represents the first external axis
  int ex_jnt_index = robot->robotInfo(ec).joint_num;
  // Move the rail axis 100mm in the negative direction in joint space
  robot->startJog(JogOpt::jointSpace, 0.6, step, ex_jnt_index, false, ec);
  // Wait for the jog to finish
  while(true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if(robot->operationState(ec) != OperationState::jogging) break;
  }
  robot->stop(ec);

  // *** Motion command example with a rail ***
  CartesianPosition p0({0.56, 0.136, 0.416, M_PI, 0, M_PI}), p1({0.56, 0.136, 0.3, M_PI, 0, M_PI});
  p0.external = { 0.02 }; // Rail moves to 0.02m, same below
  p1.external = { -0.04 };
  MoveAbsJCommand abs_j_command({0, M_PI/6, -M_PI_2,0, -M_PI/3, 0 });
  abs_j_command.target.external = { 0.1 }; // Rail moves to 0.1m
  MoveJCommand j_command(p0);
  MoveLCommand l_command(p1);
  MoveCCommand c_command(p1, p0);

  // Add custom info, which will be returned in the motion info feedback
  l_command.customInfo = "hello";

  std::string id;
  robot->moveAppend(abs_j_command, id, ec);
  robot->moveAppend(j_command, id, ec);
  robot->moveAppend(l_command, id, ec);
  robot->moveAppend(abs_j_command, id, ec);
  robot->moveAppend(c_command, id, ec);
  robot->moveStart(ec);
  waitForFinish(*robot, id, 0);
}

/**
 * @brief Find the first enabled external-axis mechanical unit (u1~u6)
 * @param[out] mech_unit Mechanical unit name, e.g. "u1"
 * @param[out] fixed_name Mechanical unit's fixed name, used for startJogWithExt
 * @return Whether an enabled mechanical unit was found
 */
static bool findEnabledMechUnit(BaseRobot *robot, std::string &mech_unit, std::string &fixed_name, error_code &ec) {
  print(os, "Scanning mechanical units u1~u6 ...");
  const char *units[] = {"u1", "u2", "u3", "u4", "u5", "u6"};
  for (const char *u : units) {
    bool enabled = false;
    ec.clear();
    robot->getMechUnit(u, "enable", enabled, ec);
    if (ec) {
      print(os, "[External axis] ", u, " enable query failed", ec);
      continue;
    }
    print(os, "[External axis] ", u, " enable =", enabled);
    if (!enabled) {
      continue;
    }
    mech_unit = u;
    fixed_name = u;
    ec.clear();
    std::string name_from_ctrl;
    robot->getMechUnit(u, "fixed_name", name_from_ctrl, ec);
    if (!ec && !name_from_ctrl.empty()) {
      fixed_name = name_from_ctrl;
    } else {
      ec.clear();
    }
    print(os, "Found enabled mechanical unit:", mech_unit, "Jog fixed_name:", fixed_name);
    return true;
  }
  return false;
}

/**
 * @brief Get the name of the first external axis under a mechanical unit, used for parameter queries
 * @param[in] mech_unit u1~u6
 * @return axis1~axis6, defaults to "axis1"
 */
static std::string firstExtAxisName(BaseRobot *robot, const std::string &mech_unit, error_code &ec) {
  std::vector<std::string> axes_info;
  robot->getMechUnit(mech_unit, "axes_info", axes_info, ec);
  if (!ec && !axes_info.empty()) {
    print(os, "Mechanical unit", mech_unit, "external axis list:", axes_info);
    return axes_info.front();
  }
  ec.clear();
  return "axis1";
}

/**
 * @brief Example - motion with an external axis (parallels moveWithRail)
 * @note External axis parameters are generally configured on the teach pendant/controller; the SDK provides
 *       query and Jog access, plus the external field on motion commands.
 *       The automated trajectory loop runs 3 cycles: each cycle runs MoveAbsJ/MoveJ/MoveL/MoveC, then the rail
 *       returns to its soft-limit origin before the next cycle begins.
 *       Waypoints are tuned for models with an external axis, such as the xMateSR4; adjust the waypoints for other models.
 *       startJogWithExt's fixed_name accepts either a mechanical unit name (e.g. "u1") or the fixed_name read via getMechUnit.
 */
template <WorkType wt, unsigned short dof>
void moveWithExtAxis(Robot_T<wt, dof> *robot) {
  print(os, "======== External axis example moveWithExtAxis starting ========");
  error_code ec;
  std::string mech_unit;
  std::string fixed_name;

  if (!findEnabledMechUnit(robot, mech_unit, fixed_name, ec)) {
    print(os, "No enabled external-axis mechanical unit found (u1~u6); enable an external axis on the controller and retry", ec);
    return;
  }

  const std::string axis_name = firstExtAxisName(robot, mech_unit, ec);
  if (ec) {
    print(os, "Failed to read external axis info", ec);
    return;
  }

  double max_speed = 0;
  double soft_lower = 0, soft_upper = 0;
  robot->getExtAxisInfo(axis_name, "max_speed", max_speed, ec);
  print(os, axis_name, "max_speed:", max_speed, ec);
  robot->getExtAxisInfo(axis_name, "soft_limit_lower", soft_lower, ec);
  robot->getExtAxisInfo(axis_name, "soft_limit_upper", soft_upper, ec);
  print(os, axis_name, "soft limits: [", soft_lower, ", ", soft_upper, "]", ec);

  print(os, "Current joint angles", robot->BaseRobot::jointPos(ec));

  // *** Jog external axis example ***
  robot->setOperateMode(OperateMode::manual, ec);
  robot->setPowerState(true, ec);

  // fixed_name takes a mechanical unit name u1~u6; index=0 means the first external axis on that unit; is_ext=true means jog the external axis
  const double jog_rate = 0.1;
  const double jog_step = 5.0; // Joint space, in degrees
  const unsigned jog_index = 0;
  print(os, "[Jog] fixed_name=", fixed_name, " index=", jog_index, " rate=", jog_rate, " step=", jog_step);
  robot->startJogWithExt(JogOpt::jointSpace, jog_rate, jog_step, jog_index, true, fixed_name, ec, true);
  if (ec) {
    print(os, "startJogWithExt failed", ec);
    return;
  }
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (robot->operationState(ec) != OperationState::jogging) {
      break;
    }
  }
  robot->stop(ec);

  // *** Motion command example with an external axis (loops 3 cycles; the rail returns to origin after each cycle) ***
  robot->setOperateMode(OperateMode::automatic, ec);
  robot->setPowerState(true, ec);
  robot->setToolset(Predefines::defaultToolset, ec);
  robot->setDefaultSpeed(200, ec);
  robot->setDefaultZone(50, ec);
  robot->setDefaultConfOpt(false, ec);

  // external is in meters; soft limits are generally in mm (e.g. 0~1300). Do not use negative values, or the lookahead will report error 50129.
  const double ext_lo_m = soft_lower / 1000.0;
  const double ext_hi_m = soft_upper / 1000.0;
  const double ext_min_m = ext_lo_m + 0.01;
  const double ext_max_m = ext_hi_m - 0.01;
  const double ext_origin_m = ext_min_m;
  auto clampExt = [&](double v) -> double {
    if (v < ext_min_m) {
      return ext_min_m;
    }
    if (v > ext_max_m) {
      return ext_max_m;
    }
    return v;
  };

  constexpr double kExtStepM = 0.02;
  constexpr double kStepZ = 0.010;
  constexpr double kStepXy = 0.008;
  constexpr int kCartSpeed = 200;
  constexpr int kCartZone = 50;
  constexpr int kCycleCount = 3;
  const MoveWaitCommand wait_5s(std::chrono::seconds(5));

  const std::vector<double> q_work = {0, M_PI / 6, -M_PI_2, 0, -M_PI / 3, 0};
  std::array<double, 6> q6{};
  for (unsigned i = 0; i < 6; ++i) {
    q6[i] = q_work[i];
  }
  CartesianPosition p_ref = robot->model().calcFk(q6, ec);
  if (ec) {
    print(os, "calcFk failed", ec);
    return;
  }
  print(os, "Reference motion posture (FK)", p_ref);
  print(os, "Rail origin external(m):", ext_origin_m, "step per cycle", kExtStepM * 1000.0, "mm, for", kCycleCount, "cycles total");

  for (int cycle = 1; cycle <= kCycleCount; ++cycle) {
    const double ext1 = clampExt(ext_origin_m + kExtStepM);
    const double ext2 = clampExt(ext_origin_m + 2.0 * kExtStepM);
    const double ext3 = clampExt(ext_origin_m + 3.0 * kExtStepM);
    print(os, "----- Cycle", cycle, "/", kCycleCount, "-----");
    print(os, "Rail external(m):", ext_origin_m, "->", ext1, "->", ext2, "->", ext3, "->", ext_origin_m);

    CartesianPosition p_j = p_ref;
    p_j.external = {ext1};
    clearCartConf(p_j);

    CartesianPosition p_l = p_ref;
    p_l.trans[2] -= kStepZ;
    p_l.external = {ext2};
    clearCartConf(p_l);

    CartesianPosition p_c_tgt = p_ref;
    p_c_tgt.trans[0] += kStepXy;
    p_c_tgt.external = {ext3};
    clearCartConf(p_c_tgt);
    CartesianPosition p_c_aux = p_ref;
    p_c_aux.trans[1] += kStepXy;
    p_c_aux.external = {ext3};
    clearCartConf(p_c_aux);

    CartesianPosition p_home = p_ref;
    p_home.external = {ext_origin_m};
    clearCartConf(p_home);

    MoveAbsJCommand abs_j_command(q_work);
    abs_j_command.target.external = {ext1};
    MoveJCommand j_command(p_j, kCartSpeed, kCartZone);
    MoveLCommand l_command(p_l, kCartSpeed, kCartZone);
    MoveCCommand c_command(p_c_tgt, p_c_aux);
    MoveLCommand home_command(p_home, kCartSpeed, kCartZone);
    l_command.customInfo = "ext_axis_demo";

    std::string id;
    print(os, "MoveAbsJ joints", q_work, "external(m)", ext1);
    robot->moveAppend(abs_j_command, id, ec);
    if (ec) {
      print(os, "moveAppend(MoveAbsJ) failed", ec);
      return;
    }
    robot->moveAppend(wait_5s, id, ec);

    print(os, "MoveJ target", j_command.target, "external(m)", ext1);
    robot->moveAppend(j_command, id, ec);
    robot->moveAppend(wait_5s, id, ec);

    print(os, "MoveL target", l_command.target, "external(m)", ext2);
    robot->moveAppend(l_command, id, ec);
    robot->moveAppend(wait_5s, id, ec);

    print(os, "MoveC target", c_command.target, "aux", c_command.aux, "external(m)", ext3);
    robot->moveAppend(c_command, id, ec);
    robot->moveAppend(wait_5s, id, ec);

    print(os, "Return to origin, MoveL target", home_command.target, "external(m)", ext_origin_m);
    robot->moveAppend(home_command, id, ec);

    robot->moveStart(ec);
    if (ec) {
      print(os, "moveStart failed, cycle", cycle, ec);
      return;
    }
    bool running = true;
    waitRobot(*robot, running);
    print(os, "Cycle", cycle, "complete, actual waypoint", robot->cartPosture(CoordinateType::endInRef, ec), ec);
    print(os, "Cycle", cycle, "complete, joint angles", robot->jointPos(ec));
  }

  print(os, "======== External axis example moveWithExtAxis finished (", kCycleCount, "cycles total) ========");
}

/**
 * @brief Reachability check example, waypoints apply to the xMateER7 model
 */
void checkPath_Example(xMateRobot &robot) {
  error_code ec;

  // Starting position
  CartesianPosition start {0.631250, 0.0, 0.507386, M_PI, 0.0, M_PI };
  // Starting joint angles. Note: must supply the correct starting joint angles that correspond to the starting position
  std::vector<double> start_joint = { 0.000, M_PI / 6, M_PI / 3, 0.0, M_PI_2, 0.0};
  // Target position
  CartesianPosition target {0.615167, 0.141585, 0.507386, M_PI, 0.0, -167.039 * M_PI / 180};

  // Check reachability of a single-waypoint linear motion
  auto calculated_target_joint = robot.checkPath(start, start_joint, target, ec);
  if(ec) {
    print(os, "Linear trajectory unreachable ", ec);
  } else {
    print(os, "Linear trajectory reachability check passed, calculated target joint angles:", calculated_target_joint);
  }

  // Check reachability of a multi-waypoint linear motion
  CartesianPosition target2 {0.615167, 0.141585, 0.517386, M_PI, 0.0, -167.039 * M_PI / 180};
  std::vector<double> target_joint;
  std::vector<CartesianPosition> waypoints = {start, target, target2}; // Starting point and subsequent waypoints
  auto error_index = robot.checkPath(start_joint, waypoints, target_joint, ec);
  if(ec) {
    print(os, "Multi-waypoint check: waypoint", error_index, "is unreachable", ec);
  } else {
    print(os, "Multi-waypoint linear trajectory reachability check passed, calculated target joint angles:", target_joint);
  }

  // Check an arc path
  CartesianPosition aux ({0.583553, 0.134309, 0.628928, M_PI, 11.286 * M_PI / 180, -167.039 * M_PI / 180});
  calculated_target_joint = robot.checkPath(start, start_joint, aux, target, ec);
  if(ec) {
    print(os, "Arc trajectory unreachable", ec);
  } else {
    print(os, "Arc trajectory reachability check passed, calculated target joint angles:", calculated_target_joint);
  }

  // Check a full-circle motion
  // Full circle rotation of 360 degrees, constant posture
  calculated_target_joint = robot.checkPath(start, start_joint, aux, target, ec, M_PI * 2,
                               MoveCFCommand::RotType::constPose);
  if(ec) {
    print(os, "Full-circle trajectory unreachable", ec);
  } else {
    print(os, "Full-circle trajectory reachability check passed, calculated target joint angles:", calculated_target_joint);
  }
}

/**
 * @brief Example - inserting a dwell between motion commands, waypoints apply to the XMS5-R800 model
 */
void moveWithDwellTime(xMateRobot &robot) {
  // Starting point
  MoveJCommand movej0 ({0.614, 0.136, 0.389, -M_PI, 0, M_PI });
  // Multi-segment linear trajectory
  std::vector<MoveLCommand> movel_list = {
    {{0.444155, -0.299134, -0.0678978, 2.82899, 0.0994708, 1.34719}},
    {{0.435115, -0.29386, -0.0680401, 2.82923, 0.0961299, 1.35047}},
    {{0.44555, -0.293048, -0.0681824, 2.82947, 0.092789, 1.35376}},
    {{0.43651, -0.287774, -0.0683246, 2.82971, 0.0894481, 1.35704}},
    {{0.446944, -0.286963, -0.0684669, 2.82996, 0.0861072, 1.36032}},
    {{0.437905, -0.281688, -0.0686092, 2.8302, 0.0827663, 1.36361}},
    {{0.448339, -0.280877, -0.0687515, 2.83044, 0.0794254, 1.36689}},
    {{0.439299, -0.275602, -0.0688938, 2.83068, 0.0760845, 1.37017}},
    {{0.449734, -0.274791, -0.0690361, 2.83092, 0.0727436, 1.37346}},
    {{0.440694, -0.269517, -0.0691784, 2.83117, 0.0694027, 1.37674}},
    {{0.451128, -0.268705, -0.0693206, 2.83141, 0.0660618, 1.38002}},
    {{0.442089, -0.263431, -0.0694629, 2.83165, 0.0627209, 1.38331}},
  };

  // Dwell 300ms between consecutive commands
  MoveWaitCommand wait_cmd(std::chrono::milliseconds(300));

  error_code ec;
  std::string cmd_id;

  robot.setToolset(Predefines::defaultToolset, ec);
  robot.moveAppend(movej0, cmd_id, ec);

  Toolset toolset;
  toolset.end.trans = {0, 0.07763, 0.49047};
  robot.setToolset(toolset, ec);

  // Dwell once after each MoveL segment
  for(auto &cmd : movel_list) {
    robot.moveAppend(cmd, cmd_id, ec);
    robot.moveAppend(wait_cmd, cmd_id, ec);
  }
  robot.moveStart(ec);

  waitForFinish(robot, cmd_id, 0);
}

/**
 * @brief main program
 */
int main() {
  try {
    using namespace rokae;

    // *** 1. Connect to the robot ***
    // *** 1. Connect to the robot ***
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateRobot robot(ip); // ****   xMate 6-axis

    // *** 2. Switch to auto mode and motor on ***
    // *** 2. Switch to auto mode and motor on ***
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);

    // *** 3. Set the default motion speed and turning zone ***
    // *** 3. set default speed and turning zone ***
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robot.setDefaultZone(50, ec); // Optional: set the default turning zone
    robot.setDefaultSpeed(200, ec); // Optional: set the default speed

    // Optional: set a callback for motion command completion and error info
    // Optional: set motion event notification
    robot.setEventWatcher(Event::moveExecution, printInfo, ec);

    // *** 4. Demo motion program ***
    // *** 4. demo motion program ***
    // multiplePosture(robot);

    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);
  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}
