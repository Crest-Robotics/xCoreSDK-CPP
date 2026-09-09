/**
 * @file path_record.cpp
 * @brief Collaborative robot drag teaching, path recording and playback
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#include <iostream>
#include <thread>
#include <unordered_map>
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace std;
using namespace rokae;

char parseInput(std::string &str);
void WaitRobot(BaseRobot *robot);
void printHelp();

/**
 * @brief Print motion execution info
 */
void printInfo(const rokae::EventInfo &info) {
  using namespace rokae::EventInfoKey::MoveExecution;
  print(std::cout, "[Motion execution info] ID:", std::any_cast<std::string>(info.at(ID)), "Index:", std::any_cast<int>(info.at(WaypointIndex)),
        "Completed: ", std::any_cast<bool>(info.at(ReachTarget)) ? "YES": "NO", std::any_cast<error_code>(info.at(Error)),
        std::any_cast<std::string>(info.at(Remark)));
}

/**
 * @brief main program
 */
int main() {
  try {
    std::string ip = "192.168.0.160";
    error_code ec;
    std::vector<std::string> paths;
    xMateRobot robot(ip); // xMate 6-axis model

    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    // Like other motion commands, the drag-playback command also reports motion completion via the motion-info callback
    robot.setEventWatcher(Event::moveExecution, printInfo, ec);

    printHelp();

    char cmd = ' ';
    while(cmd != 'q') {
      std::string str;
      // Read a command from the console
      getline(std::cin, str);
      cmd = parseInput(str);

      switch(cmd) {
        case 'p':
          if(str == "on") { robot.setPowerState(true, ec); std::cout << "* Robot powered on\n"; }
          else { robot.setPowerState(false, ec); std::cout << "* Robot powered off\n"; }
          if(ec) break; continue;
        case 'm':
          if(str == "manual") { robot.setOperateMode(OperateMode::manual, ec); std::cout << "* Manual mode\n"; }
          else { robot.setOperateMode(OperateMode::automatic, ec); std::cout << "* Automatic mode\n"; }
          if(ec) break; continue;
        case 'd':
          // Precondition for enabling drag: the robot operate mode must be switched to manual and the robot powered off
          if(str == "on") { robot.enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec); cout << "* Drag enabled\n"; }
          else { robot.disableDrag(ec); std::cout << "* Drag disabled\n"; }
          if(ec) break; continue;
        case 'a':
          robot.startRecordPath(30, ec); std::cout << "* Started recording path\n";
          if(ec) break; continue;
        case 'b':
          robot.stopRecordPath(ec); std::cout << "* Stopped recording path\n";
          if(ec) break; continue;
        case 's':
          robot.saveRecordPath(str, ec); std::cout << "* Saved path as: " << str << endl;
          if(ec) break; continue;
        case 'c':
          robot.cancelRecordPath(ec); cout << "* Recording cancelled\n";
          if(ec) break; continue;
        case 'u':
          paths = robot.queryPathLists(ec);
          if(paths.empty()) cout << "* No saved paths\n";
          else {
            cout << "* Saved paths: ";
            for(auto p : paths) cout << p << ", ";
            cout << endl;
          }
          if(ec) break; continue;
        case 'v':
          cout << "* Deleting path \"" << str << "\"\n";
          robot.removePath(str, ec);
          if(ec) break; continue;
        case 'r': {
          robot.replayPath(str, 1.0, ec);
          if (ec) break;
          robot.moveStart(ec);
          if (ec) break;
          cout << "* Starting path playback \"" << str << "\", rate 100%\n";
          WaitRobot(&robot);
          cout << "* Playback finished\n";
          continue;
        }
        case 'z':
          robot.moveReset(ec); cout << "* Motion buffer reset\n";
          if(ec) break; continue;
        case 'h':
          printHelp(); continue;
        case 'q':
          std::cout << " --- Quit --- \n"; continue;
        default:
          std::cerr << "Invalid input\n"; continue;
      }
      cerr << "! Error message: " << ec.message() << endl;
    }

    robot.disconnectFromRobot(ec);
  } catch (const rokae::Exception &e) {
    std::cerr << e.what();
  }

  return 0;
}

static const std::unordered_map<std::string, char> ConsoleInput = {
  {"quit", 'q'},
  {"power", 'p'},
  {"drag", 'd'},
  {"mode", 'm'},
  {"start", 'a'},
  {"stop", 'b'},
  {"save", 's'},
  {"cancel", 'c'},
  {"query", 'u'},
  {"remove", 'v'},
  {"reset", 'z'},
  {"replay", 'r'},
  {"help", 'h'}
}; ///< command -> char

/**
 * @brief Print usage instructions
 */
void printHelp() {
  cout << " --- Drag teaching and path playback example --- " << endl
       << "Format <command>[:parameter] e.g. save:track0" << endl << endl
       << "Command               |  Parameter"        << endl
       << "power   Power robot on/off   | on|off"      << endl
       << "mode    Manual/automatic mode | manual|auto" << endl
       << "drag    Enable/disable drag  | on|off"      << endl
       << "start   Start recording path |"             << endl
       << "stop    Stop recording path  |"             << endl
       << "save    Save path      | path name"      << endl
       << "cancel  Cancel recording |"             << endl
       << "query   Query saved paths |"            << endl
       << "remove  Remove path    | path name"     << endl
       << "reset   Reset motion buffer |"             << endl
       << "replay  Replay path    | path name"      << endl
       << "quit    Quit\n";
}

/**
 * @brief Handle console input
 */
char parseInput(std::string &str) {
  size_t delimiter;
  std::string cmd(str);
  if((delimiter = str.find(':')) != std::string::npos) {
    cmd = str.substr(0, delimiter);
    str = str.substr(delimiter + 1);
  }
  if(ConsoleInput.count(cmd))  return ConsoleInput.at(cmd);
  else return ' ';
}

/**
 * @brief Wait for the robot's motion to finish
 */
void WaitRobot(BaseRobot *robot) {
  bool running = true;
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    error_code ec;
    auto st = robot->operationState(ec);
    if(st == OperationState::idle || st == OperationState::unknown){
      running = false;
    }
  }
}