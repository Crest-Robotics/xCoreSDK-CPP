/**
 * @file rl_project.cpp
 * @brief Load and run an RL project
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#define _SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING

#include <iostream>
#include <unordered_map>
#include "rokae/robot.h"
#include "print_helper.hpp"
#include <string>

#ifdef _WIN32
#include <windows.h>
 #endif

using namespace std;
using namespace rokae;

namespace {
 rokae::xMateRobot robot;
}

void printHelp();
static const std::unordered_map<std::string, char> ConsoleInput = {
  {"on", '0'}, {"off", 'x'}, {"quit", 'q'},
  {"info", 'i'}, {"load", 'l'}, {"main", 'm'},
  {"start", 's'}, {"pause", 'p'}, {"tool", 't'},
  {"wobj", 'w'}, {"opt", 'o'}, {"help", 'h'}
}; ///< command -> char

/**
 * @brief Receive the RL program's executing line number
 */
void rlExecutionCb(const EventInfo &info) {
  using namespace EventInfoKey::RlExecution;
  std::string task_name = std::any_cast<std::string>(info.at(TaskName));
  std::string lookahead_file = std::any_cast<std::string>(info.at(LookaheadFile));
  int lookahead_line = std::any_cast<int>(info.at(LookaheadLine));
  std::string execute_file = std::any_cast<std::string>(info.at(ExecuteFile));
  int execute_line = std::any_cast<int>(info.at(ExecuteLine));
  std::cout << "[RL Execution] Task Name: " << task_name << " , Lookahead: " << lookahead_file << ", " <<
  lookahead_line << ", Executing: " << execute_file << ", " << execute_line << std::endl;
}

std::string UTF8ToGBK(const std::string& utf8Str)
{
#if defined(_WIN32) || defined(_WIN64)
  // Step 1: convert UTF-8 to wide characters
  int wcharSize = MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, nullptr, 0);
  if (wcharSize == 0) {
    return "";
  }

  std::vector<wchar_t> wcharBuffer(wcharSize);
  MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, wcharBuffer.data(), wcharSize);

  // Step 2: convert wide characters to GBK
  int gbkSize = WideCharToMultiByte(CP_ACP, 0, wcharBuffer.data(), -1, nullptr, 0, nullptr, nullptr);
  if (gbkSize == 0) {
    return "";
  }

  std::vector<char> gbkBuffer(gbkSize);
  WideCharToMultiByte(CP_ACP, 0, wcharBuffer.data(), -1, gbkBuffer.data(), gbkSize, nullptr, nullptr);

  return std::string(gbkBuffer.data());
#else
  return utf8Str; // here we assume Linux use UTF-8 encoding as default
#endif
}

/**
 * @brief Controller log report
 */
void logReport(const EventInfo& info) {
  using namespace EventInfoKey::LogReporter;
  int log_ecode = std::any_cast<int>(info.at(Ecode));
  auto log_edetail = std::any_cast<std::string>(info.at(Edetail));
  if (!log_edetail.empty())
  {
    log_edetail = UTF8ToGBK(log_edetail); // convert encoding
  }
  std::cout << "User Log Report - error code: " << log_ecode <<
  (log_edetail.empty() ? "" : ", detail: " + log_edetail) << std::endl;

}

/*
 * @brief Example - Import and delete an RL project
 */
void transferRLProject() {
  error_code ec;

  // Import an RL project in .zip format
  auto project_name = robot.importProject("ExampleRLProject.zip", true, ec);
  if (!ec) {
  	print(std::cout, project_name);
  }
  else {
  	std::cerr << "Error occurred while importing the RL project: " << ec.message() << std::endl;
  }

  // Delete the project
  robot.removeProject("test1", ec);
}

/**
 * @brief Example - Import and delete project files
 */
void importProjectFile() {
  error_code ec;
  // Import the local test.mod into project MyRlProject, under task task0
  auto ret = robot.importFile(R"(C:\Users\rokae\Desktop\test.mod)", "project/MyRlProject/task0", true, ec);

  if(ec){
    std::cerr << "Import failed" << ": " << ec << std::endl;
  }
  std::cout << "File name after import" << ret << std::endl;

  // Import the local test.mod into project MyRlProject, under task task0, and rename it to imported.mod
  robot.importFile(R"(C:\Users\rokae\Desktop\test.mod)", "project/MyRlProject/task0/imported.mod", true, ec);

  // Import the project's tool configuration files into project MyRlProject
  robot.importFile(R"(C:\Users\rokae\Desktop\MyRlProject\tool.json)", "project/MyRlProject", true, ec);
  robot.importFile(R"(C:\Users\rokae\Desktop\MyRlProject\_build\tools.sys)", "project/MyRlProject", true, ec);

  // Delete imported.mod under MyRlProject task0
  robot.removeFiles({"project/MyRlProject/task0/imported.mod"}, ec);
  // Delete MyRlProject task1
  robot.removeFiles({"project/MyRlProject/task1"}, ec);
}

/**
 * @brief Example - Set tool/workpiece pose, load, and other information
 */
void setProjectToolWobj() {
  error_code ec;
  // Set the global tool/workpiece g_tool_0
  // Hand-held, X:0, Y:45mm, Z:0, A:0, B:90°, C:0. Load 1 kg, center of mass X:0, Y:20mm, Z:0
  WorkToolInfo g_tool_0("g_tool_0", true, {0, 0.045, 0, 0, M_PI / 2, 0}, {1, { 0, 0.02, 0 }, {}});
  g_tool_0.alias = "tool for job1"; // Additional description of the tool
  robot.setToolInfo(g_tool_0, ec);

  // Set/create tool tool_1 under the RL project. A project must be loaded first
  WorkToolInfo tool_1("tool_1", true, {0.1, 0.1, 0, 0, M_PI, 0}, {1, { 0.05, 0.05,0 }, {}});
  robot.setToolInfo(tool_1, ec);

  // Set the global workpiece g_wobj_0, an external workpiece
  WorkToolInfo g_wobj_0("g_wobj_0", false, {0.1, 0.1, 0, 0, M_PI, 0}, {0, {}, {}});
  robot.setWobjInfo(g_wobj_0, ec);
}

/**
 * @brief main program
 */
int main() {
  try {
    std::string ip = "192.168.0.160";
    robot.connectToRobot(ip); // ****   xMate 6-axis
  } catch (const rokae::Exception &e) {
    std::cout << e.what();
    return 0;
  }

  error_code ec;
  robot.setMotionControlMode(MotionControlMode::NrtRLTask,ec);
  // Receive the task name and line number of the executing RL program
  robot.setEventWatcher(rokae::Event::rlExecution, rlExecutionCb, ec);
  // Controller log report
  robot.setEventWatcher(rokae::Event::logReporter, logReport, ec);

  robot.setOperateMode(OperateMode::automatic, ec);
  printHelp();
  char cmd = ' ';
  while(cmd != 'q') {
    std::string str;
    getline(cin, str);
    if(ConsoleInput.count(str)){
      cmd = ConsoleInput.at(str);
    } else {
      cmd = ' '; }

    switch(cmd) {
      case '0':
        robot.setPowerState(true, ec); cout << "* Robot powered on\n";
        if(ec) break; continue;
      case 'x':
        robot.setPowerState(false, ec); cout << "* Robot powered off\n";
        if(ec) break; continue;
      case 'i': {
        cout << "* Query project information:\n";
        auto infos = robot.projectsInfo(ec);
        if(infos.empty()) { cout << "No projects\n"; }
        else {
          for(auto &info : infos) {
            cout << "Name: " << info.name << " Tasks: ";
            for(auto &t: info.taskList) {
              cout << t << " ";}
            cout << endl; }
        }
        if(ec) break; } continue;
      case 'l':{
        cout << "* Load project, please enter the name of the project to load: ";
        std::string name, line, task;
        vector<string> tasks;
        getline(cin, name);
        cout << "Please enter the tasks to run, separated by spaces: ";
        getline(cin, line);
        istringstream iss(line);
        while (iss >> task)
          tasks.push_back(task);
        robot.loadProject(name, tasks, ec);
        if(ec) break; } continue;
      case 'm':
        robot.ppToMain(ec);
        cout << "* Program pointer set to main\n";
        if(ec) break; continue;
      case 's':
        robot.runProject(ec); cout << "* Started running the project\n";
        if(ec) break; continue;
      case 'p':
        robot.pauseProject(ec); cout << "* Paused\n";
        if(ec) break; continue;
      case 't': {
        cout << "* Query tool information\n";
        auto tools = robot.toolsInfo(ec);
        if(tools.empty()) cout << "No tools\n";
        else {
          for(auto &tool : tools) {
            cout << "Tool: " << tool.name << ", mass: " << tool.load.mass << endl;
          } }
        if(ec) break; } continue;
      case 'w': {
        cout << "* Query workpiece information\n";
        auto wobjs = robot.wobjsInfo(ec);
        if(wobjs.empty()) cout << "No workpieces\n";
        else {
          for(auto &wobj:wobjs) {
            cout << "Workpiece: " << wobj.name << ", robot-held: " << boolalpha << wobj.robotHeld << endl;}
        }
        if(ec) break; } continue;
      case 'o':{
        cout << "* Set running parameters, please enter the running rate and whether to loop ([0] single run / [1] loop), separated by a space: ";
        double rate; bool isLoop; string line;
        getline(cin, line);
        istringstream iss(line);
        iss >> rate >> isLoop;
        robot.setProjectRunningOpt(rate, isLoop, ec);
        if(ec) break;} continue;
      case 'h':
        printHelp(); continue;
      case 'q':
        std::cout << " --- Quit --- \n"; continue;
      default:
        std::cerr << "Invalid input\n"; continue;
    }
    cerr << "! Error message: " << ec.message() << endl;
  }
  return 0;
}

/**
 * @brief print help
 */
void printHelp() {
  cout << " --- Run RL project example --- \n\n"
  << "     command   \n"
  << "on    power on the robot\n"
  << "off   power off the robot\n"
  << "info  query the project list\n"
  << "load  load a project\n"
  << "main  set program pointer to main\n"
  << "start start running\n"
  << "pause pause running\n"
  << "opt   set running parameters\n"
  << "tool  query tool information\n"
  << "wobj  query workpiece information\n"
  << "help  show all commands of the example program\n"
  << "quit  exit\n";
}