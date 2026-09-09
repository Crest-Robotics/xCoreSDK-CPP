/**
 * @file controller_upgrade.cpp
 * @brief Controller upgrade and backup export example
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#include <iostream>
#include <chrono>
#include <iomanip>
#if defined(_WIN32) || defined(_WIN64)
#include <filesystem>
#endif
#include "rokae/robot.h"
#include "rokae/upgrade.h"

using namespace rokae;

namespace {
 BaseUpgrade upgrader; ///< Upgrade program instance
}

/**
 * @brief Output the formatted current date and time
 */
std::string getCurrentDateTime();

/**
 * @brief Example - Controller firmware upgrade / restore controller backup
 */
void upgradeController() {
  error_code ec;

  std::cout << "Starting controller firmware upgrade" << std::endl;
  // Upgrade the controller to the version at this path
  upgrader.upgrade(R"(C:\Users\rokae\v3.1.2.rpa)", ec);
  if(ec) {
    std::cerr << "Upgrade failed: " << ec.message() << std::endl;
  } else {
    std::cout << "Upgrade succeeded" << std::endl;
  }
}

/**
 * @brief Example - Export controller backup
 */
void exportControllerBackup() {
  error_code ec;
  // Saved file name: export_[date_time].rpa
  std::string export_file_name = "export_" + getCurrentDateTime() + ".rpa";

#if defined(_WIN32) || defined(_WIN64)
  // File is saved in the current working directory
  std::string file_save_path = (std::filesystem::current_path() / export_file_name).string();
#else
  // File is saved in the current working directory
  std::string file_save_path = "./" + export_file_name;
 #endif

  std::cout << "Starting controller backup export, saving to " << file_save_path << std::endl;

  // Export controller logs and RL project files
  upgrader.exportBackup(file_save_path, {
    BackupItem::controllerLog, BackupItem::rlProgram}, ec);
  if(ec) {
    std::cerr << "Export failed: " << ec.message() << std::endl;
  } else {
    std::cout << "Export succeeded" << std::endl;
  }
}

/**
 * @brief Main function
 */
int main() {

  std::string remote_ip = "192.168.0.160"; // Robot address

  // Connect to the controller's upgrade program
  // Can be connected independently; there is no requirement to also connect to the robot controller. Multiple connections are not allowed, and connecting at the same time as the teach pendant is not allowed
  try {
    upgrader.connect(remote_ip);
  } catch (std::exception &e) {
    std::cout << "Failed to connect to UpdateManager: " << e.what() << std::endl;
    return -1;
  }

  // Run the upgrade example
//  upgradeController();

  // Run the backup export example
  exportControllerBackup();

  return 0;
}

std::string getCurrentDateTime() {
  // Get the current time point
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);

  // Convert to local time
  std::tm tm_buf;
#ifdef _WIN32
  localtime_s(&tm_buf, &t); // Windows
#else
  localtime_r(&t, &tm_buf); // Linux / Unix
#endif

  // Format the output
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y-%m-%d_%H%M%S");
  return oss.str();
}