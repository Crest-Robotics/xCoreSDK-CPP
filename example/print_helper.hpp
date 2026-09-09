/**
 * @file print_helper.hpp
 * @brief Print API call results
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#ifndef LIBROKAEEXAMPLE_EXAMPLE_CPP_PRINT_HELPER_HPP_
#define LIBROKAEEXAMPLE_EXAMPLE_CPP_PRINT_HELPER_HPP_

#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <iterator>
#include "rokae/robot.h"
#include "rokae/data_types.h"

/**
 * @brief OperationState formatter
 */
inline std::ostream &operator<<(std::ostream &os, rokae::OperationState st) {
  using OP = rokae::OperationState;
  switch(st) {
    case OP::idle: os << "Idle"; break;
    case OP::jog: os << "Jog state"; break;
    case OP::rtControlling: os << "Real-time mode controlling"; break;
    case OP::drag: os << "Drag teaching enabled"; break;
    case OP::rlProgram: os << "RL project running"; break;
    case OP::demo: os << "Demo running"; break;
    case OP::dynamicIdentify: os << "Dynamics identification in progress"; break;
    case OP::frictionIdentify: os << "Friction identification in progress"; break;
    case OP::loadIdentify: os << "Load identification in progress"; break;
    case OP::moving: os << "Moving"; break;
    case OP::jogging: os << "Jogging"; break;
    case OP::unknown: default: os << "Unknown"; break;
  }
  return os;
}

inline std::ostream &operator<<(std::ostream &os, rokae::OperateMode mode) {
  switch(mode) {
    case rokae::OperateMode::automatic: os << "Automatic"; break;
    case rokae::OperateMode::manual: os << "Manual"; break;
    case rokae::OperateMode::unknown: default: os << "Unknown"; break;
  }
  return os;
}

inline std::ostream &operator<<(std::ostream &os, rokae::PowerState st) {
  switch(st) {
    case rokae::PowerState::on: os << "Powered on"; break;
    case rokae::PowerState::off: os << "Powered off"; break;
    case rokae::PowerState::estop: os << "Emergency stop pressed"; break;
    case rokae::PowerState::gstop: os << "Safety gate open"; break;
    case rokae::PowerState::unknown: default: os << "Unknown"; break;
  }
  return os;
}

/**
 * @brief std::array formatter
 */
template <class T, size_t S>
inline std::ostream &operator<<(std::ostream &os, const std::array<T,S> &arr) {
  os << "[ ";
  std::copy(arr.cbegin(), arr.cend() - 1, std::ostream_iterator<T>(os, ", "));
  std::copy(arr.cend() - 1, arr.cend(), std::ostream_iterator<T>(os));
  os << " ]";
  return os;
}

/**
 * @brief std::vecotr formatter
 */
template <class T>
inline std::ostream &operator<<(std::ostream &os, const std::vector<T> &arr) {
  os << "[ ";
  std::copy(arr.cbegin(), arr.cend() - 1, std::ostream_iterator<T>(os, ", "));
  std::copy(arr.cend() - 1, arr.cend(), std::ostream_iterator<T>(os));
  os << " ]";
  return os;
}

/**
 * @brief Info formatter
 */
inline std::ostream &operator<<(std::ostream &os, const rokae::Info &info) {
  os << "Controller version " << info.version << " | Model " << info.type << " | Axis count " << info.joint_num;
  return os;
}

/**
 * @brief Frame formatter
 */
inline std::ostream &operator<<(std::ostream &os, const rokae::Frame &frame) {
  os << "[ X: " << frame.trans[0] << " Y: " << frame.trans[1] << " Z: " << frame.trans[2] <<
     " A: " << frame.rpy[0] << " B: " << frame.rpy[1] << " C: " << frame.rpy[2] << " ]";
  return os;
}

/**
 * @brief CartesianPosition formatter
 */
inline std::ostream &operator<<(std::ostream &os, const rokae::CartesianPosition &cart) {
  os << "Pose - [ X: " << cart.trans[0] << " Y: " << cart.trans[1] << " Z: " << cart.trans[2] <<
     " Rx: " << cart.rpy[0] << " Ry: " << cart.rpy[1] << " Rz: " << cart.rpy[2] << " ]";
  os << "\nElbow angle - " << cart.elbow;
  if(!cart.confData.empty()) {
    os << "\nConf - [ ";
    for(const auto &d: cart.confData){
      os << d << " ";
    }
    os << "]";
  }
  return os;
}

/**
 * @brief Load formatter
 */
inline std::ostream &operator<<(std::ostream &os, const rokae::Load &load) {
  os << "Mass: " << load.mass << "kg, Center of gravity X: " << load.cog[0] << " Y: " << load.cog[1] << " Z: " << load.cog[2] <<
     ", Inertia ix: " << load.inertia[0] << " iy: " << load.inertia[1] << " iz: " << load.inertia[2];
  return os;
}

/**
 * @brief Toolset formatter
 */
inline std::ostream &operator<<(std::ostream &os, const rokae::Toolset &toolset) {
  os << "End-effector - " << toolset.end << "\nExternal - " << toolset.ref <<
     "\nLoad - " << toolset.load;
  return os;
}

/**
 * @brief std::error_code formatter
 */
inline std::ostream &operator<<(std::ostream &os, const std::error_code &ec){
  if(ec) {
    os << ec.message() <<"(" << ec.value() << ")";
  }
  return os;
}

/**
 * @brief out stream
 */
template <typename... Args>
void print(std::ostream &os, Args&&... args) {
  ((os << ' '<< std::forward<Args>(args)), ...) << std::endl;
}

#endif //LIBROKAEEXAMPLE_EXAMPLE_CPP_PRINT_HELPER_HPP_
