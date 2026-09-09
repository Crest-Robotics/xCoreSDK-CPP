//
// Created by arcia on 2025/7/9.
//
// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#ifndef XCORESDK_EXAMPLE_RELEASE_RT_RT_FUNTION_HELPER_HPP_
#define XCORESDK_EXAMPLE_RELEASE_RT_RT_FUNTION_HELPER_HPP_

#include "rokae/robot.h"
#include "rokae/utility.h"

namespace rokae::helper {
 /**
  * @brief Get real-time state data - Cartesian pose in real-time mode
  * @param robot instance
  * @return Cartesian pose [row-major]
  */
 template<WorkType Wt, unsigned short DoF>
 std::array<double, 16> getCurrentPose_matrix(rokae::Robot_T<Wt, DoF> &robot) {
   std::array<double, 16> pose{};
   try {
     // The queue receiving state data does not automatically overwrite old data; old data can be cleared by reading in a loop
     while (robot.updateRobotState(std::chrono::steady_clock::duration::zero()));
     if (robot.getStateData(RtSupportedFields::tcpPose_m, pose) == 0) {
       return pose;
     }
     throw std::runtime_error("getCurrentPose_matrix: getStateData failed");
   } catch (const std::exception &e) {
     std::cerr << e.what();
   }
   error_code ec;
   // No real-time state data received; retrieve it using the non-real-time interface
   Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), pose);
   return pose;
 }

 /**
  * @brief Get real-time state data - joint angles in real-time mode
  * @param robot instance
  * @return Joint angles [radians]
  */
 template<WorkType Wt, unsigned short DoF>
 std::array<double, DoF> getCurrentJointPos(rokae::Robot_T<Wt, DoF> &robot) {
   std::array<double, DoF> joint{};
   try {
     // The queue receiving state data does not automatically overwrite old data; old data can be cleared by reading in a loop
     while (robot.updateRobotState(std::chrono::steady_clock::duration::zero()));

     if (robot.getStateData(RtSupportedFields::jointPos_m, joint) == 0) {
       return joint;
     }
     throw std::runtime_error("getCurrentJointPos: getStateData failed");
   } catch (const std::exception &e) {
     std::cerr << e.what();
   }
   error_code ec;
   // No real-time state data received; retrieve it using the non-real-time interface
   return robot.jointPos(ec);
 }
}
#endif //XCORESDK_EXAMPLE_RELEASE_RT_RT_FUNTION_HELPER_HPP_
