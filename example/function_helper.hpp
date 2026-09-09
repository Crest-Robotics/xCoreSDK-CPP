//
// Created by arcia on 2025/7/9.
//

// Note: Comments and console messages in this file were translated from Chinese to English by Claude Code.

#ifndef XCORESDK_EXAMPLE_RELEASE_FUNCTION_HELPER_HPP_
#define XCORESDK_EXAMPLE_RELEASE_FUNCTION_HELPER_HPP_

#include <thread>
#include "rokae/robot.h"

namespace rokae::helper {

 /**
  * @brief Wait for the robot to become idle
  */
 void waitRobot(rokae::BaseRobot &robot) {
   using namespace rokae;
   while (true) {
     std::this_thread::sleep_for(std::chrono::milliseconds(100));
     error_code ec;
     auto st = robot.operationState(ec);
     if (st == OperationState::idle || st == OperationState::unknown) {
       return;
     }
   }
 }

}

#endif //XCORESDK_EXAMPLE_RELEASE_FUNCTION_HELPER_HPP_
