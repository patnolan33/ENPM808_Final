/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, patnolan33
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * @file motionControllerTest.cpp
 * @brief Unit tests for the MotionController class
 * @details This file is used to run all unit tests for the MotionController ROS node
 * @author Patrick Nolan (patnolan33)
 * @copyright MIT License.
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <motionController.hpp>

/**
 * @brief Test the ability of the motion controller to determine the correct action
 */
TEST(TestSuite, determine_action) {
  MotionController *motionController = new MotionController(1.0);
  geometry_msgs::Twist tmpMsg;
  tmpMsg.linear.x = 0.0;
  tmpMsg.linear.y = 0.0;
  tmpMsg.linear.z = 0.0;
  tmpMsg.angular.x = 0.0;
  tmpMsg.angular.y = 0.0;
  tmpMsg.angular.z = 0.0;

  geometry_msgs::Twist msg = motionController->getVehicleAction();
  bool isEqual = false;
  if(tmpMsg.linear.x == msg.linear.x && tmpMsg.linear.y == msg.linear.y && tmpMsg.linear.z == msg.linear.z &&
      tmpMsg.angular.x == msg.angular.x && tmpMsg.angular.y == msg.angular.y && tmpMsg.linear.z == msg.angular.z) {
    isEqual = true;
  }

  EXPECT_EQ(true, isEqual);
}

/**
 * @brief Test the ability to set a new forward speed
 */
TEST(TestSuite, set_speed) {
  MotionController *motionController = new MotionController(1.0);
  motionController->setForwardSpeed(2.0);

  EXPECT_EQ(2.0, motionController->getForwardSpeed());
}

/**
 * @brief Test the ability to return the current forward speed
 */
TEST(TestSuite, get_speed) {
  MotionController *motionController = new MotionController(1.0);

  EXPECT_EQ(1.0, motionController->getForwardSpeed());
}
