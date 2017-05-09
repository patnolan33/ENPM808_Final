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
 * @file vehicleTest.cpp
 * @brief Unit tests for the Vehicle class
 * @details This file is used to run all unit tests for the Vehicle ROS node
 * @author Patrick Nolan (patnolan33)
 * @copyright MIT License.
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <vehicle.hpp>

/**
 * @brief Test that verifies that the vehicle is publishing velocity commands
 */
TEST(TestSuite, drive_command_published) {
  // Handle for the process node. Will handle initialization and
  //   cleanup of the node
  ros::NodeHandle n;

  // Vehicle container
  Vehicle *vehicle = new Vehicle();

  // Check that we've published a message:
  EXPECT_EQ(0, vehicle->getPublishedMessagesCount());

  // While we are running, drive the vehicle autonomously
  vehicle->drive();

  // "Spin" a callback in case we set up any callbacks
  ros::spinOnce();

  // Check that we've published a message:
  EXPECT_EQ(1, vehicle->getPublishedMessagesCount());
}
