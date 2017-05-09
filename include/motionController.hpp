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
 *  @file      motionController.hpp
 *  @brief     MotionController class definition
 *  @details   Definition of the ROS MotionController support methods.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <obstacleDetection.hpp>
#include <enpm808_final/changeSpeedService.h>
#include <enpm808_final/togglePauseMotion.h>

/**
 * @brief MotionController class handles determining vehicle control actions
 */
class MotionController {
 public:
  /**
   * @brief MotionController constructor
   */
  MotionController(double forwardSpeed);

  /**
   * @brief Determine a vehicle action based on results from the obstacle detector
   */
  void determineAction(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief Response to the change speed service to set forward speed
   */
  bool changeSpeed(enpm808_final::changeSpeedService::Request &req,
                   enpm808_final::changeSpeedService::Response &resp);

  /**
   * @brief Response to the change threshold service to set distance threshold
   */
  bool changeThreshold(enpm808_final::changeThresholdService::Request &req,
                       enpm808_final::changeThresholdService::Response &resp);

  /**
   * @brief Response to the toggle pause motion service
   */
  bool togglePause(enpm808_final::togglePauseMotion::Request &req,
                   enpm808_final::togglePauseMotion::Response &resp);

  /**
   * @brief change the forward speed of the robot
   */
  void setForwardSpeed(double speed) {
    forwardSpeed = speed;
  }
  ;

  /**
   * @brief return the current forward speed of the robot
   */
  double getForwardSpeed() {
    return forwardSpeed;
  }
  ;

  /**
   * @brief return the current vehicle action
   */
  geometry_msgs::Twist getVehicleAction() {
    return vehicleAction;
  }
  ;

 private:
  /**
   * @brief container for an obstacle detector for the vehicle
   */
  ObstacleDetection *obstacleDetection;

  /**
   * @brief container for the forward speed of the vehicle
   */
  double forwardSpeed;

  /**
   * @brief Container for Twist message to be sent the vehicle on next "drive" command
   */
  geometry_msgs::Twist vehicleAction;

  /**
   * @brief Flag to denote if we should pause the robot or continue motion
   */
  bool pauseMotion;

  /**
   * @brief Flag to denote that we have entered a "collision" state and are turning
   */
  bool obstaclePresent;

  /**
   * @brief Counter for how long we have been spinning trying to find a free space
   */
  int obstacleCounter;
};
