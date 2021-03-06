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
 *  @file      vehicle.hpp
 *  @brief     Vehicle class definition
 *  @details   Definition of the ROS Vehicle support methods.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <motionController.hpp>
#include <camera.hpp>

/**
 * @brief Vehicle class handles the camera and motion controller interactions for turtlebot
 */
class Vehicle {
 public:
  /**
   * @brief Vehicle constructor
   */
  Vehicle();

  /**
   * @brief drive the vehicle autonomously using laser scan data as sensor feedback
   */
  void drive();

  int getPublishedMessagesCount() {
    return publishedMessagesCount;
  }
  ;

 private:
  /**
   * @brief container for the camera object
   */
  Camera *camera;

  /**
   * @brief container for the motion controller object
   */
  MotionController *motionController;

  /**
   * @brief container for a ROS publisher to publish vehicle motion commands
   */
  ros::Publisher drivePub;

  /**
   * @brief container for a ROS node handler
   */
  ros::NodeHandle nh;

  /**
   * @brief container for a ROS subscriber for camera topics
   */
  ros::Subscriber cameraSub;

  /**
   * @brief container for a ROS subscriber for laser scan topics
   */
  ros::Subscriber laserSub;

  /**
   * @brief Container for service server (for takeImage service)
   */
  ros::ServiceServer takeImageServer;

  /**
   * @brief Container for service server (for changeSpeed service)
   */
  ros::ServiceServer changeSpeedServer;

  /**
   * @brief Container for service server (for changeThreshold service)
   */
  ros::ServiceServer changeThresholdServer;

  /**
   * @brief Container for service server (for togglePause service)
   */
  ros::ServiceServer togglePauseServer;

  /**
   * @brief Container for a counter of how many messages have been published
   */
  int publishedMessagesCount;
};
