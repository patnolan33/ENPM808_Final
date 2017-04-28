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
 *  @file      vehicle.cpp
 *  @brief     Vehicle class implementation
 *  @details   Implementation of the ROS Vehicle support methods.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vehicle.hpp>
#include <enpm808_final/takeImageService.h>

/**
 * @brief Vehicle constructor
 */
Vehicle::Vehicle() {
	motionController = new MotionController(1.0);
	camera = new Camera();

	// Set up subscribers
	cameraSub = nh.subscribe < sensor_msgs::Image
			> ("/camera/rgb/image_raw", 500, &Camera::cameraCallback, camera);

	laserSub =
			nh.subscribe < sensor_msgs::LaserScan
					> ("/scan", 500, &MotionController::determineAction, motionController);

	// Register service with the master
	server = nh.advertiseService("takeImageService", &Camera::takeImage,
			camera);

	// Set up publisher:
	drivePub = nh.advertise < geometry_msgs::Twist
			> ("/mobile_base/commands/velocity", 1000);
}

/**
 * @brief drive the vehicle autonomously using laser scan data as sensor feedback
 */
void Vehicle::drive() {
	// Grab current vehicle action:
	geometry_msgs::Twist vehicleCommand = motionController->getVehicleAction();

	// Publish command:
	drivePub.publish(vehicleCommand);
}

