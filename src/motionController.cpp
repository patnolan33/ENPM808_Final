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
 *  @file      motionController.cpp
 *  @brief     MotionController class implementation
 *  @details   Implementation of the ROS MotionController support methods.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <motionController.hpp>

/**
 * @brief MotionController constructor
 */
MotionController::MotionController(double forwardSpeed) :
		forwardSpeed(forwardSpeed), pauseMotion(false), obstaclePresent(false), obstacleCounter(
				0) {
	obstacleDetection = new ObstacleDetection(1.0);

	// Initialize vehicle action to stay still:
	vehicleAction.linear.x = 0.0;
	vehicleAction.linear.y = 0.0;
	vehicleAction.linear.z = 0.0;
	vehicleAction.angular.x = 0.0;
	vehicleAction.angular.y = 0.0;
	vehicleAction.angular.z = 0.0;
}

/**
 * @brief Determine a vehicle action based on results from the obstacle detector
 */
void MotionController::determineAction(
		const sensor_msgs::LaserScan::ConstPtr& msg) {
	geometry_msgs::Twist action;
	action.linear.x = 0.0;
	action.linear.y = 0.0;
	action.linear.z = 0.0;
	action.angular.x = 0.0;
	action.angular.y = 0.0;
	action.angular.z = 0.00000001; // Note: The "stop motion" command wouldn't work
								   //  with a zero velocity, so a very small angular
								   // velocity is set to appear to stop in place

	// If we are not pausing the motion, set either forward speed or angular speed
	if (pauseMotion == false) {
		if (obstacleDetection->detectObstacle(*msg)) {
			// If this is the first time we've entered a collision state, warn the operator
			if (obstaclePresent == false) {
				ROS_WARN(
						"Obstacle detected <%f m away. Stop and turn until we are free.",
						obstacleDetection->getDistanceThreshold());
			}

			// Set linear velocity to zero
			action.linear.x = 0.0;
			// Set turn rate about the z-axis
			action.angular.z = 1.0;

			// Set flag for obstacle present:
			obstaclePresent = true;

			// Increment the counter
			obstacleCounter++;
			// If we've been searching for a while, warn the operator that we may be stuck
			if (obstacleCounter > 1000) {
				ROS_WARN_STREAM(
						"Vehicle may be stuck. Manual intervention may be required to continue.");
			}
		} else {
			// Set turn rate to zero
			action.angular.z = 0.0;
			// Move forward slowly
			action.linear.x = forwardSpeed;

			// Set flag for obstacle present:
			obstaclePresent = false;
		}
	}

	// Set vehicle action:
	vehicleAction = action;
}

/**
 * @brief Response to the change speed service to set forward speed
 */
bool MotionController::changeSpeed(
		enpm808_final::changeSpeedService::Request &req,
		enpm808_final::changeSpeedService::Response &resp) {
	// Set forward speed to desired speed:
	setForwardSpeed(req.speed);
	resp.resp = true;

	ROS_INFO("Set forward speed to: %f", req.speed);

	return resp.resp;
}

/**
 * @brief Response to the change threshold service to set distance threshold
 */
bool MotionController::changeThreshold(
		enpm808_final::changeThresholdService::Request &req,
		enpm808_final::changeThresholdService::Response &resp) {
	// Set distance threshold to desired threshold:
	obstacleDetection->setDistanceThreshold(req.threshold);
	resp.resp = true;

	ROS_INFO("Set detection threshold: %f", req.threshold);

	return resp.resp;
}

/**
 * @brief Response to the toggle pause motion service
 */
bool MotionController::togglePause(
		enpm808_final::togglePauseMotion::Request &req,
		enpm808_final::togglePauseMotion::Response &resp) {
	// Toggle pause motion flag:
	pauseMotion = req.pause;
	resp.resp = true;

	if (pauseMotion) {
		ROS_INFO_STREAM("Pause vehicle motion.");
	} else {
		ROS_INFO_STREAM("Continue vehicle motion.");
	}

	return resp.resp;
}
