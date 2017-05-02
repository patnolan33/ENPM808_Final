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
 *  @file      obstacleDetection.cpp
 *  @brief     ObstacleDetection class implementation
 *  @details   Implementation of the ROS ObstacleDetection support methods.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "obstacleDetection.hpp"

/**
 * @brief ObstacleDetection constructor
 */
ObstacleDetection::ObstacleDetection(double threshold) :
		distanceThreshold(threshold) {
	worldModel = new octomap::OcTree(0.1);
}

/**
 * @brief detect if the vehicle is about to collide with an obstacle or not
 */
bool ObstacleDetection::detectObstacle(
		const sensor_msgs::LaserScan msg) {

	// Check if any scan from the laser is less than 0.75 meters
	//  from the front of the robot. If so, a collision is about to occur
	for (int i = 0; i < msg.ranges.size(); ++i) {
		if (msg.ranges[i] < distanceThreshold) {
			return true;
		}
	}

	// Return false if we are collision free
	return false;
}
