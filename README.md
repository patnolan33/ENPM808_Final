# ENPM808X_Final
[![Build Status](https://travis-ci.org/patnolan33/ENPM808_Final.svg?branch=master)](https://travis-ci.org/patnolan33/ENPM808_Final)

## Table of Contents
- [Overview](#overview)
- [License](#license)
- [Prerequisites / Dependencies](#prerequisites-dependencies)
- [SIP Process](#sip_process)
- [Build Steps](#build-steps)

## <a name="overview"></a> Overview
The purpose of this repository is an implementation of the "area mapper" robot for ACME robotics (ENPM 808X Final Project). Specifically, this software implements ROS and the [Turtlebot](https://wiki.ros.org/Robots/TurtleBot) platform along with ROS nodes and services to illustrate an autonomous navigation/mapping capability. At a high level, the simulated robot is placed in an environment where it does not know the structure of the world. Its job is to move through the environment and avoid obstacles, all while mapping the environment via laser scans and providing an operator a visual feed of an onboard camera. The operator, via ROS services, can adjust the speed of the robot, the distance threshold that constitutes an impending collision, and also initiate a "take picture" command to capture an image of interest. In addition, an operator can initiate a "stop motion" command as well as a "resume motion" command to the robot. 

The main features of this ROS package include a decision engine based on laser scan data and a mapping capability from the same laser scan data. The laser scans determine how far away environmental features are from the vehicle, allowing the vehicle to create a 3D map of the environment all while avoiding collisions with said environment. In addition, as the robot moves through the environment, it will provide an RGB camera feed back to an operator. As the operator observes what the robot is doing, they can initiate a ROS service call to take an image and save it if what the robot is looking at is of interest. This robot is useful in many situations, namely:
- Disaster relief efforts: send in this robot to unknown areas for the safety of first responders
- Indoor navigation: map an area to later be used for indoor navigation purposes (i.e. Google maps indoors)
- New construction compliance: map a building and compare measurements with original plans to assess compliance

## <a name="license"></a> License
This project is under the [BSD License](./LICENSE)

## <a name="sip_process"></a> SIP Process
The SIP Process followed is detailed in a spreadsheet at the following link:
[SIP Process](https://docs.google.com/spreadsheets/d/1oXFDLQsNwkKpbx6czQWAZedOgvgVFRhxESJdv_tiWR0/edit?usp=sharing)

Sprint planning and other review notes are provided in the google doc at the following link:
[Sprint Planning](https://docs.google.com/document/d/1SpJCYgMqnaWGYB6uBX_Ns0fMZDnbV28vtPiZLobzIR4/edit?usp=sharing)

## <a name="prerequisites-dependencies"></a> Prerequisites / Dependencies
This package requires that [ROS](http://wiki.ros.org/indigo/Installation) is installed as well as [catkin](http://wiki.ros.org/catkin?distro=indigo#Installing_catkin). This was tested using ROS Indigo, however any subsequent versions of ROS should still work. This package also requires [Gazebo](http://gazebosim.org/) and [Turtlebot_Gazebo](http://wiki.ros.org/turtlebot_gazebo) to be installed. Gazebo should already be installed as part of the ROS distro, however Turtlebot_Gazebo may need to be installed. To do so, open a terminal and run the following command:
```
sudo apt-get install ros-indigo-turtlebot-gazebo ros-indigo-turtlebot-apps ros-indigo-turtlebot-rviz-launchers
```
This package also depends on the following ROS packages:
-roscpp
-geometry_msgs
-move_base_msgs
-sensor_msgs

## <a name="build-steps"></a> Build Steps
To use this package, a catkin workspace must be setup first. Assuming catkin has been installed, run the following steps in the directory of your choice (a common one is ~/catkin_ws)
```
$ cd <PATH_TO_YOUR_DIRECTORY>
$ mkdir -p catkin_ws/src
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
```
Your workspace should now be setup and you should be able to use ros commands like `roscd` and `rosls`. Note that if you cannot use these commands or can't find ROS packages, try running `source devel/setup.bash` again in your catkin workspace.

To build the enpm808_final ROS package in this repository, first clone the repository into the catkin `src` directory:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws/src
$ git clone https://github.com/patnolan33/ENPM808_Final
```
Now simply run catkin_make to build the ROS package.
```
$ cd ..
$ catkin_make
```
You should now see a enpm808_final directory in `catkin_ws/build`. 
