# ENPM808X_Final
[![Build Status](https://travis-ci.org/patnolan33/ENPM808_Final.svg?branch=master)](https://travis-ci.org/patnolan33/ENPM808_Final)

## Table of Contents
- [Overview](#overview)
- [License](#license)
- [Prerequisites / Dependencies](#prerequisites-dependencies)
- [SIP Process](#sip_process)
- [Build Steps](#build-steps)
- [Run Steps](#run-steps)
- [Testing](#testing)

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
This package also requires Octomap. If not installed, Octomap can be installed using the following command:
```
sudo apt-get install ros-indigo-octomap
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

## <a name="run-steps"></a> Run steps
To run the package, open a new terminal, change directories into your catkin workspace, and source your directory. Then, run the launch file using roslaunch:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ roslaunch enpm808_final enpm808_final.launch
```
You should see several windows open. You will see a terminal window open that runs the vehicle node, printing messages denoting laser scans are seen. This is also the window where we will see the vehicle node respond to any take image service calls (see [Take an image using rosservice](#take_image_service)). Another window that will appear is the Gazebo simulator window where a simulated turtlebot vehicle will be placed within a demo world (pictured below, left). The final window that will open is an image viewer window. This window shows the current RGB camera feed from onboard the turtlebot robot. As the vehicle moves around, you can watch this window to see if there are any interesting features in the world -- if so, take an image using a ROS service call. An example of this view is shown below on the right. 

![gazebo example](./results/gazeboExample.png?raw=true "Gazebo Example")

When the gazebo world is loaded, the turtlebot will start to drive forward. It will drive forward until it encounters an obstacle, at which point it will stop, turn in place until it sees no obstacle, and then continue to drive forward. This is maybe considered a "dumb" way to navigate, but in the desired use cases, the area may be completely unknown and the robot's task is to collect as much information as it can about its surrounding environment. 

## <a name="take_image_service"></a> Take an image using rosservice
When the vehicle is moving, you may see something that you wish to take a picture of in your image view window. To do so, you can issue a `rosservice` call to the vehicle. The vehicle will see this service and change the `takeImage` flag so that next time it sees the `/camera/rgb/image_raw` topic, it will take and save an image. To make this service call, open a new terminal, change directories to your workspace, source the directory, and call rosservice: 
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ rosservice call /takeImageService true
```

## <a name="testing"></a> Testing
Unit tests have been written for this repository. To run the tests, open a new terminal and change directories to your catkin workspace. Then, run the tests using catkin and the test launch file:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws/src
$ catkin_make run_tests && catkin_test_results
```
You should see a summary of the number of tests passed/failed output on the screen. 

## <a name="todo"></a> TODO
- Implement services:
  - "Stop motion" command
  - "Resume motion" command
- Output saved images and bag files to specified directory
