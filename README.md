# ENPM808X_Final
[![Build Status](https://travis-ci.org/patnolan33/ENPM808_Final.svg?branch=master)](https://travis-ci.org/patnolan33/ENPM808_Final)

## Table of Contents
- [Overview](#overview)
- [License](#license)
- [Prerequisites / Dependencies](#prerequisites-dependencies)
- [SIP Process](#sip_process)
- [ENPM808X Presentation](#presentation)
- [Build Steps](#build-steps)
- [Run Steps](#run-steps)
  - [Gazebo](#roslaunch-gazebo)
  - [RViz](#roslaunch-rviz)
- [Interacting with the vehicle (rosservice)](#vehicle-interaction)
  - [Take an image](#take-image-service)
  - [Change forward speed](#change-speed-service)
  - [Change obstacle detection threshold](#change-threshold-service)
  - [Pause / Resume vehicle motion](#pause-motion-service)
- [Testing](#testing)
- [Doxygen](#doxygen)


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

## <a name="presentation"></a> ENPM 808X Presentation
The final presentation for the semester is a Google Slides presentation located at the following link: [Presentation slides](https://docs.google.com/presentation/d/1i7uek1mZ_PQ8fuppnxsgIAiFNSbUhjTydXQjnxpaI24/edit?usp=sharing)

## <a name="prerequisites-dependencies"></a> Prerequisites / Dependencies
This package requires that [ROS](http://wiki.ros.org/indigo/Installation) is installed as well as [catkin](http://wiki.ros.org/catkin?distro=indigo#Installing_catkin). This was tested using ROS Indigo, however any subsequent versions of ROS should still work. This package also requires [Gazebo](http://gazebosim.org/) and [Turtlebot_Gazebo](http://wiki.ros.org/turtlebot_gazebo) to be installed. Gazebo should already be installed as part of the ROS distro, however Turtlebot_Gazebo may need to be installed. To do so, open a terminal and run the following command:
```
sudo apt-get install ros-indigo-turtlebot-gazebo ros-indigo-turtlebot-apps ros-indigo-turtlebot-rviz-launchers
```

This package also depends on the following ROS packages:
- roscpp
- geometry_msgs
- move_base_msgs
- sensor_msgs
-	message_generation
- image_transport

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
### <a name="roslaunch-gazebo"></a> Running with Gazebo and image_view
To run the package with the Gazebo world rendered, open a new terminal, change directories into your catkin workspace, and source your directory. Then, run the launch file using roslaunch:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ roslaunch enpm808_final gazebo_imageview_demo.launch
```
You should see several windows open. You will see a terminal window open that runs the vehicle node which will print messages as certain situations are encountered (e.g. when an obstacle is encountered or the vehicle gets "stuck"). This is also the window where we will see the vehicle node respond to any take image service calls (see [Interacting with the vehicle](#vehicle-interaction)). Another window that will appear is the Gazebo simulator window where a simulated turtlebot vehicle will be placed within a demo world (pictured below, left). The final window that will open is an image viewer window. This window shows the current RGB camera feed from onboard the turtlebot robot. As the vehicle moves around, you can watch this window to see if there are any interesting features in the world -- if so, take an image using a ROS service call. An example of this view is shown below on the right. 

![gazebo example](./results/gazeboExample.png?raw=true "Gazebo Example")

When the gazebo world is loaded, the turtlebot will start to drive forward. It will drive forward until it encounters an obstacle, at which point it will stop, turn in place until it sees no obstacle, and then continue to drive forward. This is maybe considered a "dumb" way to navigate, but in the desired use cases, the area may be completely unknown and the robot's task is to collect as much information as it can about its surrounding environment. 

### <a name="roslaunch-rviz"></a> Running with RViz
To run the package with the Gazebo world rendered, open a new terminal, change directories into your catkin workspace, and source your directory. Then, run the launch file using roslaunch:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ roslaunch enpm808_final rviz_demo.launch
```
You should see a terminal window open that runs the vehicle node which will print messages as certain situations are encountered (e.g. when an obstacle is encountered or the vehicle gets "stuck"). This is also the window where we will see the vehicle node respond to any take image service calls (see [Interacting with the vehicle](#vehicle-interaction)). Another window that will appear is the RViz viewer window where a simulated turtlebot vehilce will be placed in an empty world. The configuration file that is loaded by default (located in `/rviz/` of this repository) loads the LaserScan, DepthCloud, and Image displays. In the bottom left-hand corner of the RViz window shown below, you can see the RGB camera feed from the vehicle. In the main window, as the vehicle drives around you will see multicolored lines appear denoting depth readings obtained from Turtlebot's onboard sensors. This is the DepthCloud data, where "hot" colors denote closer depth readings and "cold" colors denote a distance reading farther away. The LaserScan data is also shown, but it is hard to see in the image--it is the thin, red line in the purple/blue regions of the image shown. This is a single array of depth readings straight out from the robot instead of in 3D.

![gazebo example](./results/rviz_screenshot.png?raw=true "RViz Example")

When the RViz window is loaded, the turtlebot will start to drive forward. Similar to the Gazebo launch option above, it will drive forward until it encounters an obstacle, at which point it will stop, turn in place until it sees no obstacle, and then continue to drive forward. As it drives around, you will see the image feed update as well as the laser scan and depth data update in the view. 

## <a name="vehicle-interaction"></a> Interacting with the vehicle (rosservice)
### <a name="take-image-service"></a> Take an image
When the vehicle is moving, you may see something that you wish to take a picture of in your image view window. To do so, you can issue a `rosservice` call to the vehicle. The vehicle will see this service and change the `takeImage` flag so that next time it sees the `/camera/rgb/image_raw` topic, it will take and save an image. To make this service call, open a new terminal, change directories to your workspace, source the directory, and call rosservice: 
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ rosservice call /takeImageService true
```
Note that if the image window seems to freeze even though the turlebot is moving, this is likely due to a performance bottleneck in the VM. Simply closing the image_view window will cause a new one to open as soon as a new /camera topic is received, and this will "reset" the camera view. You should see the view changing according to what the turtlebot is currently seeing.

### <a name="change-speed-service"></a> Change forward speed
You may also want to change the speed at which the Turtlebot moves forward. To do so, you can issue another `rosservice` call to the vehicle. The vehicle will see this service and change the `forwardSpeed` parameter to the value passed in to the service. To make this service call, open a new terminal, change directories to your workspace, source the directory, and call rosservice: 
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ rosservice call /changeSpeedService 1.0
```
The default speed is 0.25 m/s. The above command will change this to 1.0 m/s, causing the vehicle to move faster in the positive x-direction. 

### <a name="change-threshold-service"></a> Change obstacle detection threshold
Another parameter that can be modified via `rosservice` is the distance threshold at which the vehicle will stop moving to avoid an obstacle. As the vehicle moves throughout the environment, it checks its laser scan data for any obstacles closer than this distance threshold. If one is seen, the vehicle stops and turns in place until it no longer sees an obstacle. To make this service call, open a new terminal, change directories to your workspace, source the directory, and call rosservice: 
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ rosservice call /changeThresholdService 0.5
```
The default threshold is 1 m. The above command will change this to 0.5 m, causing the vehicle to get closer to an obstacle before it decides to take evasive actions. 

### <a name="pause-motion-service"></a> Pause / Resume vehicle motion
Using `rosservice`, we can also tell the vehicle to stop in place (or subsequently resume its motion). To make this service call, open a new terminal, change directories to your workspace, source the directory, and call rosservice: 
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ rosservice call /togglePauseMotionService true
```
Passing in `true` will cause the vehicle to stop in place if it is not already stopped, while passing in `false` will make the vehicle resume motion if it is not already moving. Note that the sensor data will continue to stream, it is only the vehicle that will stop.

*NOTE: For some reason, every once in a while the vehicle would not stop when a "stop" command was issued. The topic published on `/mobile_base/commands/velocity` was 0.0 for all linear and angular velocity components, but the vehicle would continue to move. I found that to stop the vehicle more reliably, a very small linear or angular velocity component should be sent to the vehicle. In this implementation, when a "stop" command is issued, the topic published is 0.0 for all linear and angular velocity components except for the z-component of angular velocity. Instead, the z-component is set to 0.00000001, which seems to stop the robot in place reliably.*

## <a name="recording-rosbag"></a> Recording using rosbag
The package `rosbag` is a common ROS tool that is used to record and playback ROS topic messages. The launch file for this project accepts a boolean flag called `record` that toggles rosbag recording if included (true for record, false for do not record). To run the package and record the published topics, run:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ roslaunch enpm808_final gazebo_imageview_demo.launch record:=true
```
Alternatively, you can run the rviz_demo.launch file using `$ roslaunch enpm808_final rviz_demo.launch record:=true`

rosbag will save a file named `enpm808Final.bag` in the `~/.ros/` directory. To inspect it, simply change into the directory with `cd ~/.ros/` and run `rosbag info enpm808Final.bag`. This will output various information about the file, such as how long the file recorded, the start and end times, file size, the number of messages, and the topics recorded. 

*NOTE: These launch files will not record camera data, like RGB images and depth images, because the file size will become too large too quickly. If camera data is needed, rosbag will have to be run separately in another terminal.*

## <a name="playback-rosbag"></a> Playback using rosbag
We can playback this recorded data to recreate a recorded scenario. Assuming a rosbag recording has taken place according to the above process, playback the rosbag file by executing:
```
$ cd ~/.ros/
$ rosbag play enpm808Final.bag
```
In the rosbag terminal, you will see an indication that the rosbag file is running. This will playback the recorded vehicle motion and sensor data (minus the camera data), with which you can use however you like provided you subscribe to the topics being published.  

An example bag file is located in `/rosbag/` of this repository. To play this bag file, instead of changing directories to `~/.ros/`, simply navigate to `./rosbag` and run the `rosbag play enpm808Final.bag` command.

*NOTE: Gazebo should not be running when playing back with rosbag.*

## <a name="testing"></a> Testing
Unit tests have been written for this repository. To run the tests, open a new terminal and change directories to your catkin workspace. Then, run the tests using catkin and the test launch file:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws/src
$ catkin_make run_tests && catkin_test_results
```
You should see a summary of the number of tests passed/failed output on the screen. 

## <a name="doxygen"></a> Doxygen Documentation

If any changes are made to the source code, please document them and re-run the Doxygen documentation generation. To do so, simply change into the `docs/` directory and run:

```
$ cd docs/
$ doxygen Doxyfile
```

If Doxygen is not installed, install using

`sudo apt-get install doxygen`

## <a name="todo"></a> TODO
- Output saved images and bag files to specified directory
- Truly fix "stop" command instead of implementing the small-value workaround discussed
- Fix gmapping-turtlebot issues to create an occupancy grid in the Nootrix Ubuntu VM
