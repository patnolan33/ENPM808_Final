cmake_minimum_required(VERSION 2.8.3)
project(enpm808_final)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	sensor_msgs
	cv_bridge
	roscpp
	geometry_msgs
	move_base_msgs
	genmsg
	message_generation
	image_transport
)

## Generate added messages and services
add_service_files(FILES takeImageService.srv)
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build vehicle 
find_package(octomap REQUIRED)
find_package( OpenCV REQUIRED COMPONENTS core imgproc highgui )
include_directories(include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(vehicle src/main.cpp src/vehicle.cpp src/camera.cpp src/motionController.cpp src/obstacleDetection.cpp)
target_link_libraries(vehicle ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBS})


## TESTING
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
  	enpm808_finalTest
	test/enpm808_finalTest.launch 
	test/main.cpp 
	test/vehicleTest.cpp 
	test/cameraTest.cpp 
	test/motionControllerTest.cpp 
	test/obstacleDetectionTest.cpp
	src/vehicle.cpp
	src/camera.cpp
	src/motionController.cpp
	src/obstacleDetection.cpp
  )
  target_link_libraries(
	enpm808_finalTest
	${catkin_LIBRARIES}
	${OCTOMAP_LIBRARIES}
	${OpenCV_LIBS}
  )
  add_dependencies(
	  enpm808_finalTest
	  ${catkin_EXPORTED_TARGETS}
  )
endif()