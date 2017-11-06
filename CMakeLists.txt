cmake_minimum_required(VERSION 2.8.3)
PROJECT(cube_pos)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp tf std_msgs sensor_msgs geometry_msgs message_generation genmsg cv_bridge image_transport )


SET(ARUCO_PATH /opt/ros/kinetic)
SET(CMAKE_MODULE_PATH ${ARUCO_PATH}/lib/cmake)
SET(ARUCO_INCLUDE_DIRS ${ARUCO_PATH}/include/aruco)

## System dependencies are found with CMake's conventions

FIND_PACKAGE(OpenCV REQUIRED)

FIND_PACKAGE(aruco REQUIRED)

INCLUDE(FindPkgConfig)

SET(ROS_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -march=native -O2")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
	${ARUCO_INCLUDE_DIRS}
    includes/
	src/
)

## Generating our messages
#add_message_files(
#		FILES
#		Robot.msg
#        Robots.msg
#)
#
#generate_messages(
#        DEPENDENCIES
#        std_msgs
#        geometry_msgs
#)

## Cube pos est l'exécutable qui prend des images en entrée et renvoient la pos des robot
add_executable(cube_pos
	src/ros_aruco.cpp 
	src/aruco_cube.cpp
    includes/aruco_cube.h
)

add_dependencies(cube_pos ${catkin_EXPORTED_TARGETS})

target_link_libraries(cube_pos 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${aruco_LIBS}
	yaml-cpp
)
## fin executable cube pos

## Calibration de la table à l'instalation des balises
add_executable(calibration_world
	src/calibration_world.cpp 
	src/aruco_cube.cpp
    includes/aruco_cube.h
)

add_dependencies(calibration_world ${catkin_EXPORTED_TARGETS})

target_link_libraries(calibration_world 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${aruco_LIBS}
	yaml-cpp
)

## Cube Pose To UART to send data through the air way up to the robots !
#### NOTE : If it does not compile or run, try recompiling ArduPi Library by executing the following commands;
#### in localisation_aruco/arduPi : `g++ -fpermissive -c arduPi.cpp -o arduPi.o`



catkin_package(CATKIN_DEPENDS message_runtime)
add_executable(cube_pose_to_uart
        src/cube_pose_to_uart.cpp
        )

add_dependencies(cube_pose_to_uart ${catkin_EXPORTED_TARGETS})
include_directories(arduPi)
target_link_libraries(cube_pose_to_uart rt pthread ${CMAKE_CURRENT_SOURCE_DIR}/arduPi/arduPi.o
	        ${catkin_LIBRARIES}
        )

add_definitions(-DDEBUG)

#fin calib

#############
## Install ##
#############

# TODO

#############
## Testing ##
#############

# TODO
