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
	src/
)

## Cube pos est l'exécutable qui prend des images en entrée et renvoient la pos des robot
add_executable(cube_pos
	src/ros_aruco.cpp 
	src/aruco_cube.cpp 
	src/aruco_cube.h
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
add_executable(calibration_aruco
	src/calibration_aruco.cpp 
	src/aruco_cube.cpp 
	src/aruco_cube.h
)

add_dependencies(calibration_aruco ${catkin_EXPORTED_TARGETS})

target_link_libraries(calibration_aruco 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${aruco_LIBS}
	yaml-cpp
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
