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

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

if( EXISTS /opt/vc/include/bcm_host.h)
	add_definitions(-DRASPI)
	find_package(WiringPi REQUIRED)
	include_directories(${WIRINGPI_INCLUDE_DIRS}) 
else()
	SET(WIRINGPI_LIBRARIES "")
endif()

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


################
## Executable ##
################

## Process marker (Bloc indépendant)
add_executable(${PROJECT_NAME}_marker_detection
	src/aruco_finder.cpp
	src/Utils.cpp
	src/aruco_finder.h
	src/Utils.h
)
set_target_properties(${PROJECT_NAME}_marker_detection PROPERTIES OUTPUT_NAME marker_detection PREFIX "")

add_dependencies(${PROJECT_NAME}_marker_detection ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_marker_detection 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${aruco_LIBS}
	${WIRINGPI_LIBRARIES}
	yaml-cpp
)
#fin Process marker

## Reporter (Bloc indépendant)
add_executable(${PROJECT_NAME}_reporter
	src/Reporter.cpp
	src/Utils.cpp
	src/Reporter.h
	src/Utils.h
)
set_target_properties(${PROJECT_NAME}_reporter PROPERTIES OUTPUT_NAME reporter PREFIX "")

add_dependencies(${PROJECT_NAME}_reporter ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_reporter 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${aruco_LIBS}
	${WIRINGPI_LIBRARIES}
	yaml-cpp
)
#fin Process marker

add_definitions(-DDEBUG)

#############
## Install ##
#############

# TODO

#############
## Testing ##
#############

# TODO
