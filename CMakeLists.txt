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
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -march=native -O2 -fpermissive")

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
add_executable(${PROJECT_NAME}_cube_pos
	src/ros_aruco.cpp 
	src/aruco_cube.cpp
    includes/aruco_cube.h
)
set_target_properties(${PROJECT_NAME}_cube_pos PROPERTIES OUTPUT_NAME cube_pos PREFIX "")

add_dependencies(${PROJECT_NAME}_cube_pos ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_cube_pos 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${aruco_LIBS}
	${WIRINGPI_LIBRARIES}
	yaml-cpp
)
## fin executable cube pos

## Calibration de la table à l'instalation des balises
add_executable(${PROJECT_NAME}_calibration_world
	src/calibration_world.cpp 
	src/aruco_cube.cpp
    includes/aruco_cube.h
)
set_target_properties(${PROJECT_NAME}_calibration_world PROPERTIES OUTPUT_NAME calibration_world PREFIX "")

add_dependencies(${PROJECT_NAME}_calibration_world ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_calibration_world 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${aruco_LIBS}
	${WIRINGPI_LIBRARIES}
	yaml-cpp
)

#fin calib

## Cube Pose To UART to send data through the air way up to the robots !


catkin_package(CATKIN_DEPENDS message_runtime)
add_executable(cube_pose_to_uart
        src/cube_pose_to_uart.cpp
        )
add_dependencies(cube_pose_to_uart ${catkin_EXPORTED_TARGETS})
target_link_libraries(cube_pose_to_uart rt pthread xbee
	        ${catkin_LIBRARIES}
        )

add_definitions(-DDEBUG)

#############
## Install ##
#############

# TODO

#############
## Testing ##
#############

# TODO
