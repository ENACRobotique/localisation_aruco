cmake_minimum_required(VERSION 2.8.3)
PROJECT(cube_pos)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs sensor_msgs geometry_msgs message_generation genmsg cv_bridge image_transport )

find_package (Eigen3 REQUIRED NO_MODULE)


SET(ARUCO_PATH /opt/ros/kinetic)
SET(CMAKE_MODULE_PATH ${ARUCO_PATH}/lib/cmake)
SET(ARUCO_INCLUDE_DIRS ${ARUCO_PATH}/include/aruco)
## System dependencies are found with CMake's conventions

FIND_PACKAGE(OpenCV REQUIRED)

FIND_PACKAGE(aruco REQUIRED)

#Generating our Service
add_service_files(
	FILES
	CalibRequest.srv
)

## Generating our messages
add_message_files(
	FILES
	Robot.msg
	Robots.msg
)
generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

if( EXISTS /opt/vc/include/bcm_host.h)
	add_definitions(-DRASPI)
	set(USE_RASPI TRUE)
	find_package(WiringPi REQUIRED)
	include_directories(${WIRINGPI_INCLUDE_DIRS}) 
else()
	SET(WIRINGPI_LIBRARIES "")
	set(USE_RASPI FALSE)
endif()

INCLUDE(FindPkgConfig)

SET(ROS_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -march=native -O2 -fpermissive -Wall")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
	${ARUCO_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
    includes/
    includes/spectra_include/
	src/
)

################
## Executable ##
################

## Process marker (Bloc indépendant)
add_executable(${PROJECT_NAME}_Marker_Spectator
	src/BaseMarkerProcess.cpp
	src/RosSpectator.cpp
	src/Utils.cpp
	src/BaseMarkerProcess.h
	src/RosSpectator.h
	src/Utils.h
)
set_target_properties(${PROJECT_NAME}_Marker_Spectator PROPERTIES OUTPUT_NAME Marker_Spectator PREFIX "")

add_dependencies(${PROJECT_NAME}_Marker_Spectator ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_Marker_Spectator 
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
#fin Reporter

IF(USE_RASPI)
## UART_interface
	add_executable(${PROJECT_NAME}_UARTspectator
		src/UART_interface/main.cpp
	)
	set_target_properties(${PROJECT_NAME}_UARTspectator PROPERTIES OUTPUT_NAME UARTspectator PREFIX "")

	add_dependencies(${PROJECT_NAME}_UARTspectator ${catkin_EXPORTED_TARGETS})

	target_link_libraries(${PROJECT_NAME}_UARTspectator 
		${catkin_LIBRARIES}
		${WIRINGPI_LIBRARIES}
		m
		crypt
		rt
	)
	#fin UART_interface
endif()

add_definitions(-DDEBUG)

#############
## Install ##
#############

# TODO

#############
## Testing ##
#############

# TODO
