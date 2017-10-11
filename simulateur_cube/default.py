#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <club_robot> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
from club_robot.builder.robots import Cube

# Add the MORSE mascott, MORSY.
# Out-the-box available robots are listed here:
# http://www.openrobots.org/morse/doc/stable/components_library.html
#
# 'morse add robot <name> club_robot' can help you to build custom robots.
robot = Cube()

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/stable/user/builder_overview.html
robot.translate(.5,.5, 0.2)
robot.rotate(0.0, 0.0, 0)

# Add a motion controller
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
#
# 'morse add actuator <name> club_robot' can help you with the creation of a custom
# actuator.
motion = MotionVW()
robot.append(motion)


# Add a keyboard controller to move the robot with arrow keys.
keyboard = Keyboard()
keyboard.properties(ControlType = 'Position')
robot.append(keyboard)

# Add a pose sensor that exports the current location and orientation
# of the robot in the world frame
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#sensors
#
# 'morse add sensor <name> club_robot' can help you with the creation of a custom
# sensor.
pose = Pose()
robot.append(pose)

# creates a new instance of the sensor
robot2 = FakeRobot()
robot2.translate(0,0,0)

videocamera = VideoCamera()
videocamera.cam_width=640
videocamera.cam_height=480
robot2.append(videocamera)
# place your component at the correct location
videocamera.translate(0, 0,1)
videocamera.rotate(1,-.5,0)



# To ease development and debugging, we add a socket interface to our robot.
#
# Check here: http://www.openrobots.org/morse/doc/stable/user/integration.html 
# the other available interfaces (like ROS, YARP...)
robot.add_default_interface('socket')
#videocamera.add_interface('socket')
robot.add_default_interface('ros')
videocamera.add_interface('ros')


# set 'fastmode' to True to switch to wireframe mode
env = Environment('table.blend', fastmode = False)
env.set_camera_location([0, 0,1])
env.set_camera_rotation([.85, 0,-1])

