#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

import cv2
import numpy as np

#Callback def
pos_v=[]
pos_c=[]
pos_v_p=[]

def conv_msg(msg):
	position = msg.pose.position
	return [position.x,position.y]

def callback_poseStamped_v(msg):
	if(pos_v!=[]):
		pos_v.pop()
	pos_v.append(conv_msg(msg))

def callback_poseStamped_c(msg):
	if(pos_v!=[]):
		pos_c.append(conv_msg(msg))
		pos_v_p.append(pos_v[0])

#Subscribe to topic
rospy.init_node("Plotter_simu")
rospy.Subscriber("/robot/pose", PoseStamped, callback_poseStamped_v)
rospy.Subscriber("/aruco/markerarray", PoseStamped, callback_poseStamped_c)

print("Processing ...")
rospy.spin() # this will block until you hit Ctrl+C
print("\nend time to plot")

#Conversion
position_v=[list() for i in range(len(pos_c[0]))]
position_c=[list() for i in range(len(pos_c[0]))]
for i in range(len(pos_c)):
	for j in range(len(pos_c[0])):
		position_v[j].append(pos_v_p[i][j])
		position_c[j].append(pos_c  [i][j])
	
import matplotlib.pyplot as plt
plt.figure(1)
plt.plot(position_c[0],'b-')
plt.plot(position_v[0],'r')

plt.figure(2)
plt.plot(position_c[1],'b-')
plt.plot(position_v[1],'r')


plt.figure(3)
plt.plot(position_c[0],position_c[1],'b--')
plt.plot(position_v[0],position_v[1],'r*')



plt.show()
cv2.destroyAllWindows()
