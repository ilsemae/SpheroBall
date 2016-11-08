#!/usr/bin/env python

import rospy
import numpy as np
import time
import sys
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from turtlesim.srv import Kill, Spawn, SetPen
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry


x = 0
y = 0
theta = 0

mode = 1 # 1 = waiting to be asked, 2 = following leader to dancefloor, 3 = dancing with partner

# turtle position callback
def pose_callback(data):
	global x
	global y
	global theta
	x = data.x
	y = data.y
	theta = float(np.mod(data.theta,2*np.pi)) # bring angle value to within +2*pi

# sphero position callback
def odom_callback(data):
	#print(data)
	p=data

# main driving function for followers (turtlesim and sphero)
def follow(robot_name):

	sim = rospy.get_param("simulation")
	print('ready to be asked to dance!')

	if sim == True:
		rospy.Subscriber(robot_name+'/pose',Pose,pose_callback)
	else:
		rospy.Subscriber(robot_name+'/odom',Odometry,odom_callback)
		pub_color = rospy.Publisher(robot_name+'/set_color', ColorRGBA, queue_size=10)
		color = ColorRGBA(100,0,150,0) # maybe the color can be the "smile" variable

	pub_vel = rospy.Publisher(robot_name+'/cmd_vel', Twist, queue_size=10)

	rospy.init_node(robot_name+'_driver', anonymous=True)
	rate = rospy.Rate(50) # hz

	if sim == True:
		r_dot = 1
		theta_dot = 1
		lin = Vector3(r_dot,0,0)
		ang = Vector3(0,0,theta_dot)
	else:
		pub_color.publish(color)
		x_dot = 1
		y_dot = 1
		lin = Vector3(x_dot,y_dot,0)
		ang = Vector3(0,0,0)

	stumble = Twist(lin,ang)
	pub_vel.publish(stumble)
	rate.sleep()

if __name__ == '__main__':

	baby_number = int(rospy.myargv(argv=sys.argv)[1])

	while rospy.get_param("setup_complete") == False:
		time.sleep(1)

	try:
		follow('baby_number')
		rospy.spin() # not sure why I put this here. Perhaps it should go inside follow()
	except rospy.ROSInterruptException:
		pass

	
