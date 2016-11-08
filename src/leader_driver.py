#!/usr/bin/env python

import rospy
import numpy as np
import random
import sys
import time
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from turtlesim.srv import Spawn, SetPen
from turtlesim.msg import Color, Pose
from nav_msgs.msg import Odometry

x = 0
y = 0
theta = 0

mode = 0 # 0 = waiting to dance, 1 = navigating to a partner, 2 = leading partner to dance, 3 = dancing with partner

def pose_callback(data):
	global x
	global y
	global theta
	x = data.x
	y = data.y
	theta = float(np.mod(data.theta,2*np.pi)) # bring angle value to within +2*pi
	print(data)

def odom_callback(data):
	global x
	global y
	global theta
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	theta = float(np.mod(data.pose.pose.orientation.w,2*np.pi)) # bring angle value to within +2*pi

def driver(robot_name):

	global mode

	sim = rospy.get_param("simulation")
	print('ready to go find a dance partner!')

	if sim == True:
		rospy.Subscriber(robot_name+'/pose',Pose,pose_callback)
	else:
		rospy.Subscriber(robot_name+'/odom',Odometry,odom_callback)
		pub_color = rospy.Publisher(robot_name+'/set_color', ColorRGBA, queue_size=10)
		color = ColorRGBA(100,30,0,0)

	pub_vel = rospy.Publisher(robot_name+'/cmd_vel', Twist, queue_size=10)

	rospy.init_node(robot_name+'_driver', anonymous=True)
	rate = rospy.Rate(20) # hz

	while not rospy.is_shutdown():

		t = time.time()
		
		if sim == True:

			if mode == 0:
				# wait to go to a partner
				r_dot = 1
				theta_dot = 1

			elif mode == 1:
				# navigate to partner using force model
				r_dot = 1
				theta_dot = 1

			elif mode == 2:
				# navigate to dance floor using force model
				r_dot = 1
				theta_dot = 1

			else:
				# dance in a circle with partner and avoid others
				r_dot = 1
				theta_dot = 1

			lin = Vector3(r_dot,0,0)
			ang = Vector3(0,0,theta_dot)

		else:
			pub_color.publish(color)

			if mode == 0:
				# wait to go to a partner
				x_dot = 30*np.sin(3*t)
				y_dot = 30*np.cos(3*t)

			elif mode == 1:
				# navigate to partner using force model
				x_dot = 30*np.sin(3*t)
				y_dot = 30*np.cos(3*t)

			elif mode == 2:
				# navigate to dance floor using force model
				x_dot = 30*np.sin(3*t)
				y_dot = 30*np.cos(3*t)

			else:
				# dance in a circle with partner and avoid others
				x_dot = 30*np.sin(3*t)
				y_dot = 30*np.cos(3*t)

			lin = Vector3(x_dot,y_dot,0)
			ang = Vector3(0,0,0)

		glide = Twist(lin,ang)
		pub_vel.publish(glide)
		rate.sleep()

if __name__ == '__main__':

	baby_number = int(rospy.myargv(argv=sys.argv)[1])

	while rospy.get_param("setup_complete") == False:
		time.sleep(1)

	try:
		driver('sphero'+str(baby_number))
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


