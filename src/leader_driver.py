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
from sphero_ball.srv import *

x = 0
y = 0
theta = 0

mode = 1 # 0 = waiting to dance, 1 = navigating to a partner, 2 = leading partner to dance, 3 = dancing with partner

# turtle position callback
def pose_callback(data):
	global x
	global y
	global theta
	x = data.x
	y = data.y
	theta = float(np.mod(data.theta,2*np.pi)) # bring angle value to within +2*pi
	#print(data)

# sphero position callback
def odom_callback(data):
	global x
	global y
	global theta
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	theta = float(np.mod(data.pose.pose.orientation.w,2*np.pi)) # bring angle value to within +2*pi

def go_to(robot_name,pose_x,pose_y,angle):

	global x
	global y
	global theta

	pub = rospy.Publisher(robot_name+'/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(350) #hz

	goal_pose = np.array([pose_x,pose_y])
	current_pose = np.array([x,y])

	direction = goal_pose-current_pose
	goal_angle = np.mod(np.arctan2(direction[1],direction[0]),2*np.pi)
	i = np.sign(goal_angle-theta)
	if abs(goal_angle-theta) > np.pi:
		i = -i
	ang = Vector3(0,0,.5*i)
	lin = Vector3(0,0,0)
	while abs(goal_angle-theta)>.01:
		stumble = Twist(lin,ang)
		pub.publish(stumble)
		rate.sleep()
	ang = Vector3(0,0,0)
	lin = Vector3(3,0,0)
	while np.linalg.norm(goal_pose-current_pose)>.1:
		current_pose = np.array([x,y])
		stumble = Twist(lin,ang)
		pub.publish(stumble)
		rate.sleep()
	goal_angle = float(np.mod(angle,2*np.pi))
	i = np.sign(goal_angle-theta)
	if abs(goal_angle-theta) > np.pi:
		i = -i
	ang = Vector3(0,0,.5*i)
	lin = Vector3(0,0,0)
	while abs(theta-goal_angle)>.1:
		stumble = Twist(lin,ang)
		pub.publish(stumble)
		rate.sleep()
	ang = Vector3(0,0,0)
	lin = Vector3(0,0,0)
	stumble = Twist(lin,ang)
	pub.publish(stumble)
	rate.sleep()

# main driving function for leader
def driver(robot_name):

	global mode

	rospy.init_node(robot_name+'_driver', anonymous=True)
	rate = rospy.Rate(350) # hz

	sim = rospy.get_param("simulation")

	if sim == True:
		rospy.Subscriber(robot_name+'/pose',Pose,pose_callback)
		p_x = 1
		p_y = np.random.randint(3,30)/3
		th = 0

		global x
		while x==0: # wait for published messages to start being read
			time.sleep(1)

		go_to(robot_name,p_x,p_y,th)
	else:
		rospy.Subscriber(robot_name+'/odom',Odometry,odom_callback)
		pub_color = rospy.Publisher(robot_name+'/set_color', ColorRGBA, queue_size=10)
		color = ColorRGBA(100,30,0,0)

	print('ready to go find a dance partner!')
	pub_vel = rospy.Publisher(robot_name+'/cmd_vel', Twist, queue_size=10)


	while not rospy.is_shutdown():

		t = time.time() # current time in seconds
		
		if sim == True:

			if mode == 0:
				# stall until you build up the courage to ask someone to dance
				r_dot = 0
				theta_dot = 0

			elif mode == 1:

				rospy.wait_for_service('robot_locator')
				try:
					robot_locator = rospy.ServiceProxy('robot_locator', RobotLocator)
					resp1 = robot_locator('sphero2')
				except rospy.ServiceException, e:
					print "Service call failed: %s"%e

				their_pose = np.array([resp1.x,resp1.y])
				goal = their_pose + np.array([np.cos(resp1.theta),np.sin(resp1.theta)])

				# navigate to partner using force model
				rospy.wait_for_service('social_force')
				try:
					social_force = rospy.ServiceProxy('social_force', SocialForce)
					resp2 = social_force(robot_name,goal[0],goal[1])
				except rospy.ServiceException, e:
					print "Service call failed: %s"%e

				net_force = np.array([resp2.x,resp2.y])
				force_angle = np.arctan2(net_force[1],net_force[0])
				#print(turtle_name+": Ok! My force angle is "+str(force_angle))

				force_angle = np.mod(force_angle,2*np.pi) # bring angle value to within +2*pi
				angle_difference = force_angle-theta
				i   = np.sign(angle_difference)
				if abs(angle_difference) > np.pi:
					i = -i
				# if current turtle angle is close to goal angle, turn and move forward at the same time
				if abs(angle_difference) < np.pi/2 or abs(angle_difference)>3*np.pi/2:
					r_dot = 1
					theta_dot = i/2
				# otherwise, spin first, then move forward
				else:
					r_dot = 0
					theta_dot = 5*i
					lin = Vector3(r_dot,0,0)
					ang = Vector3(0,0,theta_dot)
					while abs(angle_difference)>.05:
						#print(turtle_name+": Ok! My angle error is "+str(angle_difference)+", so I am spinning!")
						angle_difference = force_angle-theta
						stumble = Twist(lin,ang)
						pub_vel.publish(stumble)
					r_dot = 1
					theta_dot = 0

				if np.linalg.norm(goal-np.array([x,y]))<.3:
					print(robot_name+": Would you like to dance, sphero 2?")
					lin = Vector3(0,0,0)
					ang = Vector3(0,0,0)
					mode = 2

			elif mode == 2:
				# navigate to dance floor using force model
				r_dot = 0
				theta_dot = 0

			else:
				# dance in a circle with partner and avoid others
				r_dot = 0
				theta_dot = 0

			lin = Vector3(r_dot,0,0)
			ang = Vector3(0,0,theta_dot)

		else:

			if mode == 0:
				# wait to go to a partner
				x_dot = 0
				y_dot = 0

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
		if mode == 0:
			# randomly decide when this leader will go find a partner
			a = np.random.randint(0,9)
			while a != 7:
				time.sleep(2)
			mode = 1
			print("Ok! Time to go find a partner!")
		rate.sleep()

if __name__ == '__main__':

	baby_number = int(rospy.myargv(argv=sys.argv)[1])

	while rospy.get_param("setup_complete") == False:
		time.sleep(1)

	try:
		driver('sphero'+str(baby_number))
		rospy.spin() # same here. Does it matter if it's in driver() or not? - ilse
	except rospy.ROSInterruptException:
		pass


