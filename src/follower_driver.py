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

mode = 0 # 0 = waiting to be asked, 1 = waiting to be asked, 2 = following leader to dancefloor, 3 = dancing with partner

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


# main driving function for followers (turtlesim and sphero)
def follow(robot_name):

	global mode

	rospy.init_node(robot_name+'_driver', anonymous=True)
	rate = rospy.Rate(350) # hz

	sim = rospy.get_param("simulation")

	if sim == True:
		rospy.Subscriber(robot_name+'/pose',Pose,pose_callback)
		p_x = 10
		p_y = np.random.randint(3,30)/3
		th = np.pi

		global x
		while x==0: # wait for published messages to start being read
			time.sleep(1)

		go_to(robot_name,p_x,p_y,th)
	else:
		rospy.Subscriber(robot_name+'/odom',Odometry,odom_callback)
		pub_color = rospy.Publisher(robot_name+'/set_color', ColorRGBA, queue_size=10)
		color = ColorRGBA(100,0,150,0) # maybe the color can be the "smile" variable

	print('ready to be asked to dance!')
	pub_vel = rospy.Publisher(robot_name+'/cmd_vel', Twist, queue_size=10)

	while not rospy.is_shutdown():

		if sim == True:
			if mode == 0:
				# wait to be asked
				r_dot = 0
				theta_dot = 0

			elif mode == 1:
				# wait to be asked
				r_dot = 0
				theta_dot = 0

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

			if mode == 0:
				# wait to go to a partner
				x_dot = 0
				y_dot = 0

			elif mode == 1:
				# wait to go to a partner
				x_dot = 0
				y_dot = 0

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

		stumble = Twist(lin,ang)
		pub_vel.publish(stumble)
		rate.sleep()

if __name__ == '__main__':

	baby_number = int(rospy.myargv(argv=sys.argv)[1])
	baby_name = 'sphero'+str(baby_number)

	while rospy.get_param("setup_complete") == False:
		time.sleep(1)

	try:
		follow(baby_name)
		rospy.spin() # not sure why I put this here. Perhaps it should go inside follow()
	except rospy.ROSInterruptException:
		pass

	
