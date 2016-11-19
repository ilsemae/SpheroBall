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

mode = -1 # 0 = waiting to be asked, 1 = waiting to be asked, 2 = following leader to dancefloor, 3 = dancing with partner

# for leaders to ask followers to dance
def chatter_callback(data):
	global mode
	print "Sure, I'll dance with you, "+data.data+"!"
	mode = 2

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

def add_to_mode_counter(i):

	rospy.set_param('mode_checker_'+str(i),1)

def wait_for_next_mode():

	global mode
	rsum = 0
	for i in range(1,rospy.get_param('total_robot_n')+1):
		rsum = rsum + rospy.get_param('mode_checker_'+str(i))

	while rsum < rospy.get_param('total_robot_n') and rospy.get_param('mode') == mode :
		time.sleep(1)

	mode = mode + 1
	rospy.set_param('mode',mode)
	for i in range(1,rospy.get_param('total_robot_n')+1):
		rospy.set_param('mode_checker_'+str(i),0)

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
	rospy.Subscriber(robot_name+'/chatter',String,chatter_callback)

	sim = rospy.get_param("simulation")

	if sim == True:
		rospy.Subscriber(robot_name+'/pose',Pose,pose_callback)
		p_x = 10
		p_y = float(np.random.randint(3,30))/3
		th = np.pi

		global x
		while x==0: # wait for published messages to start being read
			time.sleep(1)

		go_to(robot_name,p_x,p_y,th)
	else:
		rospy.Subscriber(robot_name+'/odom',Odometry,odom_callback)
		pub_color = rospy.Publisher(robot_name+'/set_color', ColorRGBA, queue_size=10)
		color = ColorRGBA(100,0,150,0) # maybe the color can be the "smile" variable

	print(robot_name+': ready to be asked to dance!')
	pub_vel = rospy.Publisher(robot_name+'/cmd_vel', Twist, queue_size=10)
	add_to_mode_counter(int(robot_name.replace('sphero','')))
	wait_for_next_mode()
	print(robot_name+": Time for the next mode: "+str(mode))

	while not rospy.is_shutdown():

		if sim == True:

			if mode == 0:
				# wait
				r_dot = 0
				theta_dot = 0

			elif mode == 1:
				# move forward or back depending on how happy you are to dance
				r_dot = 0
				theta_dot = 0

			elif mode == 2:
				# navigate to dance floor using force model
				# print(robot_name+": You lead the way!")
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

	
