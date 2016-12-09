#!/usr/bin/env python

import rospy
import numpy as np
import time
import sys
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sphero_ball_real.srv import *

[x,y] = [9999999,9999999]
[x0,y0] = [0,1]
[x_init,y_init] = [9999999,9999999]

go_to_floor = False
dance_begun = False

mode = -1 # 0 = waiting to be asked, 1 = waiting to be asked, 2 = following leader to dancefloor, 3 = dancing with partner

# for leaders to ask followers to dance
def chatter_callback(data):
	global mode
	global go_to_floor
	if data.data == "bow":
		print "Thank you as well!"
		mode = 4
	else:
		print "That would be lovely, "+data.data+"!"
		go_to_floor = True

# sphero position callback
def odom_callback(data):
	global x, y
	global x0, y0
	global x_init, y_init

	if x_init == 9999999:
		x_init = data.pose.pose.position.x
		y_init = data.pose.pose.position.y
	x = (data.pose.pose.position.x - x_init) + x0
	y = (data.pose.pose.position.y - y_init) + y0
	#theta = data.pose.pose.orientation.w - theta_init + theta0

def add_to_mode_counter(i):

	rospy.set_param('mode_checker_'+str(i),1)

def wait_for_next_mode():

	global mode
	rsum = 0
	for i in range(1,rospy.get_param('total_robot_n')+1):
		rsum = rsum + rospy.get_param('mode_checker_'+str(i))

	while rsum < rospy.get_param('total_robot_n') and rospy.get_param('mode') <= mode :
		time.sleep(1)

	mode = mode + 1
	rospy.set_param('mode',mode)
	for i in range(1,rospy.get_param('total_robot_n')+1):
		rospy.set_param('mode_checker_'+str(i),0)

def go_to(robot_name,pose_x,pose_y):

	global x, y
	pub = rospy.Publisher(robot_name+'/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(750) #hz

	goal_pose = np.array([pose_x,pose_y])
	current_pose = np.array([x,y])

	displacement = goal_pose-current_pose

	while np.linalg.norm(displacement) > 0.02:
		current_pose = np.array([x,y])
		displacement = goal_pose-current_pose
		direction = displacement/np.linalg.norm(displacement)
		ang = Vector3(0,0,0)
		lin = Vector3(direction[0],direction[1],0)
		stumble = Twist(lin,ang)
		pub.publish(stumble)
		rate.sleep()

	ang = Vector3(0,0,0)
	lin = Vector3(0,0,0)
	stumble = Twist(lin,ang)
	pub.publish(stumble)
	rate.sleep()

def navigate_toward(goal,robot_name,leader_name):

	rospy.wait_for_service('social_force')

	try:
		social_force = rospy.ServiceProxy('social_force', SocialForce)
		resp2 = social_force(robot_name,goal[0],goal[1],leader_name)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

	net_force = np.array([resp2.x,resp2.y])

	r_dot = 40
	net_direction = net_force/np.linalg.norm(net_force)

	return r_dot*net_direction

def smiler(robot_name,smile,goal,leader_name):
	global x, y
	global mode
	(r_dot,theta_dot) = (0,0)
	if smile != 0:
		(r_dot,theta_dot) = navigate_toward(goal,robot_name,leader_name)
	return (r_dot,theta_dot)

#		th = np.mod(np.arctan2(direction[1],direction[0]),2*np.pi) - (5-smile)*np.pi/10
#		go_to(robot_name,x,y,th)

# main driving function for followers
def follow(robot_name,robot_number):

	global mode
	global x, y
	global x0, y0
	global smile, go_to_floor

	rate = rospy.Rate(750) # hz
	rospy.Subscriber(robot_name+'/chatter',String,chatter_callback)
	rospy.Subscriber(robot_name+'/odom',Odometry,odom_callback)

	pub_trans = rospy.Publisher(robot_name+'/transform', Vector3, queue_size=10)	

	while x == 9999999:
		time.sleep(.2)

	pub_trans.publish(Vector3(x_init,y_init,0))		

	print(robot_name+': ready to be asked to dance! I am at position: ('+str(x)+','+str(y)+").")
	pub_vel = rospy.Publisher(robot_name+'/cmd_vel', Twist, queue_size=10)
	add_to_mode_counter(int(robot_name.replace('sphero','')))
	wait_for_next_mode()
	#print(robot_name+": Time for the next mode: "+str(mode))

	while not rospy.is_shutdown():

		if mode == 0:
			# wait
			x_dot = 0
			y_dot = 0
			mode = 1

		elif mode == 1:
			# if a leader gets close, turn toward or away from them
			leader_name = 'sphero'+str(int(robot_name.replace('sphero',''))-1)
			rospy.wait_for_service('robot_locator')
			try:
				robot_locator = rospy.ServiceProxy('robot_locator', RobotLocator)
				resp1 = robot_locator(leader_name)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			their_pose = np.array([resp1.x,resp1.y])
			direction = their_pose-np.array([x,y])
			if np.linalg.norm(direction) < 1:
				print "my leader is close!"
				#(r_dot,theta_dot) = smiler(robot_name,smile,np.array([x,y])+np.sign(smile)*direction/np.linalg.norm(direction),leader_name)
				#lin = Vector3(r_dot,0,0)
				#ang = Vector3(0,0,theta_dot)
				#glide = Twist(lin,ang)
				#pub_vel.publish(glide)
				#time.sleep(.5)
				#lin = Vector3(0,0,0)
				#ang = Vector3(0,0,0)
				#glide = Twist(lin,ang)
				#pub_vel.publish(glide)
				#if smile > 0:
				#	go_to(robot_name,x,y)
				#	time.sleep(1.5)
				#elif smile < 0:
				#	go_to(robot_name,x,y)
				#	time.sleep(4.5)
				#else:
				#	time.sleep(3.5)
				if go_to_floor:
					mode = 2

		elif mode == 2:
			# follow leader to dance floor using force model
			try:
				robot_locator = rospy.ServiceProxy('robot_locator', RobotLocator)
				resp1 = robot_locator(leader_name)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e

			their_pose = np.array([resp1.x,resp1.y])
			goal = their_pose + np.array([0,-.25])

			if np.linalg.norm(goal-np.array([x,y])) < .02:
				lin = Vector3(0,0,0)
				ang = Vector3(0,0,0)
				glide = Twist(lin,ang)
				pub_vel.publish(glide)
				add_to_mode_counter(int(robot_name.replace('sphero','')))
				print robot_name + ": Okay! Waiting for everyone else to line up."
				wait_for_next_mode()
			else:
				[x_dot,y_dot] = navigate_toward(goal,robot_name,leader_name)

				if time.time() % 1 > .99:
					print "-----------------------------------------------"
					print "I am following my leader to the floor."
					print "They are at " + str(their_pose) + "."
					print "I'm going to " + str(goal) + "."
					print "My velocity is " + str ([x_dot,y_dot]) + "."

		elif mode == 3:

			global dance_begun
			# dance with partner and avoid others
			try:
				robot_locator = rospy.ServiceProxy('robot_locator', RobotLocator)
				resp1 = robot_locator(leader_name)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			their_pose = np.array([resp1.x,resp1.y])

			displacement = their_pose-np.array([x,y])

			# stay a certain distance in front of your partner
			goal = their_pose - .25*displacement/np.linalg.norm(displacement)
			if np.linalg.norm(goal-np.array([x,y])) < .02:
				(x_dot,y_dot) = (0,0)
				#if smile > 0 and dance_begun == False:
					#twist
			else:
				dance_begun = True
				[x_dot,y_dot] = navigate_toward(goal,robot_name,leader_name)

		else:
			goal = [x0,y0]
			if np.linalg.norm(goal-np.array([x,y])) < .02:
				(x_dot,y_dot) = (0,0)
				print "Yay! That was fun."
				#if smile > 0 and dance_begun == False:
					#twist
			else:
				[x_dot,y_dot] = navigate_toward(goal,robot_name,'')

		lin = Vector3(x_dot,y_dot,0)
		ang = Vector3(0,0,0)

		stumble = Twist(lin,ang)
		pub_vel.publish(stumble)
		rate.sleep()

if __name__ == '__main__':

	baby_number = int(rospy.myargv(argv=sys.argv)[1])
	x0 = (baby_number + 1)/2
	baby_name = 'sphero'+str(baby_number)
	rospy.init_node(baby_name+'_driver', anonymous=True)

	if baby_number <= rospy.get_param('total_robot_n'):

		# defines how happy a robot is about dancing. All but sphero2 will be neutral (0)
		# sphero2's value will be varied for our experment. -5 = very unhappy, 5 = very happy
		if baby_name == 'sphero2':
			smile = 5
		else:
			smile = 0

		while rospy.get_param("setup_complete") == False:
			time.sleep(1)

		try:
			follow(baby_name,baby_number)
			rospy.spin() # not sure why I put this here. Perhaps it should go inside follow()
		except rospy.ROSInterruptException:
			pass

		
