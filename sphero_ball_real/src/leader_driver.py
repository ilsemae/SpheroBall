#!/usr/bin/env python

import rospy
import numpy as np
import random
import sys
import time
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sphero_ball_real.srv import *

[x,y] = [9999999,9999999]
[x0,y0] = [0,4]
[x_init,y_init] = [9999999,9999999]

t_0 = 0 # to hold starting time of dance
t_dance = 20 # length of dance in seconds
dance_state = -1
mode = -1 # 0 = waiting to dance, 1 = navigating to a partner, 2 = leading partner to dance, 3 = dancing with partner

# sphero position callback
def odom_callback(data):
	global x, y
	global x0, y0
	global x_init, y_init

	if x_init == 9999999:
		x_init = data.pose.pose.position.x
		y_init = data.pose.pose.position.y
	x = (data.pose.pose.position.x - x_init) + x0
	y = -(data.pose.pose.position.y - y_init) + y0

# used to begin and end the dance
def music_callback(data):

	global dance_state
	if data.data == 'begin':
		dance_state = 0
		print "Let the dance begin!"
	else:
		dance_state = 99999
		print "The dance has ended"

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

def navigate_toward(goal,robot_name,follower_name):

	global x,y

	rospy.wait_for_service('social_force')

	try:
		social_force = rospy.ServiceProxy('social_force', SocialForce)
		resp2 = social_force(robot_name,goal[0],goal[1],follower_name)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

	net_force = np.array([resp2.x,resp2.y])
	net_direction = net_force/np.linalg.norm(net_force)

	# slow down when getting close
	my_pose = np.array([x,y])
	if np.linalg.norm(my_pose-goal) < .5:
		r_dot = .1
	else:
		r_dot = .35

	return r_dot*net_direction


# main driving function for leader
def driver(robot_name,robot_number):

	global x, y
	global x0, y0
	global x_init, y_init

	global mode
	global dance_state
	global t_0, t_dance

	rate = rospy.Rate(750) # hz

	rospy.Subscriber('music', String, music_callback)
	rospy.Subscriber(robot_name+'/odom',Odometry,odom_callback)

	pub_vel = rospy.Publisher(robot_name+'/cmd_vel', Twist, queue_size=10)
	pub_trans = rospy.Publisher(robot_name+'/transform', Vector3, queue_size=10)

	while x == 9999999:
		time.sleep(1)
	
	pub_trans.publish(Vector3(x_init,y_init,0))		

	print(robot_name+': ready to go find a dance partner!  I am at position: ('+str(x)+','+str(y)+').')
	add_to_mode_counter(int(robot_name.replace('sphero','')))
	wait_for_next_mode()
	#print(robot_name+": Time for the next mode: "+str(mode))

	t0 = time.time()
	while not rospy.is_shutdown():

		my_pose = np.array([x,y])

		t = time.time() # current time in seconds
		
		if mode == 0:

			x_dot = 0
			y_dot = 0

			# stall until you build up the courage to ask someone to dance
			time.sleep(2)
			print(robot_name+": Okay, I'm feeling brave!")
			mode = 1

		# go ask a follower to dance
		elif mode == 1:

			follower_number=int(robot_name.replace('sphero',""))
			follower_name = 'sphero' + str(follower_number+1)
			pub_chat = rospy.Publisher(follower_name+'/chatter', String, queue_size=10)

			rospy.wait_for_service('robot_locator')
			try:
				robot_locator = rospy.ServiceProxy('robot_locator', RobotLocator)
				resp1 = robot_locator(follower_name)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e

			their_pose = np.array([resp1.x,resp1.y])
			goal = their_pose + np.array([0,.5])

			if np.linalg.norm(goal-np.array([x,y])) < .05:
				x_dot = 0
				y_dot = 0
				lin = Vector3(x_dot,y_dot,0)
				ang = Vector3(0,0,0)
				glide = Twist(lin,ang)
				pub_vel.publish(glide)
				rate.sleep()
				time.sleep(1)
				print(robot_name+": Would you like to dance, "+follower_name+"?")
				dance_proposal = String(robot_name)
				pub_chat.publish(dance_proposal)
				mode = mode+1
				#print(robot_name+": Time for the next mode: "+str(mode))
				rate = rospy.Rate(1) #hz
			else:
				# navigate to partner using force model
				[x_dot,y_dot] = navigate_toward(goal,robot_name,follower_name)
				if time.time() % 1 > .9:
					print "Gonna go get a partner! Me: " + str([round(x,2),round(y,2)]) + ". Them: " + str(their_pose) + ". Goal: " + str(goal) + ". My vel: " + str ([x_dot,y_dot]) + "."

		elif mode == 2:
			rate = rospy.Rate(750) #hz

			# navigate to dance floor using force model
			goal = np.array([float(robot_name.replace('sphero','')),2.5])

			if np.linalg.norm(goal-np.array([x,y]))<.05:
				add_to_mode_counter(int(robot_name.replace('sphero','')))
				print(robot_name+": Okay, ready to start the dance!")
				wait_for_next_mode()
				lin = Vector3(0,0,0)
				ang = Vector3(0,0,0)
				glide = Twist(lin,ang)
				pub_vel.publish(glide)
				t_0 = time.time()
			else:
				[x_dot,y_dot] = navigate_toward(goal,robot_name,follower_name)
				if time.time() % 1 > .9:
					print "I am leading my partner to the floor. I am: " + str(my_pose) + ". My goal: " + str(goal) + ". My vel: " + str ([round(x_dot,2),round(y_dot,2)]) + "./n"

		elif mode == 3:

			# dance with partner and avoid others
			if dance_state == -1:
				x_dot = 0
				y_dot = 0

			else:
				
				pointer = (their_pose - my_pose)/np.linalg.norm(their_pose-my_pose)
				rate = rospy.Rate(50) # hz

				if dance_state < 400:
					r = .3
					x_dot = r*np.cos(-2*(t-t0))
					y_dot = r*np.sin(-2*(t-t0))
					print "Spin! [", round(x_dot,2), ",", round(y_dot,2) ,"]"
					dance_end = String('cw')
					pub_chat.publish(dance_end)

				else:
					rate = rospy.Rate(750) # hz
					print(robot_name+": Thank you for a lovely dance, "+follower_name+".")
					dance_end = String('bow')
					pub_chat.publish(dance_end)
					x_dot = 0
					y_dot = 0
					lin = Vector3(x_dot,y_dot,0)
					ang = Vector3(0,0,0)

					glide = Twist(lin,ang)
					pub_vel.publish(glide)
					time.sleep(2)
					mode = 4

		else: 
			goal = [x0,y0]
			if np.linalg.norm(goal-np.array([x,y])) < .05:
				(x_dot,y_dot) = (0,0)
				print robot_name+": Yay! That was fun."
				return
				#if smile > 0 and dance_begun == False:
					#twist
			else:
				[x_dot,y_dot] = navigate_toward(goal,robot_name,'')


		lin = Vector3(x_dot,y_dot,0)
		ang = Vector3(0,0,0)

		glide = Twist(lin,ang)
		pub_vel.publish(glide)
		rate.sleep()

if __name__ == '__main__':

	baby_number = int(rospy.myargv(argv=sys.argv)[1])
	baby_name = 'sphero'+str(baby_number)
	rospy.init_node(baby_name+'_driver', anonymous=True)
	x0 = (baby_number+1)/2

	if baby_number <= rospy.get_param('total_robot_n'):

		while rospy.get_param("setup_complete") == False:
			time.sleep(1)

		try:
			driver('sphero'+str(baby_number),baby_number)
			rospy.spin() # same here. Does it matter if it's in driver() or not? - ilse
		except rospy.ROSInterruptException:
			pass


