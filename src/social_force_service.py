#!/usr/bin/env python

import rospy
import numpy as np
import random
import sys
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute
from turtlesim.msg import Color, Pose
from sphero_ball.srv import *

# used to calculate force due to the other turtles
turtle_poses = dict({"sphero1":[0,0],
	                 "sphero2":[0,0],
	                 "sphero3":[0,0],
	                 "sphero4":[0,0],
	                 "sphero5":[0,0],
	                 "sphero6":[0,0],
	                 "sphero7":[0,0],
	                 "sphero8":[0,0]})

# used to set global variable for the pose of all turtles' positions
def t_pose_callback(data):
	topic_name=data._connection_header['topic']
	turtle_name=topic_name.replace('/',"").replace("pose","")
	global turtle_poses
	turtle_poses[turtle_name][0] = data.x
	turtle_poses[turtle_name][1] = data.y

def get_net_force(turtle_name,goal):

	net_force = np.array([0,0])
	goal_coefficient = 10
	x = turtle_poses[turtle_name][0]
	y = turtle_poses[turtle_name][1]

	# add the social force from each stranger turtle
	for i in range(1,rospy.get_param('total_robot_n')):

		stranger = 'sphero'+str(i)
		global turtle_poses

		if stranger!=turtle_name and stranger!='sphero2':	# There is no force on that turtle from itself and sphero 2 is not a repelling force
			if turtle_poses[stranger]==[0,0]:
				time.sleep(.1) # wait for turtle's pos to be updated for the first time
				if turtle_poses[stranger]==[0,0]: # if it still hasn't updated, that's because there is no turtle with that name (lazy programming on my part)
					continue # skip this iteration
			# if this turtle is a leader, avoid it
			goal_coefficient = goal_coefficient + 30  # for each leader turtle, the weight of the goal must increase to keep it proportionally important enough
			stranger_pos = np.array(turtle_poses[stranger])
			stranger_pointer = stranger_pos-np.array([x,y]) # vector from current turtle to stranger turtle
			stranger_dist = np.linalg.norm(stranger_pointer)
			if stranger_pointer[1] == 0:
				stranger_pointer[1] = .01
			stranger_force = np.array([0,1/stranger_pointer[1]])
			net_force = net_force-stranger_force
			#print(turtle_name+": due to "+stranger+": "+str(-stranger_force))
			if stranger_dist<.5:
				print(turtle_name+": Oh dear! Pardon me, "+stranger+"! So sorry to have bumped you!")

	# add the social force of the goal
	vect_to_goal = goal - np.array([x,y])
	dist_to_goal = np.linalg.norm(vect_to_goal)

	goal_vect = 1/dist_to_goal*vect_to_goal/dist_to_goal
	net_force = net_force + goal_coefficient*goal_vect

	return net_force


def handle(req):
    force = get_net_force(req.name,[req.goal_x,req.goal_y])
    return SocialForceResponse(force[0],force[1])

def social_force_server():
	rospy.init_node('social_force_server')
	s = rospy.Service('social_force', SocialForce, handle)

	global turtle_poses
	for stranger in range(1,rospy.get_param('total_robot_n')+1):
		rospy.Subscriber('sphero'+str(stranger)+'/pose',Pose,t_pose_callback)

	print "Ready to calculate social forces."
	rospy.spin()

if __name__ == "__main__":
    social_force_server()