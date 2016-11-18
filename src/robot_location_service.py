#!/usr/bin/env python

import rospy
import time
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from sphero_ball.srv import *

turtle_poses = dict({"sphero1":[0,0,0],
	                 "sphero2":[0,0,0],
	                 "sphero3":[0,0,0],
	                 "sphero4":[0,0,0],
	                 "sphero5":[0,0,0],
	                 "sphero6":[0,0,0],
	                 "sphero7":[0,0,0],
	                 "sphero8":[0,0,0]})

# used to set global variable for the pose of all turtles' positions
def pose_callback(data):

	global turtle_poses

	topic_name=data._connection_header['topic']
	turtle_name=topic_name.replace('/',"").replace("pose","")
	turtle_poses[turtle_name][0] = data.x
	turtle_poses[turtle_name][1] = data.y
	turtle_poses[turtle_name][2] = data.theta


# sphero position callback
def odom_callback(data):

	global turtle_poses

	topic_name=data._connection_header['topic']
	turtle_name=topic_name.replace('/',"").replace("odom","")
	turtle_poses[turtle_name][0] = data.pose.pose.position.x
	turtle_poses[turtle_name][1] = data.pose.pose.position.y


def handle(req):

	global turtle_poses

	#print(req.name)
	while turtle_poses[req.name]==[0,0,0]:
		time.sleep(.1) # wait for turtle's pos to be updated for the first time
		#if turtle_poses[req.name]==[0,0]: # if it still hasn't updated, that's because there is no turtle with that name (lazy programming on my part)
		#	print("No such robot found") # skip this iteration

	return RobotLocatorResponse(turtle_poses[req.name][0],turtle_poses[req.name][1],turtle_poses[req.name][2])


def robot_location_server():

	rospy.init_node('robot_location_server')
	s = rospy.Service('robot_locator', RobotLocator, handle)

	global turtle_poses

	sim = rospy.get_param("simulation")
	if sim == True:
		for stranger in range(1,rospy.get_param('total_robot_n')+1):
			rospy.Subscriber('sphero'+str(stranger)+'/pose',Pose,pose_callback)
	else:
		for stranger in range(1,rospy.get_param('total_robot_n')+1):
			rospy.Subscriber('sphero'+str(stranger)++'/odom',Odometry,odom_callback)

	print "Ready to locate robots."
	rospy.spin()

if __name__ == "__main__":
	robot_location_server()