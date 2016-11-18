#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from turtlesim.srv import Kill, Spawn, SetPen
from turtlesim.msg import Pose
from rosgraph_msgs.msg import Log
from subprocess import Popen

def turtle_mommy(baby_name):

	global x
	global y
	global theta

	rospy.wait_for_service('spawn')
	try:
		new_robot = rospy.ServiceProxy('spawn',Spawn)
		resp = new_robot(5.5,5.5,0,baby_name)
	except rospy.ServiceException, e:
		print "%s"%e

	rospy.wait_for_service(baby_name+'/set_pen')
	try:
		# turn pen off
		changepen = rospy.ServiceProxy(baby_name+'/set_pen',SetPen)
		changepen(0,0,0,1,255)
	except rospy.ServiceException, e:
		print "%s"%e

def find_spheros():

	class SpheroConnections:

		def __init__(self):
			self.bt = 0
			rospy.Subscriber('rosout', Log, self.set_bt_state)

		def set_bt_state(self, log):
			""" Sets our Bluetooth variable based on the connection success. """
			if "Connect to Sphero with address" in log.msg:
				self.bt =  1
			elif "Failed to connect to Sphero." in log.msg:
				self.bt = -1

		def connect(self):
			""" Loops until a Sphero is connected. """
			i = 0
			while i < rospy.get_param("total_robot_n"):
				self.bt = 0
				# Give the Sphero a unique namespace.
				ns = 'sphero'+str(i+1)
				# Start a new sphero.py process to begin connecting.
				p = Popen(['rosrun', 'sphero_node', 'sphero.py', '__ns:=' + ns])
				# Spin our wheels while connecting.
				while not self.bt:
					if p.poll():
						break
				if not p.returncode:
					if self.bt == 1:
						i += 1
					elif self.bt == -1:
						continue
				else:
					break
			rospy.set_param("setup_complete",True)

	rospy.init_node('spheros_connection',anonymous=True)
	SpheroConnections().connect()
	rospy.spin()

if __name__ == '__main__':

	rospy.wait_for_service('kill')
	try:
		retire = rospy.ServiceProxy('kill',Kill)
		resp = retire('turtle1')
	except rospy.ServiceException, e:
		print "%s"%e

	if rospy.get_param("simulation")==True:
		#launch robots in turtlesim
		for baby_number in range(1,rospy.get_param("total_robot_n")+1):
			turtle_mommy('sphero'+str(baby_number))
		rospy.set_param("setup_complete",True)
	else:
		#connect to spheros
		find_spheros()
	
