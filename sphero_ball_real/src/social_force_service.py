#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from sphero_ball_real.srv import SocialForce, SocialForceResponse

class SocialForceServer:

	def __init__(self):
		""" Instantiates the social force server. """
		rospy.init_node('social_force_server')
		rospy.Service('social_force', SocialForce, self.handle)
		# Total number of robots on the dance floor.
		self.n = rospy.get_param('total_robot_n')
		# Used to calculate force due to the other robots.
		self.robot_poses = {'sphero'+str(i+1): [-1,-1] for i in range(self.n)}
		self.robot_transforms = {'sphero'+str(i+1): [-1,-1] for i in range(self.n)}
		# Create a pose subscriber for each robot.
		for i in range(self.n):
			name = 'sphero'+str(i+1)
			rospy.Subscriber(name + '/odom', Odometry, self.pose_callback, name)
			rospy.Subscriber(name + '/transform', Vector3, self.transform_callback, name)
		# Print a ready message and then spin forever.
		print "Ready to calculate social forces."
		rospy.spin()

	def handle(self, req):
		""" Used to get the social force exerted on a robot. """
		args = [req.name, req.partner_name, req.goal_x, req.goal_y]
		return SocialForceResponse(*self.get_net_force(*args))

	def pose_callback(self, data, name):
		""" Used to set the pose for Sphero positions. """
		# initial position of robots in our reference frame
		x0 = (int(name.replace('sphero','')) + 1)/2
		y0 = 3*(int(name.replace('sphero','')) % 2) + 1  # 1 for even spheros, 4 for odd
		self.robot_poses[name][0] = (data.pose.pose.position.x - self.robot_transforms[name][0]) + x0
		self.robot_poses[name][1] = -(data.pose.pose.position.y - self.robot_transforms[name][1]) + y0

	def transform_callback(self, data, name):
		""" Used to set the pose for Sphero positions. """
		self.robot_transforms[name] = [data.x, data.y]


	def get_net_force(self, robot_name, partner_name, goal_x, goal_y):
		""" Computes the net social force exerted on a robot. """
		pos = np.array(self.robot_poses[robot_name])
		goal_coefficient = float((self.n-1)*25) # proportional to the total no. of robots
		net_force = np.array([0.0,0.0])
		# Factor the social force of each robot into the net force.
		for i in range(self.n):
			stranger = 'sphero'+str(i+1)
			# Skip over the robot itself and its partner.
			if stranger != robot_name and stranger != partner_name:
				# Wait for the stranger's pose to be updated for the first time.
				while self.robot_poses[stranger] == [-1,-1]: pass
				# Calculate and subtract the stranger's force from the net force. 
				stranger_pos = np.array(self.robot_poses[stranger])
				stranger_vec = stranger_pos - pos # vector to stranger robot
				stranger_dist = np.linalg.norm(stranger_vec)
				stranger_force = 3 * self.n/stranger_dist * stranger_vec/stranger_dist
				net_force -= stranger_force
				#print robot_name,": force from " + stranger + " equals " + str(stranger_force)
		# Add the social force of the goal.
		vect_to_goal = [goal_x, goal_y] - pos
		dist_to_goal = np.linalg.norm(vect_to_goal)
		goal_vect = 1/dist_to_goal * vect_to_goal/dist_to_goal
		net_force += goal_coefficient * goal_vect
		#print pos, vect_to_goal, goal_vect, net_force
		# Return the net force vector.
		return net_force

if __name__ == "__main__":
	SocialForceServer()


