#!/usr/bin/env python

import rospy
import time
import numpy as np
from turtlesim.msg import Pose
from sphero_ball.srv import *

class SocialForceServer:

  def __init__(self):
    self.n = rospy.get_param('total_robot_n')
    # Used to calculate force due to the other robots.
    self.robot_poses = {"sphero"+str(i+1): [-1,-1] for i in range(self.n)}

    rospy.init_node('social_force_server')
    rospy.Service('social_force', SocialForce, self.handle)

    for i in range(self.n):
      name = 'sphero'+str(i+1)
      rospy.Subscriber(name + '/pose', Pose, self.t_pose_callback, name)

    print "Ready to calculate social forces."
    rospy.spin()

  def handle(self, req):
    force = self.get_net_force(req.name, req.partner_name, req.goal_x, req.goal_y)
    return SocialForceResponse(force[0],force[1])

  def t_pose_callback(self, data, name):
    """ Used to set the pose for Sphero positions. """
    self.robot_poses[name] = [data.x, data.y]

  def get_net_force(self, robot_name, partner_name, goal_x, goal_y):
    x,y = self.robot_poses[robot_name]
    net_force = np.array([0,0])
    goal_coefficient = 10

    # Add the social force from each stranger robot
    for i in range(self.n):
      stranger = 'sphero'+str(i+1)
      # There is no force on the robot from itself
      # and the goal robot is not a repelling force.
      if stranger != robot_name and stranger != partner_name:
        if self.robot_poses[stranger] == [-1,-1]:
          time.sleep(.1) # wait for robot's pos to be updated for the first time
          if self.robot_poses[stranger] == [-1,-1]:
            continue # skip this iteration
        # TODO: if this robot is a leader, avoid it (where does this factor in?)
        # For each stranger robot, the weight of the goal
        # must increase to keep it proportionally important.
        goal_coefficient = goal_coefficient + 30
        stranger_pos = np.array(self.robot_poses[stranger])
        stranger_pointer = stranger_pos-np.array([x,y]) # vector from current robot to stranger robot
        stranger_dist = np.linalg.norm(stranger_pointer)
        stranger_force = self.n*2.5/np.linalg.norm(stranger_pointer)*stranger_pointer/np.linalg.norm(stranger_pointer)
        net_force = net_force-stranger_force
        #print(robot_name+": due to "+stranger+": "+str(-stranger_force))
        if stranger_dist<.5:
          print(robot_name+": Oh dear! Pardon me, " + stranger + "! So sorry to have bumped you!")

    # Add the social force of the goal.
    vect_to_goal = [goal_x, goal_y] - np.array([x,y])
    dist_to_goal = np.linalg.norm(vect_to_goal)
    goal_vect = 1/dist_to_goal*vect_to_goal/dist_to_goal
    net_force = net_force + goal_coefficient*goal_vect

    return net_force

if __name__ == "__main__":
  SocialForceServer()


