#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sphero_ball_real.srv import *

class RobotLocationServer:

  def __init__(self):
    self.n = rospy.get_param('total_robot_n')
    self.turtle_poses = {"sphero"+str(i+1): [0,0,0] for i in range(self.n)}

    rospy.init_node('robot_location_server')
    rospy.Service('robot_locator', RobotLocator, self.handle)

    for i in range(self.n):
      name = 'sphero'+str(i+1)
      rospy.Subscriber(name + '/odom', Odometry, self.odom_callback, name)

    print "Ready to locate robots."
    rospy.spin()

  def handle(self, req):
    """ Callback for the RobotLocator service. """
    # Wait for the Sphero's pose to be updated for the first time.
    while self.turtle_poses[req.name] == [0,0,0]: pass
    # Return the response to the service call.
    return RobotLocatorResponse(*self.turtle_poses[req.name])

  def pose_callback(self, data, name):
    """ Used to set the pose of Sphero positions. """
    self.turtle_poses[name] = [data.x, data.y, data.theta]

  def odom_callback(self, data, name):
    """ Used to set the odometry of Sphero positions. """
    self.turtle_poses[name][0] = data.pose.pose.position.x
    self.turtle_poses[name][1] = data.pose.pose.position.y

if __name__ == "__main__":
  RobotLocationServer()


