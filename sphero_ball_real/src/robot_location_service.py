#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sphero_ball_real.srv import RobotLocator, RobotLocatorResponse

class RobotLocationServer:

  def __init__(self):
    """ Instantiates the robot location server. """
    rospy.init_node('robot_location_server')
    rospy.Service('robot_locator', RobotLocator, self.handle)
    # Total number of robots on the dance floor.
    self.n = rospy.get_param('total_robot_n')
    # Used to store the pose of each robot.
    self.robot_poses = {"sphero"+str(i+1): [0,0,0] for i in range(self.n)}
    # Create a pose subscriber for each robot.
    for i in range(self.n):
      name = 'sphero'+str(i+1)
      rospy.Subscriber(name + '/odom', Odometry, self.pose_callback, name)
    # Print a ready message and then spin forever.
    print "Ready to locate robots."
    rospy.spin()

  def handle(self, req):
    """ Used to get the position of a robot. """
    # Wait for the Sphero's pose to be updated for the first time.
    while self.robot_poses[req.name] == [0,0,0]: pass
    # Return the response to the service call.
    return RobotLocatorResponse(*self.robot_poses[req.name])

  def pose_callback(self, data, name):
    """ Used to set the pose for Sphero positions. """
    self.robot_poses[name][0] = data.pose.pose.position.x
    self.robot_poses[name][1] = data.pose.pose.position.y

if __name__ == "__main__":
  RobotLocationServer()


