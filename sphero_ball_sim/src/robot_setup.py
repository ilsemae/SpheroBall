#!/usr/bin/env python

import rospy
from turtlesim.srv import Spawn
from std_msgs.msg import String
from rosgraph_msgs.msg import Log

class RobotSetup:

  def __init__(self):
    self.n = rospy.get_param('total_robot_n')
    rospy.init_node('robot_setup')
    pub = rospy.Publisher('music', String, queue_size=10)
    rospy.wait_for_service('spawn')
    self.spawn_sim_spheros()
    rospy.set_param('setup_complete', True)
    raw_input('Press enter to begin the dance.')
    pub.publish(String('begin'))
    raw_input('Press enter to end the dance.')
    pub.publish(String('end'))
    rospy.spin()

  def spawn_sim_spheros(self):
    for i in range(self.n):
      name = 'sphero'+str(i+1)
      color = i%2
      if i == 1:
        color = 5
      rospy.ServiceProxy('spawn', Spawn).call(13, 7, 0, name, color)


if __name__ == '__main__':
  RobotSetup()
	
