#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class RobotSetup:

  def __init__(self):
    self.n = rospy.get_param('total_robot_n')
    rospy.init_node('robot_setup')
    pub = rospy.Publisher('music', String, queue_size=10)
    # <connect to Spheros>
    rospy.set_param('setup_complete', True)
    raw_input('Press enter to begin the dance.')
    pub.publish(String('begin'))
    raw_input('Press enter to end the dance.')
    pub.publish(String('end'))
    rospy.spin()

if __name__ == '__main__':
  RobotSetup()


