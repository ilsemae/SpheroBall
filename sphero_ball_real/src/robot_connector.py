#!/usr/bin/env python

import rospy
import sys
from sphero_ball_real.srv import Connector
from std_msgs.msg import Int32, ColorRGBA
from rosgraph_msgs.msg import Log
from subprocess import Popen

special = 1 # index of the special Sphero

colors = (
  ColorRGBA(255, 128, 23, 0), # leader
  ColorRGBA(100, 3, 100, 0),  # follower
  ColorRGBA(0, 200, 90, 0)    # special
)

class RobotConnector:

  def __init__(self):
    self.connected = 0
    self.processes = {}
    rospy.init_node('robot_setup')
    rospy.Subscriber('rosout', Log, self.set_connection_state)
    rospy.Service('connect_sphero', Connector, self.connect_sphero)
    rospy.Service('disconnect_sphero', Connector, self.disconnect_sphero)
    rospy.spin()

  def connect_sphero(self, data):
    """ Establishes a Bluetooth connection with the Sphero. """
    i = data.index
    # Ensure two Spheros do not have the same name.
    if i in self.processes: return 0
    # Determine the Sphero's name and color.
    name = 'sphero' + str(i+1)
    color = colors[-1] if i == special else colors[i%2]
    # Start a new sphero.py process to begin connecting.
    p = Popen(['rosrun', 'sphero_node', 'sphero.py', '__ns:=' + name])
    # Spin our wheels while connecting.
    self.connected = 0
    while not self.connected:
      # Break if the process died.
      if p.poll(): break
    # If the connection was a success,
    # set the color and return true.
    if self.connected:
      self.processes[i] = p
      rospy.Publisher(name + '/set_color', ColorRGBA, queue_size=1, latch=1).publish(color)
      return 1
    # Otherwise, return false.
    return 0

  def disconnect_sphero(self, data):
    """ Breaks the Bluetooth connection with the Sphero. """
    i = data.index
    # Attempt to terminate the process.
    try:
      p = self.processes[i]
      p.terminate()
      while p.poll():
        pass
      self.processes.pop(i)
      return 1
    # Return false if there are any errors.
    except:
      return 0

  def set_connection_state(self, log):
    """ Sets a status variable based on the connection success. """
    if "Connect to Sphero with address" in log.msg:
      self.connected = 1
    elif "Failed to connect to Sphero." in log.msg:
      self.connected = 0

if __name__ == '__main__':
  RobotConnector()

