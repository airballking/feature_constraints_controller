#!/usr/bin/python

from math import *
import time

import roslib; roslib.load_manifest('motionControl')


import yarp
import yarp_tools



class JoystickTrigger:
  def __init__(self, btn_index, callback=(lambda : 0)):
    self.btn_index = btn_index
    self.callback = callback
    import roslib
    roslib.load_manifest('rospy')
    roslib.load_manifest('joy')
    from joy.msg import Joy
    import rospy
    self.rospy = rospy
    rospy.init_node('joy_trigger', disable_signals=True)
    self.sub = rospy.Subscriber('/joy', Joy, self._joy_callback)
    self.btn_state = -1

  def _joy_callback(self, msg):
    btn = msg.buttons[self.btn_index]
    if btn == 1 and self.btn_state == 0:
      self.callback()
    self.btn_state = btn

  def ok(self):
    return not self.rospy.is_shutdown()




def read_from_port(port):
  bottle = port.read(True)
  return [bottle.get(i).asDouble() for i in range(bottle.size())]



class Statistics:
  def __init__(self):
    self.n = 0.0
    self.sum = 0.0
    self.sumsq = 0.0
  def add_reading(self, val):
    """ add a reading to the statistics """
    self.sum += val
    self.sumsq += val*val
    self.n += 1
  def get(self):
    """ return mean and std. dev """
    mean = self.sum / self.n
    var  = self.sumsq / self.n - mean*mean
    var  = max(var, 0)
    return (mean, sqrt(var))
  def reset(self):
    self.__init__()


class TorqueAnalyzer:
  def __init__(self, robot_name='/lwr/left'):
    full_name = robot_name+'/torque_analysis'

    self.torque_p = yarp_tools.new_port(full_name + '/toRobot_torque', 'in', robot_name+'/robot/torque')

    self.torque_raw_p = yarp_tools.new_port(full_name + '/toRobot_torque_raw', 'in', robot_name+'/robot/torque_raw')
    self.pos_p = yarp_tools.new_port(full_name + '/toRobot_pos', 'in', robot_name+'/robot/pos')

    self.torque_stat = []
    self.torque_raw_stat = []
    self.pos_stat = Statistics()

    for i in range(7):
      self.torque_stat.append(Statistics()) 
      self.torque_raw_stat.append(Statistics()) 

    self.max_iter = 100


  def callback(self):
    """ Perform torque sensor measurement """

    self.pos_stat.reset()
    for i in range(7):
      self.torque_stat[i].reset()
      self.torque_raw_stat[i].reset()

    for i in range(self.max_iter):
      torques = read_from_port(self.torque_p)
      torques_raw = read_from_port(self.torque_raw_p)
      pos = read_from_port(self.pos_p)

      pos_sum2 = sqrt(sum([p*p for p in pos]))
      self.pos_stat.add_reading(pos_sum2) 

      for j in range(len(pos)):
        self.torque_stat[j].add_reading(torques[j]) 
        self.torque_raw_stat[j].add_reading(torques_raw[j]) 


    # print the statistics
    for i in range(len(pos)):
      print self.torque_stat[i].get()[0],
    for i in range(len(pos)):
      print self.torque_stat[i].get()[1],

    for i in range(len(pos)):
      print self.torque_raw_stat[i].get()[0],
    for i in range(len(pos)):
      print self.torque_raw_stat[i].get()[1],

    print self.pos_stat.get()[1]



# main

analyzer = TorqueAnalyzer()
trigger = JoystickTrigger(14, analyzer.callback)

while trigger.ok():
  time.sleep(1.0)

