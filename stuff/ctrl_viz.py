#!/usr/bin/env python

# shows the state of the jacobian in some fancy color image

import roslib
roslib.load_manifest('motion_viz')
roslib.load_manifest('motionControl')

from math import *

import numpy

import marker

import wx

class DataPanel(wx.Panel):
  """ creates a panel with matrix """
  def __init__(self, parent, id):
    wx.Panel.__init__(self, parent, id)
    self.Bind(wx.EVT_PAINT, self._paint)
    self.Bind(wx.EVT_SIZE, self._resize)
    #self.SetBackgroundColour("white")

  def set_values(self, values):
    self.values = numpy.array(values)
    self.Refresh()

  def _resize(self, event):
    self.Refresh()

  def _paint(self, event):

    #determine size
    border = 10
    size_r = (self.Parent.Size[0] - 2*border) / self.values.shape[1]
    size_c = (self.Parent.Size[1] - 2*border) / self.values.shape[0]
    size = max(1, min(size_r, size_c))

    value_range = 1.0
    f_col = (lambda x: marker.color_code(x, [-value_range, 0.0, value_range],
                        [[0.0, 0.0, 1.0], [1.0, 1.0, 1.0], [1.0, 0.0, 0.0]]))

    colors = numpy.array([[f_col(c) for c in r] for r in self.values])

    width = colors.shape[1]*size
    height = colors.shape[0]*size

    rgb = numpy.array(255.99*colors, dtype='uint8').repeat(size,0).repeat(size,1)
    bitmap = wx.BitmapFromBuffer(width, height, rgb.flatten())
    dc = wx.PaintDC(self)

    dc.DrawBitmap(bitmap, border, border)


class RosComm:
  def __init__(self, topic_name='~jacobian'):
    """ initialize ROS """

    import rospy
    from std_msgs.msg import Float64MultiArray

    self.rospy = rospy

    rospy.init_node('density_viz')
    #self.name = rospy.names.get_name()

    self.sub_jac = rospy.Subscriber(topic_name, Float64MultiArray, self._jac_callback)

    self._shutdown_timer = wx.Timer()
    self._shutdown_timer.Bind(wx.EVT_TIMER, self._on_shutdown_timer)
    self._shutdown_timer.Start(100)

  def _jac_callback(msg):
    pass

  def name(self):
    return self.rospy.get_name()


  def _on_shutdown_timer(self, event):
    """shut down the program when the node closes"""
    if self.rospy.is_shutdown():
      wx.Exit()



class YARPComm:
  def __init__(self, port_name='/jacobian', name='/density_viz'):
    """ get data from YARP bottle """

    self.callback = (lambda x: None)

    import yarp
    from yarp_tools import yarp_connect_blocking, new_port
    self.inport = new_port(name+'/data', 'in', port_name, 'udp')

    self._timer = wx.Timer()
    self._timer.Bind(wx.EVT_TIMER, self._on_timer)
    self._timer.Start(100)

  def _on_timer(self, event):
    bottle = self.inport.read(False)
    if bottle:
      s = bottle.size()
      data = numpy.array([[bottle.get(i).asDouble() for i in range(s)]])
      if s > 7:
        sqr = int(ceil(sqrt(s)))
        sqc = int(ceil(s / float(sqr)))
        
        data.resize(sqr * sqc)        # complete the rectangle ...
        data = data.reshape(sqr, sqc) # ... and reshape

      self.callback(data)


# main

if __name__ == "__main__":

  app = wx.PySimpleApp()

  import sys
  if len(sys.argv) < 3:
    print 'usage: '+sys.argv[0]+' <ros|yarp> <port_name>'
    sys.exit()

  framework = sys.argv[1]
  port_name = sys.argv[2]

  ros_comm = RosComm(port_name)

  frame = wx.Frame(None, -1, ros_comm.name(), size = (200, 200))
  panel = DataPanel(frame,-1)

  if framework == 'yarp':
    yarp_comm = YARPComm(port_name, ros_comm.name())
    yarp_comm.callback = panel.set_values


  panel.set_values(numpy.diag(numpy.arange(-1, 1.01, 0.4)))

  frame.Show(True)
  app.MainLoop()

