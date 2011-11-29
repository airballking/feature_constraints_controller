#!/usr/bin/env python

# shows the state of the jacobian in some fancy color image

import roslib
roslib.load_manifest('motion_viz')
#import rospy

from math import *

import numpy

import marker

import wx

class DataPanel(wx.Panel):
  """ creates a panel with matrix """
  def __init__(self, parent, id):
    wx.Panel.__init__(self, parent, id)
    self.Bind(wx.EVT_PAINT, self._paint)
    #self.SetBackgroundColour("white")

  def set_values(self, values):
    self.values = values

  def _paint(self, event):
    print 'draw'
    size = 15
    value_range = 1.0
    f_col = (lambda x: marker.color_code(x, [-value_range, 0.0, value_range],
                        [[0.0, 0.0, 1.0], [1.0, 1.0, 1.0], [1.0, 0.0, 0.0]]))

    colors = numpy.array([[f_col(c) for c in r] for r in self.values])

    width = colors.shape[1]*size
    height = colors.shape[0]*size

    rgb = numpy.array(255.99*colors, dtype='uint8').repeat(size,0).repeat(size,1)
    #rgb = grey.repeat(3,1) #lazy greyscale image

    bitmap = wx.BitmapFromBuffer(width, height, rgb.flatten())
    dc = wx.PaintDC(self)
    dc.DrawBitmap(bitmap, 30, 30)


class RosComm:
  def __init__(self):
    """ initialize ROS """

    rospy.init_node('ctrl_viz')
    self.name = rospy.names.get_name()

    self.sub_jac = rospy.Subscriber('~jacobian', Float64MultiArray, self._jac_callback)

    self._shutdown_timer = wx.Timer()
    self._shutdown_timer.Bind(wx.EVT_TIMER, self._on_shutdown_timer)
    self._shutdown_timer.Start(100)

  def _jac_callback(msg):
    pass

  def _on_shutdown_timer(self, event):
    """shut down the program when the node closes"""
    if rospy.is_shutdown():
      wx.Exit()



# main

if __name__ == "__main__":

  app = wx.PySimpleApp()

  #ros = RosComm()

  frame = wx.Frame(None, -1, 'test', size = (400, 310))
  panel = DataPanel(frame,-1)

  #frame.ClientSize = panel.BestSize

  panel.set_values([[-0.7, -0.3, 0.3, 0.8], [1.0, 0.0, -1.0, 0.5]])

  frame.Show(True)
  app.MainLoop()

