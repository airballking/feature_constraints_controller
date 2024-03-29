#!/usr/bin/env python

""" Show a window with buttons.

The button texts are taken from the ROS parameter ~buttons, which must be
a list of strings. Whenever a button is pressed, it's text is sent to the
topic /state which is of type std_msgs/String.

"""

import roslib
roslib.load_manifest('feature_constraints_utils')
from std_msgs.msg import String
import rospy

from math import pi

import wx

class ButtonPanel(wx.Panel):
  """A panel with buttons."""
  def __init__(self, parent, id, button_names):
    wx.Panel.__init__(self, parent, id)

    self.SetBackgroundColour("white")
    self.sizer = wx.GridSizer(0, 1, 0, 0)
    self.SetSizer(self.sizer)

    self.button_names = button_names

    self.Bind(wx.EVT_BUTTON, self._button_update)

    #Add buttons
    for i in range(len(self.button_names)):
      b = wx.Button(self, id=i,
            label = button_names[i])
      #b.SetSizeHints(300, 40)

      self.sizer.Add(b, 1, wx.EXPAND)

  def set_button_cb(self, callback):
    """Set the callback that will be called when a button is pressed."""
    self.button_callback = callback

  def _button_update(self, event):
    """Internal callback which delegates to self.button_callback."""
    self.button_callback(self.button_names[event.Id])


class RosComm:
  def __init__(self):
    """Initialize ROS."""

    self.joint_names = None

    rospy.init_node('buttons_gui')
    self.name = rospy.names.get_name()

    if rospy.client.has_param('~buttons'):
      self.button_names = rospy.client.get_param('~buttons')
    else:
      self.button_names = None

    self._pub = rospy.Publisher('/state', String)

    self._shutdown_timer = wx.Timer()
    self._shutdown_timer.Bind(wx.EVT_TIMER, self._on_shutdown_timer)
    self._shutdown_timer.Start(100)

  def _on_shutdown_timer(self, event):
    """Shut down the program when the node closes."""
    if rospy.is_shutdown():
      wx.Exit()

  def send_event(self, button_text):
    """Publish button text."""
    self._pub.publish(String(data=button_text))



if __name__ == "__main__":

  app = wx.PySimpleApp()

  ros = RosComm()

  if not ros.button_names:
    rospy.logerr("parameter ~buttons not set")
    exit()

  frame = wx.Frame(None, -1, ros.name, size = (400, 310))
  panel = ButtonPanel(frame,-1, ros.button_names)
  panel.set_button_cb(ros.send_event) # on a button press, ROS sends the text.

  frame.ClientSize = panel.BestSize

  frame.Show(True)
  app.MainLoop()

