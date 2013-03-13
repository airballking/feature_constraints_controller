#!/usr/bin/env python

# A debugging window for the constraint controller

import roslib
roslib.load_manifest('constraint_msgs')
import rospy

from constraint_msgs.msg import ConstraintConfig, ConstraintCommand, ConstraintState

from math import *

import numpy

import wx

# TODO: refactor into 2D FlexGrid
# -> ConstraintPanel no longer inherits from wx.Panel

#helper function

def clamp(minimum, x, maximum):
  return max(minimum, min(x, maximum))

def extract(item_list, index, default):
  try:
    return item_list[index]
  except:
    return default

class ConstraintPanel(wx.Panel):
  """A Panel that shows the state of one constraint"""
  def __init__(self, parent, id, pos_min=-1, pos_max=1):
    """Display constraint state information.

    The arguments pos_min and pos_max are the minimum and maximum position that
    are to be expected. THe whole drawing area is scaled to this range.
    """
    wx.Panel.__init__(self, parent, id)
    self.Bind(wx.EVT_PAINT, self._paint)
    self.Bind(wx.EVT_SIZE, self._resize)

    self.SetBackgroundColour('white')
    self.sizer = wx.GridSizer(1, 0, 0, 0)
    self.SetSizer(self.sizer)

    self.label_sizer = wx.GridSizer(0, 1, 0, 0)

    self.constraint_name_label = wx.StaticText(self, id=-1, label='blablabla')
    self.label_sizer.Add(self.constraint_name_label, 1, wx.ALIGN_BOTTOM)

    font = self.constraint_name_label.GetFont()
    font.SetPointSize(12)
    font.SetWeight(wx.FONTWEIGHT_BOLD);
    self.constraint_name_label.SetFont(font);


    self.constraint_func_label = wx.StaticText(self, id=-1, label='blubb')
    self.label_sizer.Add(self.constraint_func_label, 1, wx.LEFT)

    self.sizer.Add(self.label_sizer, 1, wx.EXPAND)

    self.canvas = wx.Panel(self, id=-1)
    self.canvas.SetBackgroundColour('white')
    self.sizer.Add(self.canvas, 1, wx.EXPAND)

    self.pos_min = pos_min
    self.pos_max = pos_max
    self.vel_scale = 1.0  # show where the controller wants to be in 1.0 seconds

    self.box = (0.1, 0.9)
    self.pos = 0.5


  def set_description(self, constraint_name, constraint_function, feature1, feature2):
    """Set the textual description for this constraint.

    All parameters are strings.
    """
    #set constraint name
    self.constraint_name_label.SetLabel(constraint_name)

    #set constraint function
    func = constraint_function + '(' + feature1 + ', ' + feature2 + ')'
    self.constraint_func_label.SetLabel(func)

  def set_command(self, weight, pos_lo, pos_hi, vel_lo, vel_hi):
    width = self.pos_max - self.pos_min

    if weight < 0.1:
      self.canvas.SetBackgroundColour('cyan')
    else:
      self.canvas.SetBackgroundColour('white')

    left  = clamp(-0.1, (pos_lo - self.pos_min) / width, 1.1)
    right = clamp(-0.1, (pos_hi - self.pos_min) / width, 1.1)
    print 'cmd: (',pos_lo,' ',pos_hi,') -> (',left,', ',right,')'
    self.box = (left, right)

  def _resize(self, event):
    self.Refresh()
    event.Skip()

  def set_state(self, weight, pos, vel_desired, singular_value):
    width = self.pos_max - self.pos_min
    self.pos = clamp(0.0, (pos - self.pos_min) / width, 1.0)
    self.vel = clamp(0.0, (vel_desired * self.vel_scale) / width, 1.0)

  def _paint(self, event):
    #determine size
    pixel_width  = self.canvas.Size[0]
    left = self.box[0] * pixel_width
    right = self.box[1] * pixel_width
    dc = wx.PaintDC(self.canvas)
    dc.SetBrush(wx.GREY_BRUSH)
    dc.DrawRectangle(left, 10, right - left, 15)
    dc.DrawLine(self.pos * pixel_width,  5, self.pos * pixel_width, 30)
    dc.DrawLine(self.pos * pixel_width, 17, (self.pos + self.vel) * pixel_width, 17)


class ConstraintView(wx.Panel):
  def __init__(self, parent, id):
    wx.Panel.__init__(self, parent, id)

    self.SetBackgroundColour("grey")
    self.sizer = wx.GridSizer(0, 1, 3, 3)
    self.SetSizer(self.sizer)

    self.frame = parent
    self.panels = []

  def reconstruct_panels(self, num): 
    if len(self.panels) != num:
      #TODO: delete old panels
      print 'Re-creating the panels'
      for i in range(num):
        panel = ConstraintPanel(self, i)
        self.panels.append(panel)
        self.sizer.Add(panel, 1, wx.EXPAND)
      self.sizer.Layout()


class ConstraintDashboard:
  def __init__(self, view, prefix='/constraint_controller'):
    """Initialize ROS communication."""

    rospy.init_node('constraint_dashboard')
    self.name = rospy.names.get_name()

    self.sub_config  = rospy.Subscriber(prefix+'/constraint_config', ConstraintConfig, self._config_callback)
    self.sub_command = rospy.Subscriber(prefix+'/constraint_command', ConstraintCommand, self._command_callback)
    self.sub_state   = rospy.Subscriber(prefix+'/constraint_state', ConstraintState, self._state_callback)

    self.config  = None
    self.command = None
    self.state   = None

    self.view = view

    self._shutdown_timer = wx.Timer()
    self._shutdown_timer.Bind(wx.EVT_TIMER, self._on_shutdown_timer)
    self._shutdown_timer.Start(25)

  def _config_callback(self, msg):
    print 'config'
    self.config = msg
    self.view.reconstruct_panels(self.num_constraints())
    self._dispatch_constraint_description(msg)

  def _command_callback(self, msg):
    print 'command'
    self.command = msg
    self._dispatch_command(msg)

  def _state_callback(self, msg):
    self.state = msg
    self._dispatch_state(msg)

  def _dispatch_constraint_description(self, config):
    for i in range(len(self.view.panels)):
      c = config.constraints[i]
      self.view.panels[i].set_description(c.name, c.function, c.tool_feature.name, c.world_feature.name)

  def _dispatch_command(self, command):
    for i in range(len(self.view.panels)):
      cmd = self.command
      self.view.panels[i].set_command(extract(cmd.weight, i,  0.0),
                                      extract(cmd.pos_lo, i,  0.0), extract(cmd.pos_hi,  i, 0.0),
                                      extract(cmd.max_vel, i, 0.0), extract(cmd.min_vel, i, 0.0))

  def _dispatch_state(self, state):
    for i in range(len(self.view.panels)):
      self.view.panels[i].set_state(state.weights[i], state.chi[i], state.ydot_desired[i], state.singular_values[i])

  def num_constraints(self):
    if self.config != None:
      return len(self.config.constraints)
    elif self.command != None:
      return len(self.command.pos_lo)
    elif self.state != None:
      return len(self.state.chi)
    else:
      return 0

  def name(self):
    return self.rospy.get_name()

  def _on_shutdown_timer(self, event):
    """Shut down the program when the node closes."""
    self.view.Refresh()  # must do the Refresh here because of threading issues.
    if rospy.is_shutdown():
      wx.Exit()



if __name__ == "__main__":
  app = wx.PySimpleApp()
  frame = wx.Frame(None, -1, 'dashboard', size = (200, 200))

  view = ConstraintView(frame, -1)

  prefix = rospy.get_param('/constraint_controller_prefix', '/feature_controller')
  rospy.loginfo('using topic prefix '+prefix)
  dashboard = ConstraintDashboard(view, prefix)

  frame.ClientSize = view.BestSize
  frame.Show(True)
  app.MainLoop()

