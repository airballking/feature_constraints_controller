#!/usr/bin/env python

# A debugging window for the constraint controller

import roslib
roslib.load_manifest('constraint_msgs')
roslib.load_manifest('sensor_msgs')
import rospy

from sensor_msgs.msg import JointState
from constraint_msgs.msg import ConstraintConfig, ConstraintCommand, ConstraintState

from threading import Lock

from math import *

import numpy

import wx

from constraint_debugging import *

config = {'pos_min':   -1.0, # minimum feature position that is to be expected.
          'pos_max':    1.0, # maximum feature position that is to be expected.
          'vel_scale':  1.0, # show where the controller wants to be in x second
          'active_color':   'white', # background color of active constraints
          'inactive_color': 'cyan'   # background color of inactive constraints
}


#helper function

def clamp(minimum, x, maximum):
  return max(minimum, min(x, maximum))

def extract(item_list, index, default):
  try:
    return item_list[index]
  except:
    return default

class Painter(wx.Window):
  def __init__(self, parent, id, painter_cb):
    wx.Window.__init__(self, parent, id)
    self.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)
    self.SetMinSize(wx.Size(400, 50))
    self.Bind(wx.EVT_PAINT, painter_cb)
    self.Bind(wx.EVT_ERASE_BACKGROUND, self._erase)

  def _erase(self, event):
    pass
    

class ConstraintPanel:
  """A Panel that shows the state of one constraint"""
  def __init__(self, parent, id):
    """Display constraint state information."""
    self.text_panel = wx.Panel(parent, id)
    self.text_panel.SetBackgroundColour(config['active_color'])

    self.label_sizer = wx.GridSizer(0, 1, 0, 0)
    self.text_panel.SetSizer(self.label_sizer)

    self.constraint_name_label = wx.StaticText(self.text_panel, id=-1, label='blablabla')
    self.label_sizer.Add(self.constraint_name_label, 1, wx.ALIGN_BOTTOM)

    font = self.constraint_name_label.GetFont()
    font.SetPointSize(12)
    font.SetWeight(wx.FONTWEIGHT_BOLD);
    self.constraint_name_label.SetFont(font);

    self.constraint_func_label = wx.StaticText(self.text_panel, id=-1, label='blubb')
    self.label_sizer.Add(self.constraint_func_label, 1, wx.LEFT)

    parent.sizer.Add(self.text_panel, 1, wx.EXPAND)

    self.canvas = Painter(parent, -1, self._paint)
    self.canvas.SetBackgroundColour(config['active_color'])
    parent.sizer.Add(self.canvas, 1, wx.EXPAND)

    # initialize variables
    self.box = (-0.1, 1.1)
    self.pos = 0.5
    self.vel = 0.0
    self.vel_desired = 0.0
    self.vel_act = 0.0
    self.background_color = config['active_color']
    self.background_brush = wx.Brush(self.background_color)
    self.constraint_label = 'Constraint Name'
    self.func_label = 'function(feature_1, feature_2)'
    self.do_update = False

  def set_description(self, constraint_name, constraint_function, feature1, feature2):
    """Set the textual description for this constraint.

    All parameters are strings.
    """
    #set constraint name
    self.constraint_label = constraint_name

    #set constraint function
    self.func_label = constraint_function + '(' + feature1 + ', ' + feature2 + ')'

    self.do_update = True

  def set_command(self, weight, pos_lo, pos_hi, vel_lo, vel_hi):
    width = float(config['pos_max'] - config['pos_min'])

    if weight < 0.1:
      self.background_color = config['inactive_color']
    else:
      self.background_color = config['active_color']
    self.background_brush = wx.Brush(self.background_color)
    

    left  = clamp(-0.1, (pos_lo - config['pos_min']) / width, 1.1)
    right = clamp(-0.1, (pos_hi - config['pos_min']) / width, 1.1)
    self.box = (left, right)
    self.do_update = True

  def _resize(self, event):
    self.Refresh()
    event.Skip()

  def _update(self):
    self.text_panel.SetBackgroundColour(self.background_color)
    self.constraint_name_label.SetLabel(self.constraint_label)
    self.constraint_func_label.SetLabel(self.func_label)
    self.text_panel.GetParent().Layout()
    

  def set_state(self, weight, pos, vel_desired, vel_actual):
    self.vel_desired = vel_desired

    width = float(config['pos_max'] - config['pos_min'])
    self.pos = clamp(0.0, (pos - config['pos_min']) / width, 1.0)
    self.vel = clamp(-2.0, (vel_desired * config['vel_scale']) / width, 2.0)
    self.vel_act = clamp(-2.0, (vel_actual * config['vel_scale']) / width, 2.0)

  def _paint(self, event):
    if self.do_update:
      self._update()
      self.do_update = False

    width  = self.canvas.Size[0]
    left = self.box[0] * width
    right = self.box[1] * width
    dc = wx.PaintDC(self.canvas)
    dc.SetBackground(self.background_brush)
    dc.Clear()
    dc.SetBrush(wx.GREY_BRUSH)
    dc.DrawRectangle(left, 10, right - left, 15)
    dc.DrawLine(self.pos * width,  5, self.pos * width, 30)
    dc.DrawLine(self.pos * width, 17, (self.pos + self.vel) * width, 17)

    dc.DrawLine(self.pos * width,  5, (self.pos + self.vel_act) * width, 17)
    dc.DrawLine(self.pos * width, 30, (self.pos + self.vel_act) * width, 17)

    dc.DrawText('v_des=% 5.3f' % (self.vel_desired), width-200, 30)
    dc.DrawText('v_act=% 5.3f' % (self.vel_act), width-100, 30)


class ConstraintView(wx.Panel):
  def __init__(self, parent, id):
    wx.Panel.__init__(self, parent, id)

    self.SetBackgroundColour("grey")
    self.sizer = wx.FlexGridSizer(0, 2, 3, 0)
    self.sizer.AddGrowableCol(1, 1.0)
    self.SetSizer(self.sizer)

    self.panels = []

  def reconstruct_panels(self, num): 
    if len(self.panels) != num:
      print 'Re-creating the panels'
      for i in range(self.sizer.GetRows()):
         self.sizer.RemoveGrowableRow(i)
      self.sizer.Clear(True)
      self.panels = []

      self.sizer.SetRows(num)
      for i in range(num):
        panel = ConstraintPanel(self, i)
        self.panels.append(panel)
        self.sizer.AddGrowableRow(i, 1.0)
      self.sizer.Layout()


class ConstraintDashboard:
  def __init__(self, view, prefix='/constraint_controller'):
    """Initialize ROS communication."""
    self.qdot    = [0.0]*7

    rospy.init_node('constraint_dashboard')
    self.name = rospy.names.get_name()
    self.joint_names = joint_names(prefix)

    self.sub_config  = rospy.Subscriber(prefix+'/constraint_config', ConstraintConfig, self._config_callback)
    self.sub_command = rospy.Subscriber(prefix+'/constraint_command', ConstraintCommand, self._command_callback)
    self.sub_state   = rospy.Subscriber(prefix+'/constraint_state', ConstraintState, self._state_callback)
    self.sub_js      = rospy.Subscriber('/joint_states', JointState, self._js_callback)

    self.config  = None
    self.command = None
    self.state   = None
    self.do_reconfigure = False
    self.mutex = Lock()

    self.view = view

    self._redraw_timer = wx.Timer()
    self._redraw_timer.Bind(wx.EVT_TIMER, self._on_redraw_timer)
    self._redraw_timer.Start(25)

  def _config_callback(self, msg):
    self.mutex.acquire()
    try:
      self.config = msg
      self.do_reconfigure = True 
    finally:
      self.mutex.release()

  def _command_callback(self, msg):
    self.mutex.acquire()
    try:
      for i in range(len(self.view.panels)):
        self.view.panels[i].set_command(extract(msg.weight,  i, 0.0),
                                        extract(msg.pos_lo,  i, 0.0),
                                        extract(msg.pos_hi,  i, 0.0),
                                        extract(msg.max_vel, i, 0.0),
                                        extract(msg.min_vel, i, 0.0))
    finally:
      self.mutex.release()

  def _state_callback(self, msg):
    self.mutex.acquire()
    try:
      self.state = msg
      self.ydot_real = recover_ydot(msg, self.qdot)
      for i in range(min(msg.chi, len(self.view.panels))):
        self.view.panels[i].set_state(msg.weights[i], msg.chi[i], msg.ydot_desired[i], float(self.ydot_real[i,0]))
    finally:
      self.mutex.release()

  def _js_callback(self, msg):
    self.mutex.acquire()
    try:
      self.js = msg
      self.qdot = extract_qdot(msg, self.joint_names)
    finally:
      self.mutex.release()

  def num_constraints(self):
    num = float('inf')
    if self.config != None:
      num = min(num, len(self.config.constraints))
    if self.command != None:
      num = min(num, len(self.command.pos_lo))
    if self.state != None:
      num = min(num, len(self.state.chi))
    if self.config == None and self.command == None and self.state == None:
      num = 0
    return num

  def name(self):
    return self.rospy.get_name()

  def _on_redraw_timer(self, event):
    """Our hook of the wxWidgets loop.

    Here we must call all wxWidgets related functions: Refresh,
    Widget construction and shutdown."""
    self.mutex.acquire()
    try:
      if self.do_reconfigure:
        self.do_reconfigure = False
        self.view.reconstruct_panels(len(self.config.constraints))
        for i in range(len(self.view.panels)):
          c = self.config.constraints[i]
          self.view.panels[i].set_description(c.name, c.function, c.tool_feature.name, c.world_feature.name)
      self.view.Refresh()
      if rospy.is_shutdown():
        wx.Exit()
    finally:
      self.mutex.release()



if __name__ == "__main__":
  app = wx.PySimpleApp()
  frame = wx.Frame(None, -1, 'constraint dashboard', size = (800, 200))

  view = ConstraintView(frame, -1)

  prefix = rospy.get_param('/constraint_controller_prefix', '/feature_controller')
  rospy.loginfo('using topic prefix '+prefix)
  dashboard = ConstraintDashboard(view, prefix)

  rospy.sleep(0.5)

  frame.ClientSize = view.BestSize
  frame.Show(True)
  app.MainLoop()

