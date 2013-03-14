#!/usr/bin/env python

# A debugging window for the constraint controller

import roslib
roslib.load_manifest('constraint_msgs')
roslib.load_manifest('sensor_msgs')
import rospy

from sensor_msgs.msg import JointState
from constraint_msgs.msg import ConstraintConfig, ConstraintCommand, ConstraintState

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
    self.background_color = config['active_color']
    self.background_brush = wx.Brush(self.background_color)
    self.constraint_label = 'blabla'
    self.func_label = 'blubb'
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
      for panel in self.panels:
        self.sizer.Detach(panel)
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

    self.view = view

    self._shutdown_timer = wx.Timer()
    self._shutdown_timer.Bind(wx.EVT_TIMER, self._on_shutdown_timer)
    self._shutdown_timer.Start(25)

  def _config_callback(self, msg):
    print 'config'
    self.config = msg
    self.view.reconstruct_panels(self.num_constraints())
    for i in range(len(self.view.panels)):
      c = msg.constraints[i]
      self.view.panels[i].set_description(c.name, c.function, c.tool_feature.name, c.world_feature.name)

  def _command_callback(self, msg):
    for i in range(len(self.view.panels)):
      self.view.panels[i].set_command(extract(msg.weight,  i, 0.0),
                                      extract(msg.pos_lo,  i, 0.0),
                                      extract(msg.pos_hi,  i, 0.0),
                                      extract(msg.max_vel, i, 0.0),
                                      extract(msg.min_vel, i, 0.0))

  def _state_callback(self, msg):
    self.state = msg
    self.ydot_real = recover_ydot(msg, self.qdot)
    for i in range(len(self.view.panels)):
      self.view.panels[i].set_state(msg.weights[i], msg.chi[i], msg.ydot_desired[i], float(self.ydot_real[i,0]))

  def _js_callback(self, msg):
    self.js = msg
    self.qdot = extract_qdot(msg, self.joint_names)

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
  frame = wx.Frame(None, -1, 'constraint dashboard', size = (800, 200))

  view = ConstraintView(frame, -1)

  prefix = rospy.get_param('/constraint_controller_prefix', '/feature_controller')
  rospy.loginfo('using topic prefix '+prefix)
  dashboard = ConstraintDashboard(view, prefix)

  rospy.sleep(0.5)

  frame.ClientSize = view.BestSize
  frame.Show(True)
  app.MainLoop()

