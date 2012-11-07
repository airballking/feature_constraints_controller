#!/usr/bin/python

import roslib
roslib.load_manifest('constraint_msgs')
roslib.load_manifest('motion_viz')
import PyKDL as kdl
import marker
import rospy
import tf
import tf_conversions.posemath as pm

from constraint_msgs.msg import ConstraintConfig, Constraint, Feature
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA


grey    = ColorRGBA(0.7, 0.7, 0.7, 0.5)
red     = ColorRGBA(0.7, 0.1, 0.1, 1.0)
yellow  = ColorRGBA(0.7, 0.7, 0.1, 0.6)

_config_ = {'line_width': 0.02,
            'frame_id': '/base_link',
            'ns': 'features',
            'marker_color': grey,
            'marker_id': 0}


def msg2vec(v):
  """Convert a geometry_msgs/Vector3 to a KDL Vector."""
  return kdl.Vector(v.x, v.y, v.z)


def msg2feature(msg):
  """Convert a constraint_msgs/Feature to a Feature."""
  pos = msg2vec(msg.position)
  dir = msg2vec(msg.direction)
  return Feature(msg.type, pos, dir, msg.frame_id)


def marker_base(**args):
  global _config_

  if args.has_key('line_width'):
    line_width = args.pop('line_width')
  else:
    line_width = _config_['line_width']

  m = marker.Marker(id=_config_['marker_id'],
                    ns=_config_['ns'], **args)
  m.header.frame_id = _config_['frame_id']
  m.scale.z = line_width
  if not args.has_key('color'):
    m.color = _config_['marker_color']

  _config_['marker_id'] += 1

  return m


def marker_line(start, end, **args):
  global _config_
  m = marker_base(type=marker.Marker.CYLINDER, **args)
  marker.align(m, start, end, m.scale.z)
  return m


def marker_point(point, **args):
  global _config_
  m = marker_base(type=marker.Marker.SPHERE, **args)
  m.pose.position = Point(point[0], point[1], point[2])
  m.scale.x = m.scale.z
  m.scale.y = m.scale.z
  return m


class LocatedVector:
  def __init__(self, pos, dir, source=None):
    self.pos = pos
    self.dir = dir
    self.source = source

  def __str__(self):
    p = self.pos
    d = self.dir
    return ('LocatedVector(kdl.Vector(%f, %f, %f), kdl.Vector(%f, %f, %f))'
            % (p.x(), p.y(), p.z(), d.x(), d.y(), d.z()))
            

  def __add__(self, other):
    """Add the second vector to the first vector.

    The resulting vector is located at the first vector's position.

    """
    return LocatedObject(self.pos, self.dir + other.dir)

  def __sub__(self, other):
    """Subtract the second vector from the first vector.

    The resulting vector is located at the first vector's position.

    """
    return LocatedVector(self.pos, self.dir - other.dir)

  def __mul__(self, scalar):
    """Scale the length of the vector.

    The position remains the same.

    """
    return LocatedVector(self.pos, self.dir * scalar)

  def compute(self):
    # propagate compute() to the source, if present
    if self.source != None:
      source = self.source()
      self.pos = source.pos
      self.dir = source.dir
    return self

  def show(self):
    return [marker_line(self.pos, self.pos + self.dir)]



class Feature:
  def __init__(self, type, pos, dir, frame_id):
    self.type = type
    self.rel_pos = pos
    self.rel_dir = dir
    self.pos = pos
    self.dir = dir
    self.frame_id = frame_id

  def transform(self, frame):
    f_now = kdl.Frame(self.rel_pos)
    f_new = frame*f_now
    self.pos = kdl.Vector(f_new.p)
    self.dir = frame.M*self.rel_dir

  def v(self):
    return LocatedVector(self.pos, self.dir / 2, self.v)

  def compute(self):
    return self

  def show(self):
    global _config_
    if self.type == 0: # LINE
      return [marker_line(self.pos - self.dir/2, self.pos + self.dir/2,
                          color=yellow)]
    elif self.type == 1: #PLANE
      dir = (self.dir / self.dir.Norm()) * _config_['line_width']/2
      return [marker_line(self.pos - dir, self.pos + dir, color=yellow, line_width=self.dir.Norm())]
    elif self.type == 2: #POINT
      w =  _config_['line_width'] * 2
      return [marker_point(self.pos, color=yellow, line_width=w)]


class Len:
  def __init__(self, vector):
    self.vector = vector

  def compute(self):
    vec = self.vector.compute()
    return vec.dir.Norm()

  def show(self):
    global _config_
    vec = self.vector.compute()
    w = _config_['line_width']*1.5
    len_marker = marker_line(vec.pos, vec.pos + vec.dir,
                             color=red, line_width=w)
    return self.vector.show() + [len_marker]


class D:
  def __init__(self, start, end):
    self.start = start
    self.end = end

  def compute(self):
    return LocatedVector(self.start.pos, self.end.pos - self.start.pos)

  def show(self):
    return self.compute().show()


class Proj_P:
  def __init__(self, vec, ref):
    self.vec = vec
    self.ref = ref
    self.result = kdl.Vector()

  def compute(self):
    vec = self.vec.compute()
    ref = self.ref.compute()
    self.result_dir = ref * (kdl.dot(ref.dir, vec.dir) / ref.dir.Norm()**2) - vec
    return LocatedVector(ref.pos, self.result_dir.dir)

  def show(self):
    vec = self.vec.compute()
    ref = self.ref.compute()
    res = self.compute()
    along = res.dir + vec.dir

    markers  = vec.show() + res.show() + ref.show()
    markers += [marker_line(vec.pos, vec.pos + along)]

    return markers


class Cos:
  def __init__(self, vec1, vec2):
    self.vec1 = vec1
    self.vec2 = vec2

  def compute(self):
    vec1 = self.vec1.compute()
    vec2 = self.vec2.compute()

    return kdl.dot(vec1, vec2) / (vec1.Norm() * vec2.Norm())

  def show(self):
    global _config_
    w = 3*_config_['line_width']

    vec1 = self.vec1.compute()
    vec2 = self.vec2.compute()
    vec2.pos = vec1.pos

    vec1_l = vec1.dir.Norm()
    vec2_l = vec2.dir.Norm()

    mrk = marker.sector(vec1.dir / vec1_l * w, vec2.dir / vec2_l * w, vec1.pos)
    mrk.ns = _config_['ns']
    mrk.id = _config_['marker_id']
    mrk.header.frame_id = _config_['frame_id']
    _config_['marker_id'] += 1
    mrk.color = red

    return [mrk] + vec1.show() + vec2.show()

class Proj_A:
  def __init__(self, vec, ref):
    self.vec = vec
    self.ref = ref

  def compute(self):
    vec = self.vec.compute()
    ref = self.ref.compute()
    res = ref * (kdl.dot(ref.dir, vec.dir) / ref.dir.Norm()**2)
    return LocatedVector(vec.pos, res.dir)

  def show(self):
    vec = self.vec.compute()
    ref = self.ref.compute()
    res = self.compute()
    perp = res.dir - vec.dir
    dist_marker = marker_line(ref.pos, ref.pos + perp)
    return vec.show() + res.show() + ref.show() + [dist_marker]



# a map from constraint name to a function (Feature x Feature -> Value)
constraint_functions = {
  'distance':  lambda (f_t, f_w) : Len(Proj_P(D(f_t, f_w), f_w.v())),
  'height':    lambda (f_t, f_w) : Len(Proj_A(D(f_t, f_w), f_w.v())),
  'perpendicular':  lambda (f_t, f_w) : Cos(f_t.v(), f_w.v())}

class ConstraintDisplay:
  def __init__(self, base_frame_id):
    #TODO: shall we index tool- and world features by their names?
    self.tool_features = []
    self.world_features = []
    self.constraints = {}
    self.base_frame_id = base_frame_id
    self.listener = tf.TransformListener()

  def transform(self):
    for f in self.tool_features + self.world_features:
      try:
        frame = self.listener.lookupTransform(self.base_frame_id,
                                              f.frame_id, rospy.Time(0))
      except:
        continue
      f.transform(pm.fromTf(frame))

  def show(self):
    markers = []
    for (i,f) in enumerate(self.tool_features):
      _config_['ns'] = 'tool_feature_' + str(i)
      markers += f.show()
    for (i,f) in enumerate(self.world_features):
      _config_['ns'] = 'world_feature_' + str(i)
      markers += f.show()
    for name in self.constraints:
      _config_['ns'] = name
      markers += self.constraints[name].show()
    _config_['ns'] = 'features'
    return markers




def callback(msg):
  global _config_

  constraint_display.tool_features = []
  constraint_display.world_features = []
  constraint_display.constraints = {}

  for c in msg.constraints:
    f_tool  = msg2feature(c.tool_feature)
    f_world = msg2feature(c.world_feature)
    if c.function in constraint_functions:
      constraint_display.tool_features.append(f_tool)
      constraint_display.world_features.append(f_world)
      constraint = constraint_functions[c.function]((f_tool, f_world))
      constraint_display.constraints[c.name] = constraint
    else:
      print "constraint function '%s' not found!" % c.function

  for m in constraint_display.show():
    marker.publish(m)


rospy.init_node('feature_vis')

# base_frame is an arbitrary frame in which the markers are displayed
# the marker locations are defined by the feature frame_id's!
base_frame_id = rospy.get_param('~base_frame', 'baker')
rospy.loginfo('base frame: ' + base_frame_id)
_config_['frame_id'] = base_frame_id
constraint_display = ConstraintDisplay(base_frame_id)

sub = rospy.Subscriber('/constraint_config', ConstraintConfig, callback)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
  constraint_display.transform()
  _config_['marker_id'] = 0
  for m in constraint_display.show():
    marker.publish(m)
  rate.sleep()
