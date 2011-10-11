#!/usr/bin/python

import roslib
roslib.load_manifest('motionControl')
roslib.load_manifest('motion_viz')
import rospy

from math import pi, sqrt

import PyKDL as kdl
import numpy
import numpy.linalg


# utilities for manipulation with tool

def frange(start, stop, step):
  return (start + step*i for i in range(int(round((stop - start) / step))))


jjjx = [[[1.5963506698608398,
   -0.64114373922348022,
   0.036419227719306946,
   1.4789206981658936,
   -0.16140541434288025,
   -1.3096494674682617,
   -0.35271838307380676],
  [1.434751033782959,
   -1.0972779989242554,
   -1.8947522640228271,
   1.3034549951553345,
   0.43582826852798462,
   -0.87532496452331543,
   -1.5764085054397583]],
 [[1.6178646087646484,
   -0.60877937078475952,
   -0.41341796517372131,
   1.1455343961715698,
   1.2619624137878418,
   -1.7018660306930542,
   -0.16580282151699066],
  [2.8656747341156006,
   -0.025311952456831932,
   -2.0775058269500732,
   1.8874248266220093,
   2.3723063468933105,
   -0.1950422078371048,
   -0.21616850793361664],
  [2.9297683238983154,
   0.72894871234893799,
   -2.7104527950286865,
   1.4243160486221313,
   1.6966598033905029,
   -1.0742908716201782,
   0.43649238348007202]],
 [[2.3265585899353027,
   -0.8958510160446167,
   -2.1730101108551025,
   1.8039276599884033,
   1.8111274242401123,
   -1.5255725383758545,
   -0.39112409949302673],
  [1.4017268419265747,
   -1.4306706190109253,
   -1.8560434579849243,
   1.9437103271484375,
   2.3614270687103271,
   -1.7071753740310669,
   -1.8223618268966675],
  [1.3596574068069458,
   -1.4127588272094727,
   -1.8480616807937622,
   1.9745997190475464,
   2.2935843467712402,
   -1.7215982675552368,
   -1.9850354194641113]],
 [[1.9126957654953003,
   -0.98558109998703003,
   -1.8353539705276489,
   1.862993597984314,
   2.1858971118927002,
   -1.3013437986373901,
   -0.32349777221679688]],
 [[1.1474825143814087,
   -1.6574608087539673,
   -2.0055160522460938,
   1.9669015407562256,
   2.971898078918457,
   -1.7746227979660034,
   1.33432936668396],
  [0.22798782587051392,
   -1.4767004251480103,
   -0.05588802695274353,
   1.3172060251235962,
   2.3698256015777588,
   -1.3504667282104492,
   1.4881961345672607]]]

jjj = [[[1.5963506698608398,
  -0.64114373922348022,
  0.036419227719306946,
  1.4789206981658936,
  -0.16140541434288025,
  -1.3096494674682617,
  -0.35271838307380676],
  [1.5827468633651733,
   -0.69932633638381958,
   -1.4973642826080322,
   1.4696336984634399,
   0.28689926862716675,
   -1.6027486324310303,
   -1.7720389366149902]],
 [[1.7402266263961792,
   -0.93995296955108643,
   -1.950283408164978,
   1.5389466285705566,
   0.23417945206165314,
   -1.2583650350570679,
   -1.8580336570739746]],
 [[1.6176015138626099,
   -1.4207086563110352,
   -1.7797050476074219,
   1.7396180629730225,
   -0.47794327139854431,
   -0.41702964901924133,
   -0.78559231758117676]],
 [[1.7368184328079224,
   -0.76735579967498779,
   -1.0876214504241943,
   1.3740079402923584,
   0.10462969541549683,
   -1.9684770107269287,
   -0.5126340389251709]],
 [[2.1197104454040527,
   -1.0062757730484009,
   -1.9784053564071655,
   2.0278310775756836,
   0.60690391063690186,
   -1.112139105796814,
   -1.4620333909988403]],
 [[1.199406,
   -1.3561840000000001,
   -2.8534570000000001,
   0.93507700000000005,
   2.961964,
   -1.5855030000000001,
   -0.16187199999999999],
  [1.2839235067367554,
   -1.4937722682952881,
   -2.5483505725860596,
   1.8993285894393921,
   2.7365572452545166,
   -1.879456639289856,
   0.13870590925216675]],
 [[1.1706160306930542,
   -1.4990566968917847,
   -2.4687063694000244,
   1.8323588371276855,
   2.8475189208984375,
   -1.983702540397644,
   0.87012135982513428]],
 [[1.1520102024078369,
   -1.6741428375244141,
   -2.5723474025726318,
   1.9349348545074463,
   2.8093047142028809,
   -1.994239330291748,
   -0.40163359045982361]]]

jx_back = [[0.73817431926727295,
  -0.36578276753425598,
  -0.99922722578048706,
  1.6152302026748657,
  2.2692902088165283,
  -1.617437481880188,
  0.33658424019813538],
 [1.9126957654953003,
  -0.98558109998703003,
  -1.8353539705276489,
  1.862993597984314,
  2.1858971118927002,
  -1.3013437986373901,
  -0.32349777221679688]]



joints_back = [[1.4455359999999999,
   -1.208364,
   -2.9762620000000002,
   0.95012700000000005,
   2.9301309999999998,
   -1.5688800000000001,
   -0.24596000000000001],
  [1.4884489999999999,
   -0.230349,
   0.080914,
   1.2294290000000001,
   -0.195546,
   -1.807366,
   -0.71734399999999998],
  [0.79852753877639771,
   -0.97089070081710815,
   0.45446977019309998,
   1.2476179599761963,
   0.32387024164199829,
   -0.9858022928237915,
   -1.3880546092987061]]

class ToolFeature:
  OTHER = 0
  POINT = 1
  LINE = 2
  PLANE = 3
  def __init__(self, position, direction, feature_type, size=0.1):
    self.position = position
    self.direction = direction
    self.size = size
    self.feature_type = feature_type
    self.sign = self._sign(feature_type)

  def _sign(self, feature_type):
    return {ToolFeature.OTHER: 0, ToolFeature.POINT: 0,
            ToolFeature.LINE: -1, ToolFeature.PLANE: 1}[feature_type]

  def __repr__(self):
    return ('<'+['other', 'point', 'line ', 'plane'][self.feature_type]
            +' p: '+repr(self.position)+'  d: '+repr(self.direction)+'>')

  def __getstate__(self):
    p = self.position
    d = self.direction
    return {'pos': [p[0], p[1], p[2]],
            'dir': [d[0], d[1], d[2]],
            'size': self.size,
            'type': self.feature_type}
  def __setstate__(self, state):
    self.position = kdl.Vector(*state['pos'])
    self.direction = kdl.Vector(*state['dir'])
    self.size = state['size']
    self.feature_type = state['type']
    self.sign = self._sign(self.feature_type)

def _inv(x, s):
  if s == 0:
    return 0
  elif s > 0:
    return x
  else:
    return sqrt(1 - x*x)


def transform(feature, frame):
  direction = frame.M*feature.direction
  position = frame*feature.position
  return ToolFeature(position, direction, feature.feature_type)


def aligned(feature1, feature2):
  ''' zero is aligned '''
  sign = -feature1.sign*feature2.sign
  return _inv(kdl.dot(feature1.direction, feature2.direction), sign)

def incident(feature1, feature2):
  ''' zero is aligned '''
  return (feature1.position - feature2.position).Norm()


def perpendicular(feature1, feature2):
  sign = feature1.sign*feature2.sign
  return _inv(kdl.dot(feature1.direction, feature2.direction), sign)


def height(feature1, feature2):
  p = feature1.position - feature2.position
  # compute distance perpendicular to plane
  return kdl.dot(p, feature1.direction)


def distance(feature1, feature2):
  return (feature1.position - feature2.position).Norm()


class FeatureSet:
  def __init__(self, tool_features, workspace_feature):
    # tool features are in 'hand' coordinates, workspace feature in /map

    self.tool_features = []
    self.tool_features.extend(tool_features)
    self.align_vector = [[aligned, 0, 0], [incident, 0, 0]]

    # just one workspace feature for now.
    self.workspace_features = [workspace_feature]

    #special 'feature'
#    distant_feature = max(self.tool_features, key=lambda(f): f.position.Norm())
#    self.tool_features.append(ToolFeature(-distant_feature.position,
#                                           distant_feature.position,
#                                           ToolFeature.LINE))


  def __repr__(self):
    s = '<feature_set\n'
    s += '  tool_features:'
    for t in self.tool_features:
      s += '\n    '+repr(t)
    s += '\n  workspace_features:'
    for w in self.workspace_features:
      s += '\n    '+repr(w)
    s += '>'
    return s


  def coordinates(self, frame):
    w = self.workspace_features[0]
    return [func(transform(self.tool_features[i], frame), self.workspace_features[j]) for func, i, j in self.align_vector]



def derive_features(frame, feature_function, dd=0.01):
  ''' numerically derive the task angle function around the given frame.
      Also computes the inverse: this yields the instantaneous rotation axes
      for each angle.
  '''

  q0 = numpy.mat(feature_function(frame))
  size = q0.size
  print q0.size
  jac_i = numpy.mat(numpy.zeros((q0.size, 6)))

  for i in range(6):
    twist = kdl.Twist()
    twist[i] = 1.0
    frame_moved = kdl.addDelta(frame, twist, dd)
    q_i = numpy.mat( feature_function(frame_moved) )
    jac_i[0:size,i] = ((q_i - q0) / dd).T


  # the columns of jac are six twists to be displayed
  jac = numpy.linalg.pinv(jac_i)

  return (jac_i, jac)


def compute_increment(goal_features, feature_set, frame, dd=0.1):
  tw = compute_direction(goal_features, feature_set, frame)
  print 'twist: '+ str(tw)
  return kdl.addDelta(frame, tw, dd)


def compute_direction(goal_features, feature_set, frame):
  ''' returns a twist towards the goal features '''
  q_now = feature_set.coordinates(frame)
  error = [q_des - q for (q_des, q) in zip(goal_features, q_now)]

  (jac_inv, jac) = derive_features(frame, feature_set.coordinates)

  print 'jac_inv: '+str(jac_inv)
  print 'jac: '+str(jac)

  P = 0.6
  tol = 0.01

  twist = numpy.mat(numpy.zeros((6,1)))

  qdot = []

  for i,e in enumerate(error):
    # control law: P-controller with deadzone
    if abs(e) < tol:
      control_output = 0
    else:
      control_output = e*P  # TODO: WTF? why not negative ?!
    twist += jac[:,i] * control_output
    qdot.append(control_output)

  print 'q: '+str(q_now)
  print 'q_des: '+str(goal_features)
  print 'err: '+str(error)
  print 'qdot: '+str(qdot)
  return kdl.Twist(kdl.Vector(*twist[0:3]), kdl.Vector(*twist[3:6]))


def make_test_features():
  f = [None]*4
  f[0] = ToolFeature(kdl.Vector(0.2, 0.1, 0), kdl.Vector(0, 0, 1), ToolFeature.LINE)   #front
  f[1] = ToolFeature(kdl.Vector(0.15, 0.07, 0), kdl.Vector(0, 1, 0), ToolFeature.LINE) #side
  #f[1] = ToolFeature(kdl.Vector(0.15, 0.13, 0), kdl.Vector(0, 1, 0), ToolFeature.LINE) #side2
  f[2] = ToolFeature(kdl.Vector(0.15, 0.1, 0), kdl.Vector(0, 0, 1), ToolFeature.PLANE) #blade
  f[3] = ToolFeature(kdl.Vector(0, 0, 0), kdl.Vector(0, 0, 1), ToolFeature.PLANE) #workspace

  return f



import marker


# rosrun tf static_transform_publisher 0.08  0.25  0.16  0.120496839  -0.674659823005 0.030979277586 0.7262919523  /right_arm_hand_link /spatula 100

approx_tool = kdl.Frame(kdl.Rotation( 0.08 , -0.21 , -0.97 ,
                                     -0.12 ,  0.97 , -0.22 ,
                                      0.99 ,  0.13 ,  0.06 ),
                        kdl.Vector(   0.08 ,  0.25 ,  0.16 ))


class RobotMover():
  def __init__(self, robot_proxy, fs=None):
    self.rp = robot_proxy
    self.single_movement_timeout = 5
    self.table_height = 0.855
    p = kdl.Vector(-2.98, 1.835, self.table_height)
    d = kdl.Vector(0,0,1)
    self.workspace = ToolFeature(p,d, ToolFeature.PLANE)

    if(fs != None):
      self.features = fs

  def create_features(self):
    # somewhere this tool frame gets turne 90deg, oh well...

    tp = approx_tool.p
    tx = approx_tool.M.UnitX()
    ty = approx_tool.M.UnitY()
    tz = approx_tool.M.UnitZ()

    #front_blade
    p = tp + ty*0.05
    d = tx
    self.front_blade = ToolFeature(p,d, ToolFeature.LINE)

    p = tp + tx*0.04
    d = ty
    self.lower_side_blade = ToolFeature(p,d, ToolFeature.LINE)

    p = tp - tx*0.04
    d = ty
    self.upper_side_blade = ToolFeature(p,d, ToolFeature.LINE)

    # some place on the table
    p = kdl.Vector(-2.98, 1.835, self.table_height)
    d = kdl.Vector(0,0,1)
    self.workspace = ToolFeature(p,d, ToolFeature.PLANE)

    self.features = FeatureSet([self.front_blade, self.lower_side_blade], self.workspace)

  def get_task_angles(self):
    f = self.rp.arm.getCurrentFrame(5)
    return self.features.coordinates(f)


  def arm_speed(self, vel):
    self.rp.hb.set_max_vel( vel * pi / 1.8)
    self.rp.hb.set_vf_max_vel(vel)

  def touch_with_increasing_speed(self, dist=0.15, frame=None):
    if frame == None:
      frame = self.rp.arm.getCurrentFrame(5)
    for vel in [0.15, 0.20, 0.39]:
      self.arm_speed(vel)
      self.move_down_and_up(dist, frame)
      rospy.sleep(4)

  def set_relevant_feature(self, index):
      tool = kdl.Frame(self.features.tool_features[index].position)
      self.rp.arm.setTool(tool)

  def set_relevant_feature(self, index):
      tool = kdl.Frame(self.features.tool_features[index].position)
      self.rp.arm.setTool(tool)

  def move_towards_angles(self, c, dd=0.1, frame=None, iterations=1):
    self.rp.hb.cartesian_controller()
    if frame == None:
      frame = self.rp.arm.getCurrentFrame(5)

    for i in range(iterations):
      frame_new = compute_increment(c, self.features, frame, dd)


      self.rp.arm.gotoFrameBlocking(frame_new, self.single_movement_timeout)
      frame = kdl.Frame() * frame_new
    return frame_new


  def move_towards_with_angles(self, c, dd=0.1, frame=None, iterations=1):
    self.rp.hb.cartesian_controller()
    if frame == None:
      frame = self.rp.arm.getCurrentFrame(5)

    for i in range(iterations):
      frame_new = compute_increment(c, self.features, frame, dd)


      self.rp.arm.gotoFrameBlocking(frame_new, self.single_movement_timeout)
      frame = kdl.Frame() * frame_new
    return frame_new



  def move_down_and_up(self, dist=0.15, frame=None):
    if frame == None:
      frame = self.rp.arm.getCurrentFrame(5)
    frame.p[2] -= dist

    self.rp.send_state(3)
    self.rp.arm.gotoFrameBlocking(frame, 3)
    rospy.sleep(2)

    frame.p[2] += dist
    self.rp.send_state(4)
    self.rp.arm.gotoFrameBlocking(frame, 4)

  def display_features(self):
    frame = approx_tool
    for i,f in enumerate(self.features.tool_features):
      display_feature(f, frame, '/right_arm_hand_link', i)


  def complete_sequence(self):
    self.rp.hjc.setJointsBlocking(jjj[0][0], self.rp.hb, 5.0)

    # set tool to spatula front edge
    tool = kdl.Frame(kdl.Vector(   0.0769738,    0.303185,     0.18818))
    self.rp.arm.setTool(tool)

    self.rp.send_state(0)  # 'rest' calibration
    self.rp.random_movements(100, 1.0, 0.0)
    self.rp.send_state(1)  # calibration finished
    self.impact_sequence()

    for j in joints_back:
      self.rp.hjc.setJointsBlocking(j, self.rp.hb, 5.0)

    self.rp.send_state(10)  # calibration done


  def aligned_sequence(self):
    self.rp.hjc.setJointsBlocking(jjj[0][0], self.rp.hb, 5.0)

    # set tool to spatula front edge
    tool = kdl.Frame(kdl.Vector(   0.0769738,    0.303185,     0.18818))
    self.rp.arm.setTool(tool)

    self.rp.send_state(0)  # 'rest' calibration
    self.rp.random_movements(100, 1.0, 0.0)
    self.rp.send_state(1)  # calibration finished

    for i,jj in enumerate(jjjx):
      print '=======================  '+str(i)
      for j in jj:
        self.rp.hjc.setJointsBlocking(j, self.rp.hb, 5.0)
      self.rp.sleep(2)

      self.rp.hb.cartesian_controller()
      frame = self.rp.arm.getCurrentFrame(5)
      if i == 0:
        self.arm_speed(0.25)
        frame.p = frame.p + kdl.Vector(0,0,0.02)
      else:
        self.arm_speed(0.39)

      for i in range(20):
        self.move_down_and_up(0.15, frame)
        self.rp.sleep(2)

      self.rp.hb.joint_controller()
      self.arm_speed(0.20)

    for j in jx_back:
      self.rp.hjc.setJointsBlocking(j, self.rp.hb, 5.0)

    self.rp.send_state(10)  # calibration done





  def impact_sequence(self):
    for i,jj in enumerate(jjj):
      print '=======================  '+str(i)
      for j in jj:
        self.rp.hjc.setJointsBlocking(j, self.rp.hb, 5.0)
      self.rp.sleep(2)
      self.turn_and_crash()

  def turn_and_crash(self, height=0.16):
    self.rp.hb.cartesian_controller()

    frame = self.rp.arm.getCurrentFrame(5)
    for a in frange(-0.32, 0.32, 0.128):
      frame_turned = frame*kdl.Frame(kdl.Rotation.RotX(a))
      self.rp.arm.gotoFrameBlocking(frame_turned, 3)
      self.move_down_and_up(height, frame_turned)

      frame_turned = frame*kdl.Frame(kdl.Rotation.RotY(a))
      self.rp.arm.gotoFrameBlocking(frame_turned, 3)
      self.move_down_and_up(height, frame_turned)

      frame_turned = frame*kdl.Frame(kdl.Rotation.RotZ(a))
      self.rp.arm.gotoFrameBlocking(frame_turned, 3)
      self.move_down_and_up(height, frame_turned)

      self.rp.arm.gotoFrameBlocking(frame, 3)

    self.rp.hb.joint_controller()


def display_results(cop_answer, tm):
  features = cop_to_features(cop_answer, tm)
  display_features(features)

def display_features(features, frame= kdl.Frame(), frame_id="/right_arm_hand_link", offset=0):
  for i, f in enumerate(features):
    display_feature(f, frame, frame_id, i + offset)

def display_feature(feature, frame, frame_id, id=1):
  m = marker.create(type=marker.Marker.CYLINDER, id=id)

  p = frame * feature.position
  d = (frame * (feature.position + feature.direction)) - p
  s = feature.size
  marker.align(m, p-d*s, p+d*s, 0.01)
  m.header.frame_id = frame_id
  m.color = marker.ColorRGBA(0.0, 1.0, 0.0, 1.0)
  marker.publish(m)


def cop_to_wfeatures(cop_answer, rp):
  right_arm = rp.hlo.getLOid('/base_link')[0]
  features = []
  for a in cop_answer:
    (pose, cov) = rp.hlo.getFrame(a.position, right_arm, True)
    print pose
    p = kdl.Vector() + pose.p
    d = kdl.Vector() + pose.M.UnitZ()
    s = cov[14]
    features.append(ToolFeature(p,d, ToolFeature.LINE ,s))
  return features

def cop_to_features(cop_answer, rp):
  right_arm = rp.hlo.getLOid('/right_arm_hand_link')[0]
  features = []
  for a in cop_answer:
    (pose, cov) = rp.hlo.getFrame(a.position, right_arm, True)
    print pose
    p = kdl.Vector() + pose.p
    d = kdl.Vector() + pose.M.UnitZ()
    s = cov[14]
    features.append(ToolFeature(p,d, ToolFeature.LINE ,s))
  return features

def test_object_follower(object_name, rm):
  cop = rm.rp.find_obj(object_name, 1)
  wfeature = cop_to_wfeatures(cop ,  rm.rp)
  if(len(wfeature) > 0):
    wfeature[0].position[2] += 0.3
    rm.features.workspace_features = wfeature
    display_features(wfeature, frame_id="/base_link")
    display_features(rm.features.tool_features, offset=1)
    f = rm.move_towards_angles([0.0, 0.0], 0.1, None, 60)
  display_features(wfeature, frame_id="/base_link")
  display_features(rm.features.tool_features, offset=1)



# this always affects the rotation only in order to align.
# it is an adequate angle representation as long as tool center point
# is chosen reasonably close to the feature centers.

# actually the feature twists should be moved to the feature location in order to be more meaningful!
# That is:
# for column,feature in zip(jac, features):
#   kdl.changeRefPoint(column, -feature.position)
#
# let's draw debuggging output...
