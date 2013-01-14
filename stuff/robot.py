#!/usr/bin/python

# robot kinematics proxy, using urdf_parser_py, PyKDL
# and the /joint_states topic
#

# Display a manipulability ellipsoid in rviz.
# this time, use the urdf and joint_state for getting the kinematic chain
# and the current joint angles.

import roslib
roslib.load_manifest('stuff')

import rospy
import marker

import urdf_parser_py.urdf as urdf
import PyKDL as kdl

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import tf_conversions.posemath as pm


def get_joint_names(base, tip):
  model = urdf.URDF.load_from_parameter_server()
  return model.get_chain(base, tip, True, False, False)


def get_kdl_chain(base, tip):
  model = urdf.URDF.load_from_parameter_server()
  joint_names = model.get_chain(base, tip, True, False, True)

  #NOTE: The ordering of link-joint in KDL is different from URDF.
  #      Therefore we prepend a fixed joint, append a 0-Frame and re-associate
  links = []
  joints = [kdl.Joint(kdl.Joint.None)]
  for joint_name in joint_names:
    urdf_joint = model.joints[joint_name]

    ### get link transform
    rot = kdl.Rotation.RPY(*urdf_joint.origin.rotation)
    pos = kdl.Vector(*urdf_joint.origin.position)
    links.append(kdl.Frame(rot, pos))

    ### get joint
    if urdf_joint.joint_type == urdf_joint.FIXED:
      joint_type = kdl.Joint.None
      scale = 1.0
    else:
      # NOTE: having a string here is inconsistent in urdf_parser_py...
      axis = map(float, urdf_joint.axis.split())
      axis_id = None
      if abs(axis[0]) == 1.0 and axis[1] == 0.0 and axis[2] == 0.0:
        axis_id = 0
      if axis[0] == 0.0 and abs(axis[1]) == 1.0 and axis[2] == 0.0:
        axis_id = 1
      if axis[0] == 0.0 and axis[1] == 0.0 and abs(axis[2]) == 1.0:
        axis_id = 2
      scale = axis[axis_id] #This fails if not axis-aligned. Hack up PyKDL then!
 
      if (urdf_joint.joint_type == urdf_joint.REVOLUTE
          or urdf_joint.joint_type == urdf_joint.CONTINUOUS):
        joint_type = kdl.Joint.RotX + axis_id
      elif ujnt.joint_type == ujnt.PRISMATIC:
        joint_type = kdl.Joint.TransX + axis_id

    #NOTE: There is no accessor for the scale field of a KDL Joint!
    #      If we need this value later, we must remember it ourselves!
    joints.append(kdl.Joint(joint_type, scale))

  links.append(kdl.Frame())

  ### assemble chain
  chain = kdl.Chain()
  for joint, link in zip(joints, links):
    chain.addSegment(kdl.Segment(joint, link))

  return chain


class Robot:
  def __init__(self, base_frame_id, tip_frame_id):
    self.base_frame_id = base_frame_id
    self.tip_frame_id = tip_frame_id

    self.chain = get_kdl_chain(base_frame_id, tip_frame_id)
    self.kin_fwd = kdl.ChainFkSolverPos_recursive(self.chain)
    self.kin_jac = kdl.ChainJntToJacSolver(self.chain)
    self.joints = kdl.JntArray(self.chain.getNrOfJoints())
    names = get_joint_names(base_frame_id, tip_frame_id)
    self.joint_names = {}
    for i,name in enumerate(names):
      self.joint_names[name] = i

    self.joint_sub = rospy.Subscriber('/joint_states', JointState,
                                      self.joints_cb)

  def joints_cb(self, msg):
    for i in range(len(msg.name)):
      if msg.name[i] in self.joint_names:
        self.joints[self.joint_names[msg.name[i]]] = msg.position[i]

  def pose(self):
    frame = kdl.Frame()
    self.kin_fwd.JntToCart(self.joints, frame)
    return frame

  def jac(self):
    jac = kdl.Jacobian(self.chain.getNrOfJoints())
    self.kin_jac.JntToJac(self.joints, jac)
    return jac

  def debug_kinematics(self):
    frame = kdl.Frame()

    for i in range(10):
      if i == 9:
        i = -1
      self.kin_fwd.JntToCart(self.joints, frame, i)

      mrk = marker.create(type=marker.Marker.CUBE,
                          scale=Vector3(0.1, 0.1, 0.1),
                          color=ColorRGBA(1, i*0.12, 0, 1),
                          ns='jnts', id=i)
      mrk.header.frame_id = self.base_frame_id
      mrk.pose = pm.toMsg(frame)
      marker.publish(mrk)


if __name__ == '__main__':
  rospy.init_node('robot_kinematics_test')

  #disp = Robot('torso_lift_link', 'l_wrist_roll_link') # PR2
  disp = Robot('base_link', 'left_arm_7_link') # Rosie

  rate = rospy.Rate(25)
  while not rospy.is_shutdown():
    disp.debug_kinematics()


