
import roslib
roslib.load_manifest('itasc')
import rospy

import PyKDL as kdl

from tf_conversions import posemath as pm
from geometry_msgs.msg import Pose
from constraint_msgs.msg import ConstraintCommand, ConstraintState

from itasc_helpers import *

class Itasc(object):
    """ Class to handle the iTaSC controller from python
    """
    def __init__(self, side='right'):
        self.current_constraint = None

        if side != '':
            prefix = '/' + side
        else:
            prefix = ''

        self.pub_cmd = rospy.Publisher(prefix+'/command', ConstraintCommand)
        self.sub_state = rospy.Subscriber(prefix+'/state', ConstraintState, self._state_callback, queue_size = 1, tcp_nodelay = True)

        self.chi = []
        self.chi_desired = []

        self.desired_hi = []
        self.desired_lo = []
        self.weights_desired = []

        self.tool_frame   = kdl.Frame() # TODO: use between-fingers default frame
        self.object_frame = kdl.Frame() # TODO: better default

        self.pub_tool   = rospy.Publisher(prefix+'/tool_pose',   Pose)
        self.pub_object = rospy.Publisher(prefix+'/object_pose', Pose)

    def setTool(self, frame):
        """ set the tool frame for the iTaSC controller """
        self.tool_frame = frame
        msg = pm.toMsg(frame)
        self.pub_tool.publish(msg)


    def setObject(self, frame):
        """ set the object frame for the iTaSC controller """
        self.object_frame = frame
        msg = pm.toMsg(frame)
        self.pub_object.publish(msg)


    def _sendCommand(self):
        """ sends the commanded values to itasc """
        msg = ConstraintCommand()
        msg.pos_hi = self.desired_hi
        msg.pos_lo = self.desired_lo
        msg.max_vel =  [0.2]*len(self.desired_hi)
        msg.min_vel = [-0.2]*len(self.desired_hi)
        msg.weight = self.desired_weight
        self.pub_cmd.publish(msg)


    def setDesired(self, chi_desired, tolerance=0.01):
        """ sends the arm to the specified joint angles """
        self.desired_hi = [v + tolerance for v in chi_desired]
        self.desired_lo = [v - tolerance for v in chi_desired]
        self._sendCommand()


    def setWeights(self, weights_desired):
        """ sets the importance for the desired task angles """
        self.desired_weight = weights_desired
        self._sendCommand()


    def getAngles(self):
        """ get the current angles of the virtual linkage """
        return self.chi


    def stop(self):
        """ stops the robot by setting desired pose to current pose. """
        self.desired_weight = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        self.setDesired(self.chi)


    def happy(self, tolerance=0.05):
        """ at desired position? """
        if self.current_constraint:
            return self.current_constraint.holds(self.chi)


    def _state_callback(self, msg):
        """ callback method for current task angles and low-level weights. """
        self.chi = msg.chi
        self.chi_desired = msg.chi_desired
        self.weightsll = msg.weights
        if len(self.desired_hi) == 0:
          self.desired_hi = list(self.chi_desired)
          self.desired_lo = list(self.chi_desired)
          self.desired_weight = [1.0]*len(self.chi_desired)


    def setConstraint(self, c):
        """ sets a constraint (consisting of ranges and weights) """
        self.current_constraint = c

        self.desired_lo = []
        self.desired_hi = []
        self.desired_weight = []

        for i in range(6):
          self.desired_lo.append(c.pos[i].lo)
          self.desired_hi.append(c.pos[i].hi)
          self.desired_weight.append(c.weight[i])

        self._sendCommand()

