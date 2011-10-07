#!/usr/bin/python

import roslib ; roslib.load_manifest('motionControl')

from PyKDL import *
from math import pi
import PyKDL as kdl


# not defined in PyKDL
def equal(j1, j2):
  eps = 0.000001
  for j in range(j1.rows()):
    for i in range(j1.columns()):
      if abs(j1[i,j] - j2[i,j]) > eps:
        return False
  return True


# for the old, broken printing functions of PyKDL
def printjac(jac):
  for j in range(jac.rows()):
    s = ''
    for i in range(jac.columns()):
      s += str(jac[i,j])+'   '
    print s
 
arm_segments = [
        Segment(Joint(Joint.None),
            Frame(Rotation.RPY(0.71178, 0.85723, -0.71169),Vector(0.385121, -0.0570121, 1.15408)  )),
        Segment(Joint(Joint.None),
            Frame(Rotation.Identity(), Vector(0.0, 0.0, 0.11))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(-pi/2), Vector(0.0, 0.0, 0.20))),
        Segment(Joint(Joint.RotZ, -1),
            Frame(Rotation.RotX(pi/2), Vector(0.0, -0.20, 0.0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(pi/2), Vector(0, 0, .20))),
        Segment(Joint(Joint.RotZ, -1),
            Frame(Rotation.RotX(-pi/2), Vector(0, 0.2, 0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(-pi/2), Vector(0, 0, 0.19))),
        Segment(Joint(Joint.RotZ, -1),
            Frame(Rotation.RotX(pi/2), Vector(0, -0.078, 0.0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotZ(3*pi/4)*Rotation.RotX(pi/2)*Rotation.RotY(pi), Vector(-0.075, -0.075, -0.094))),
#        Segment(Joint(Joint.None),
#            Frame(Rotation.Identity(), Vector(0.07, -0.025, 0.28))),
            ]

# create some random joints
import random
rand = random.Random()
joints = JntArray(7)

for i in range(7):
  joints[i] = rand.uniform(-pi, pi)


# create the chains

frame = kdl.Frame(Rotation.Rot(Vector(0.3, -0.2, 0.5), 0.7), Vector(0.1, 0.2, 0.3))

chain  = kdl.Chain()
chain2 = kdl.Chain()

for s in arm_segments:
  chain.addSegment(s)
  chain2.addSegment(s)

chain2.addSegment(Segment(Joint(Joint.None), frame))

# forward kinematics

fk = ChainFkSolverPos_recursive(chain)
fk2 = ChainFkSolverPos_recursive(chain2)

f = Frame()
f2 = Frame()

fk.JntToCart(joints, f)
fk2.JntToCart(joints, f2)

##################################
assert(kdl.Equal(f2, f*frame))
##################################

# differential kinematics

jac  = ChainJntToJacSolver(chain)
jac2 = ChainJntToJacSolver(chain2)

j   = Jacobian(7)
j2  = Jacobian(7)
j2b = Jacobian(7)
jx  = Jacobian(7)
jxb  = Jacobian(7)

kdl.SetToZero(j)
kdl.SetToZero(j2)
kdl.SetToZero(jx)


jac.JntToJac(joints, j)
jac2.JntToJac(joints, j2)

#########################
fr2 = f*frame
kdl.changeRefPoint(j, fr2.p - f.p, jx)
assert(equal(j2, jx))
# transform to base frame
kdl.changeRefPoint(j2, -f2.p, j2b)
kdl.changeRefPoint(j, -f.p, jxb)

assert(equal(j2b, jxb))
#########################

