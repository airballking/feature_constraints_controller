#!/usr/bin/python

# send marker messages to visualize a KDL chain

# (specialized for pancake baking for now)

from PyKDL import *

chain_baker = Chain()
chain_baker.addSegment( Segment(Joint(Joint.RotZ))   )
chain_baker.addSegment( Segment(Joint(Joint.TransZ)) )
chain_baker.addSegment( Segment(Joint(Joint.TransX)) )


segments_spatula = [
 Segment(Joint(Joint.RotZ)),
 Segment(Joint(Joint.RotY)),
 Segment(Joint(Joint.RotX))
]

chain_spatula = Chain()
for segment in segments_spatula:
  chain_spatula.addSegment(segment)

FIXED = 0
ROT = 1
TRANS = 2

types = {Joint.None: FIXED,
         Joint.RotX: ROT, Joint.RotY: ROT, Joint.RotZ: ROT,
         Joint.TransX: TRANS, Joint.TransY: TRANS, Joint.TransZ: TRANS}

axes = {Joint.None:   [0,0,0],
        Joint.RotX:   [1,0,0], Joint.RotY:   [0,1,0], Joint.RotZ:   [0,0,1],
        Joint.TransX: [1,0,0], Joint.TransY: [0,1,0], Joint.TransZ: [0,0,1]}


class ChainDrawer:
  def __init__(self, chain):
    self.chain = chain
    self.fk_solver = ChainFkSolverPos_recursive(self.chain)
  def set_angles(self, angles):
    self.angles = angles
  def get_markers(self):
    markers = []

    f = Frame.Identity()
    j = 0
    for i in range(self.chain.getNrOfSegments()):
      segment = self.chain.getSegment(i)
      t = segment.getJoint().getType()
      type = types[t]
      axis = axes[t]

      if type == ROT:
        markers.append(self.rot_marker(f, axis))
      if type == TRANS:
        markers.append(self.trans_marker(f, axis))
      if type == FIXED:
        markers.append(self.fixed_marker(f))

      f_next = f * segment.pose(self.angles[j])

      markers.append(self.link_marker(f.p, f_next.p))

      if type != FIXED:
        j = j + 1  # used one joint angle, proceed with next one

    return markers

  def rot_marker(self, f, axis):
    return 'rot'
  def trans_marker(self, f, axis):
    return 'trans'
  def fixed_marker(self, f):
    return 'fixed'
  def link_marker(self, frm, to):
    return 'link'
  

# main #

drawer = ChainDrawer(chain_baker)
drawer.set_angles([0,0,0,0,0,0])
print drawer.get_markers()
