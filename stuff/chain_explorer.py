#!/usr/bin/python

"""
Explore how many virtual kinematic chains cover all six dofs of
rigid transformations. These chains only consist of

RotX=0, RotY=1, RotZ=2,
TransX=3, TransY=4, TransZ=5

A 'typelist' is a list of the above numeric types, an 'index'
is this list packed into an integer, so all 6**6 possible chains
can be enumerated.

"""

import roslib ; roslib.load_manifest('motionControl')

from math import pi
import PyKDL as kdl
import numpy
from numpy.linalg import svd,det


# not defined in PyKDL
def equal(j1, j2):
  """Compare two JointArrays."""
  eps = 0.000001
  for j in range(j1.rows()):
    for i in range(j1.columns()):
      if abs(j1[i,j] - j2[i,j]) > eps:
        return False
  return True


def random_joints(num):
  """Create random joint values."""
  joints = kdl.JntArray(num)
  for i in range(num):
    joints[i] = random_joints.rand.uniform(-pi, pi)
  return joints

import random
random_joints.rand = random.Random()

MAX_ITER = 1000


def jacobian_rank(jac, tol=1e-8, first_row=0, last_row=5):
  """Compute the matrix rank of a KDL Jacobian.

  The parameters first_row and last_row are used to select only the position
  or orientation part.
  """
  A = numpy.matrix([[jac[i,j] for j in range(jac.columns())] for i in range(first_row, last_row + 1)])
  s = svd(A, compute_uv=0)
  return numpy.sum( numpy.where( s > tol, 1, 0 ) )


def typelist_to_index(tlist):
  """Convert a typelist to an integer."""
  index = 0
  for entry in tlist:
    index += entry
    index *= 6
  return index


def index_to_typelist(index, num_joints=6):
  """Convert an index into a typelist."""
  typelist = []
  for i in range(num_joints):
    typelist.append(index % 6)
    index = index / 6
  return typelist


def chain_to_typelist(chain):
  """Convert a KDL chain back into a typelist."""
  joint_types = [kdl.Joint.RotX, kdl.Joint.RotY, kdl.Joint.RotZ,
                 kdl.Joint.TransX, kdl.Joint.TransY, kdl.Joint.TransZ]

  typelist = []
  for i in range(chain.getNrOfSegments()):
    typelist.append(joint_types.index(chain.getSegment(i).getJoint().getType()))

  return typelist


def create_chain(typelist):
  """Create a chain from a typelist."""
  joint_types = [kdl.Joint.RotX, kdl.Joint.RotY, kdl.Joint.RotZ,
                 kdl.Joint.TransX, kdl.Joint.TransY, kdl.Joint.TransZ]
  segments = [kdl.Segment(kdl.Joint(joint_types[t]), kdl.Frame()) for t in typelist]
  chain = kdl.Chain()
  for s in segments:
    chain.addSegment(s)
  return chain


def chain_str(chain):
  """Pretty-print a virtual kinematic chain."""
  types = []
  for i in range(chain.getNrOfSegments()):
    types.append(str(chain.getSegment(i).getJoint()))
  return '[' + ', '.join(types) + ']'


def max_rank(chain, min_row=0, max_row=5):
  """Compute the maximum number of (cartesian) DOFs of the chain.

  The parameters first_row and last_row are used to select only the position
  or orientation part.
  """
  num_joints = chain.getNrOfJoints()
  solver_jac = kdl.ChainJntToJacSolver(chain)
  r = 0
  for iter in range(MAX_ITER):
    jnts = random_joints(num_joints)
    jac = kdl.Jacobian(num_joints)
    solver_jac.JntToJac(jnts, jac)
    r = max(r, jacobian_rank(jac, 1e-8, min_row, max_row))
  return r 


def position_rank(chain):
  """How many of the 3 position DOFs does the chain cover?"""
  return max_rank(chain, 0, 2)

def orientation_rank(chain):
  """How many of the 3 orientation DOFs does the chain cover?"""
  return max_rank(chain, 3, 5)


def position_orientation_chain(chain):
  """ Can the chain be separated into position and orientation part?

  Check if the first 3 joints cover the position DOFs, and the last 3 joints
  cover orientations (or vice versa).
  """

  if chain.getNrOfJoints() != 6:
    return False

  # split into two chains
  chain1 = kdl.Chain()
  for i in range(0, 3):
    chain1.addSegment(chain.getSegment(i))
  # ...
  chain2 = kdl.Chain()
  for i in range(3, 6):
    chain2.addSegment(chain.getSegment(i))

  # first chain covers positions, second covers orientation
  if position_rank(chain1) == 3 and orientation_rank(chain2) == 3:
    return True

  # vice versa
  if orientation_rank(chain1) == 3 and position_rank(chain2) == 3:
    return True

  # not that easy...
  return False



def equivalent_under_permutation(chain1, chain2):
  """Check if the chains are equivalent under any permutation of their joint angles."""
  if chain1.getNrOfJoints() != chain2.getNrOfJoints():
    return False

  units1 = [t<3 for t in chain_to_typelist(chain1)]
  units2 = [t<3 for t in chain_to_typelist(chain2)]

  if units1.count(True) != units2.count(True):
    return False

  num_joints = chain1.getNrOfJoints()

  import itertools
  for perm in itertools.permutations(range(num_joints)):
    if units1 != [units2[perm[i]] for i in range(num_joints)]:
      continue

    if equivalent(chain1, chain2, perm):
      return True

  return False


def equivalent(chain1, chain2, permutation=[0, 1, 2, 3, 4, 5]):
  """Compute if the chains are kinematically equivalent.

  Permute the joint angle of chain2 with 'permutation'.
  """
  if chain1.getNrOfJoints() != chain2.getNrOfJoints():
    return False

  num_joints = chain1.getNrOfJoints()

  solver_fk1 = kdl.ChainFkSolverPos_recursive(chain1)
  solver_fk2 = kdl.ChainFkSolverPos_recursive(chain2)
  f1 = kdl.Frame()
  f2 = kdl.Frame()
  for iter in range(MAX_ITER):
    jnts1  = random_joints(num_joints)
    jnts2 = kdl.JntArray(num_joints)
    for i in range(num_joints):
      jnts2[i] = jnts1[permutation[i]]
    pose1 = solver_fk1.JntToCart(jnts1, f1)
    pose2 = solver_fk2.JntToCart(jnts2, f2)
    if not kdl.Equal(f1, f2, 0.0001):
      return False
  return True


def count_full_rank_chains():
  """Count how many of the 6**6 possible chains have full rank."""
  count = 0
  indices = []
  for index in range(6**6):
    chain = create_chain(index_to_typelist(index))
    if max_rank(chain) == 6:
      count += 1
      indices.append(index)
    if (index % 100) == 0:
      print '## progress: '+str(index)
  return (count, indices)


def count_pos_ori_chains(chains):
  """Count how many chains are separable into a position and an orientation part."""
  count = 0
  indices = []
  for ii, index in enumerate(chains):
    chain = create_chain(index_to_typelist(index))
    if position_orientation_chain(chain):
      count += 1
      indices.append(index)
    if (ii % 100) == 0:
      print '## progress: %d/%d' % (ii, len(chains))
  return (count, indices)


def eliminate_equivalents(chains):
  """Remove all equivalent chains from chain index list.

  By permuting some of the joint angles, some of the chains become kinematically
  equivalent. Remove thos duplicates from the given index list and return the cleaned
  list.
  """
  cleaned_chains = list(chains)

  for i1 in range(len(chains)):
    print '## progress: %d/%d' % (i1, len(chains))
    for i2 in range(i1 + 1, len(chains)):
      if not chains[i1] in cleaned_chains:
        continue	
      if not chains[i2] in cleaned_chains:
        continue

      chain1 = create_chain(index_to_typelist(chains[i1]))
      chain2 = create_chain(index_to_typelist(chains[i2]))
      if equivalent_under_permutation(chain1, chain2):
        cleaned_chains.remove(chains[i2])
        print '## XXXXXXX'

  return cleaned_chains


def joint_type_statistics(chains):
  """Make histogram about the chain indices in 'chains'.

  The histogram entry h[i] contains the number of chains which have i prismatic
  joints.
  """
  num_trans_joints = [0]*7
  for index in chains:
    num_trans = len([t for t in index_to_typelist(index) if t > 2])
    num_trans_joints[num_trans] += 1
  return num_trans_joints


if __name__ == "__main__":
  import chain_results
  print 'full rank chains'
  print joint_type_statistics(chain_results.fullrank)
  print 'unique chains'
  print joint_type_statistics(chain_results.unique)
  print 'separable chains'
  print joint_type_statistics(chain_results.separable)
  print 'unique separable chains'
  print joint_type_statistics(chain_results.unique_separable)

