#!/usr/bin/python

class Range:
  def __init__(self, lo, hi):
    self.lo = lo
    self.hi = hi

  def __repr__(self):
    return 'Range('+str(self.lo)+',  '+str(self.hi)+')'

  def __str__(self):
    if self.hi < self.lo:
      return '___'
    else:
      return '['+str(self.lo)+' .. '+str(self.hi)+']'

  def __xor__(self, other):
    ''' intersection of two ranges '''
    return Range(max(self.lo, other.lo), min(self.hi, other.hi))

  def inside(self, val, precision=0.0):
    return val > self.lo - precision and val < self.hi + precision

  def empty(self):
    return self.lo > self.hi

  def relax(self, margin):
    self.lo -= margin
    self.hi += margin


class Constraint:
  def __init__(self, pos=None, weight=None):
    if pos == None:
      self.pos    = [Range(1.0,-1.0)]*6
    elif type(pos) is list:
      self.pos = [Range(lo,hi) for lo,hi in pos]
    else:
      self.pos = pos

    if weight == None:
      self.weight = [0.0]*6
    else:
      self.weight = weight

  def __repr__(self):
    return ( 'Constraint(pos=['
             +(', '.join(['[%.3f,%.3f]'%(r.lo,r.hi) for r in self.pos]))
             +'],\n           weight=['
             +(', '.join(map(str, self.weight)) +'])' ) )

  def __str__(self):
    return ('\nP=['+' '.join(map(str, self.pos))+']'
          + '\nw=['+' '.join(map(str, self.weight))+']')

  def valid(self):
    ''' a low-level constraint is valid if all relevant joints (where weight is 1) have
        non-empty ranges'''
    return all(not (p.empty() and w != 0.0)
                for (p,w) in zip(self.pos, self.weight))

  def holds(self, angles, precision_lin=0.015, precision_ang=0.05):
    ''' checks whether all important angles are within the desired ranges '''
    # THIS IS A HACK! could possibly be extracted from jacobian.
    precisions = [precision_ang, precision_lin, precision_lin,
                  precision_ang, precision_ang, precision_ang]

    if len(angles) != len(self.pos):
      return False
    else:
      return all(p.inside(a, pr) for p,w,a,pr in zip(self.pos, self.weight, angles, precisions) if w != 0.0 )

