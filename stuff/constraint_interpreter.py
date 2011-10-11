from math import *

class Range:
  def __init__(self, lo, hi):
    self.lo = lo
    self.hi = hi

  def __repr__(self):
    return '['+str(self.lo)+' .. '+str(self.hi)+']'

  def __str__(self):
    if self.hi < self.lo:
      return '___'
    else:
      return self.__repr__()

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


axis_names = {
              'angle': 0,
              'distance': 1,
              'height': 2,
              'roll': 4,
              'pitch': 5,
              'yaw': 3
             }



class ConstraintHL:
  def __init__(self, action='', obj1='', obj2=''):
    self.action = action
    self.obj1 = obj1
    self.obj2 = obj2
  def __repr__(self):
    return ("['%s', '%s', '%s']"
              % (self.action, self.obj1, self.obj2))


class ConstraintLL:
  def __init__(self, tool='', obj=''):
    self.tool   = tool
    self.obj    = obj
    self.pos    = [Range(1.0,-1.0)]*6
    self.force  = [Range(1.0,-1.0)]*6
    self.weight = [0.0]*6

  def __repr__(self):
    return ('%s -> %s' % (self.tool, self.obj)
        +  '\nP=['+' '.join(map(str, self.pos))+']'
        #+ '\nF=['+' '.join(map(str, self.force))+']'
        + '\nw=['+' '.join(map(str, self.weight))+']')

  def valid(self):
    ''' a low-level constraint is valid if all relevant joints (where weight is 1) have
        non-empty ranges'''
    return all(not (p.empty() and w != 0.0)
                for (p,w) in zip(self.pos, self.weight))

  def holds(self, angles, precision_lin=0.015, precision_ang=0.05):
    ''' checks whether all important angles are within the desired ranges '''
    precisions = [precision_ang, precision_lin, precision_lin,
                  precision_ang, precision_ang, precision_ang]
    if len(angles) != len(self.pos):
      return False
    else:
      return all(p.inside(a, pr) for p,w,a,pr in zip(self.pos, self.weight, angles, precisions) if w != 0.0 )


def change_reference_objects(c, tool, obj):
  ''' changes the reference object of the constraint.
      It looks up the original objects angles in the
      new chain and adapts the ranges accordingly.
  '''
  print "implement me"



def tweak_ref(c1, c2):
  ''' tweak the reference objects of the constraints to make them
      compatible. Hack for the demos, its possible to do better.
  '''
  if c1.obj == 'world':
    c1.obj = c2.obj
  if c2.obj == 'world':
    c2.obj = c1.obj

# straightforward so far, lets go west...

def combine(c1, c2):
  ''' assume equal reference objects for now '''
  c = ConstraintLL(c1.tool, c1.obj)

  for i in range(6):
    if c1.weight[i] == 0.0:
      c.weight[i] = c2.weight[i]
      c.pos[i] = c2.pos[i]
    elif c2.weight[i] == 0.0:
      c.weight[i] = c1.weight[i]
      c.pos[i] = c1.pos[i]
    else:
      c.weight[i] = max(c1.weight[i], c2.weight[i])
      c.pos[i] = c1.pos[i] ^ c2.pos[i]

  return c



# actions verbs and 'modifiers' are often strongly bound together.

#AT = 0
#BELOW = -1
#ABOVE = 1

AT = 'at'
ABOVE = 'above'
BELOW = 'below'


class Hint:
  COMPARISONS = {1: 'above', -1: 'below', 0: 'at', 2: 'well above', -2: 'well below'}

  def __init__(self, axis='', comparison=0):
    self.tool = ''
    self.object = ''
    self.axis = ''
    self.comparison = 0

  def __repr__(self):
    return "[tool: '%s' object: '%s' axis: '%s' comp: '%s']" % (self.tool, self.object, self.axis, uConstraint.COMPARISONS[self.comparison])




def interpret(c_hl):
  action_synonyms = {
                      'keep above': 'move over',
                      'keep over': 'move over'
                    }

  # look up synonyms
  if c_hl.action in action_synonyms:
    c_hl.action = action_synonyms[c_hl.action]

  action_hints = {
                   'point towards':   [('pitch', AT, 1.0, 0.1)],
                   'from left':       [('angle', AT, 1.57, 1.0)],
                   'push down onto':  [('height', AT, -0.06, 0)],
                   'move under':      [('height', BELOW, 0.06, 0), ('distance', BELOW, -0.03, 0.005)],
                   'move over':       [('height', ABOVE, 0.15, 0), ('distance', BELOW, -0.16, 0.005)],
                   'move next to':    [('height', AT, -0.04, 0), ('distance', AT, 0.16, 0.01)],
                   'keep horizontal': [('roll', AT, -0.25, 0.05), ('yaw', AT, 0.0, 0.02)],
#                   'keep vertical':   [('roll', AT, 0, 0), ('yaw', AT, 0, 0)],
#                   'align':           [('roll', AT, 0, 0), ('yaw', AT, 0, 0)],
                   'lift':            [('height', ABOVE, 0.20, 0)],
                   'flip':            [('roll', AT, 0, 0.15), ('yaw', AT, -0.99, 0.05)]
#                   'turn':            [('roll', AT, 0, 0.5), ('yaw', AT, 0.2*pi, 0.1)]
                 }

  # see if we can complete the second object
  world_actions = ['horizontal', 'vertical', 'lift', 'turn', 'flip']

  if any(word in world_actions for word in c_hl.action.split()):
    print 'found a world action'
    c_hl.obj2 = 'world'


  if not c_hl.action in action_hints:
    return (c_hl, None)  # sorry no match
  else:
    return (c_hl, action_hints[c_hl.action])



  




# TODO: for 'horizontal' add obj2=world
# add some completion heuristics
def complete(c_hl):
  print 'implement me'


TOOL = 1
OBJECT = 0

def extents(obj, which_end):
  # get pose of (sub-)object
  if which_end == TOOL:
    poses = {
             'spatula': [0 , 0.05 , 0, 0,0,0], ##
             'blade': [0,0.05,0, 0,0,0],
             'front_edge': [0,0,0, 0,0,0],  
             'skimmer': [0,0.05,0, 0,0,0],  ##
            }

  if which_end == OBJECT:
    poses = {
             'baker': [0,0,0, 0,0,0],  ##
             'pancake': [0,0.07,0, 0,0,0], ##
             'pot': [0,0.16,0, 0,0,0], ##
             'sausage': [0,0.4,0.2, 0,0,0], ##
             'world': [0,0,0, 0,0,0], ##
            }
  if not obj in poses:
    print '%s object not found: %s' % (['tool', 'object'][which_end], obj)

  return poses[obj]
  # SO: we assume to know the sizes.
  # how do we deal with it? assume no overlap (that would not be realistic)

  # TODO: create sizes, i.e. ranges. -> 'point inside distribution problem' 


def translate(c_hl):

  (c_hl, hints) = interpret(c_hl)

  print 'translating constraint '+str(c_hl)
  print 'hints: '+str(hints)

  c_ll = ConstraintLL()
  c_ll.tool = c_hl.obj1
  c_ll.obj  = c_hl.obj2

  o1 = extents(c_hl.obj1, TOOL)  # object attached to tool
  o2 = extents(c_hl.obj2, OBJECT)  # object attached to the world

  if hints:
    for (axis, where, offset, tol) in hints:
      axis_id = axis_names[axis]
      # turn ABOVE/BELOW/AT into a range, using o1 and o2
      # this assumes that o1 and o2 are fixed w.r.t. the chain end points
      # and ...
      # spanning a new chain would definitely work. When is this equivalent
      # to our simplified approach?
      # when dist(o1, o2, a) == dist(co1, co2, a) - dist(

      pos = o2[axis_id] - o1[axis_id]

      rr = None

      if where == AT:
        rr = Range(pos + offset - tol, pos + offset + tol)
      elif where == BELOW:
        rr = Range(float('-inf'), pos - offset + tol)
      elif where == ABOVE:
        rr = Range(pos + offset - tol, float('inf'))

      if rr != None:
        c_ll.pos[axis_id] = rr
        c_ll.weight[axis_id] = 1.0
  return c_ll

def test():
  c_hl = ConstraintHL()
  c_hl.type = 'move'
  c_hl.obj1 = 'spatula'
  c_hl.obj2 = 'pancake'
  c_hl.direction = 'under'

  c_ll = translate(c_hl)
