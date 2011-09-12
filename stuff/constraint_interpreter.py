
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
      return __repr__(self)

  def __xor__(self, other):
    ''' intersection of two ranges '''
    return Range(max(self.lo, other.lo), min(self.hi, other.hi))

  def empty(self):
    return self.lo > self.hi


axis_names = {
              'angle': 0,
              'distance': 1,
              'height': 2,
              'roll': 3,
              'pitch': 4,
              'yaw': 5
             }



class ConstraintHL:
  def __init__(self):
    self.type = ''
    self.obj1 = ''
    self.obj2 = ''
    self.direction = ''
  def __repr__(self):
    return ("[type: '%s' obj1: '%s' obj2: '%s' direction: '%s']"
              % (self.type, self.obj1, self.obj2, self.direction))


class ConstraintLL:
  def __init__(self):
    self.tool   = ''
    self.object = ''
    self.pos    = [Range(1.0,-1.0)]*6
    self.force  = [Range(1.0,-1.0)]*6
    self.weight = [0.0]*6

  def __repr__(self):
    return ('P=['+' '.join(map(str, self.pos))+']'
        + '\nF=['+' '.join(map(str, self.force))+']'
        + '\nw=['+' '.join(map(str, self.weight))+']')

  def valid(self):
    ''' a low-level constraint is valid if all relevant joints (where weight is 1) have
        non-empty ranges'''
    reduce(bool.__and__,
      [not (p.empty() and w != 0.0)
        for (p,w) in zip(self.pos, self.weight)],
      True)

# straightforward so far, lets go west...

POSITION=1
FORCE=2

action_types = {
                'push': 2,
                'move': 1,
                'align': 1
               }
BELOW = -1
ABOVE = 1
AT = 0

direction_hints = {
                   'down onto':  [('height', AT)],
                   'under':      [('height', BELOW), ('distance', BELOW)],
                   'above':      [('height', ABOVE), ('distance', BELOW)],
                   'over':       [('height', ABOVE), ('distance', BELOW)],
                   'horizontal': [('roll', AT), ('pitch', AT)],
                   'next to':    [('height', AT), ('distance', AT)],
                   'align':      [('roll', AT), ('pitch', AT)]  # 0
                  }


# TODO: for 'horizontal' add obj2=world
# add some completion heuristics
def complete(c_hl):
  print 'implement me'


TOOL = 1
OBJECT = 0

def extents(obj, which_end):
  print 'implement me'
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
            }
  return poses[obj]
  # SO: we assume to know the sizes.
  # how do we deal with it? assume no overlap (that would not be realistic)

  # TODO: create sizes, i.e. ranges. -> 'point inside distribution problem' 


def translate(c_hl):
  print 'translating constraint\n'+str(c_hl)
  c_ll = ConstraintLL()

  o1 = extents(c_hl.obj1, TOOL)  # object attached to tool
  o2 = extents(c_hl.obj2, OBJECT)  # object attached to the world

  if c_hl.direction in direction_hints:
    dir_hints = direction_hints[c_hl.direction]
    print 'found hint: '+str(dir_hints)
    for (axis, where) in dir_hints:
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
        rr = Range(pos, pos)
      elif where == BELOW:
        rr = Range(float('-inf'), pos)
      elif where == ABOVE:
        rr = Range(pos, float('inf'))

      if rr != None:
        c_ll.pos[axis_id] = rr
        c_ll.weight[axis_id] = 1.0
        

def test():
  c_hl = ConstraintHL()
  c_hl.type = 'move'
  c_hl.obj1 = 'spatula'
  c_hl.obj2 = 'pancake'
  c_hl.direction = 'under'

  c_ll = translate(c_hl)
