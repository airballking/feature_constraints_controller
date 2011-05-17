

filename = 'arm_segment_a.obj'

f = open(filename)


from math import *

eps = 0.00001

class Vertex:
  def __init__(self, x,y,z):
    self.x = float(x)
    self.y = float(y)
    self.z = float(z)
  def __eq__(self, other):
    return fabs(self.x - other.x) < eps and fabs(self.y - other.y) < eps and fabs(self.z - other.z) < eps
  def __lt__(self, other):
    if self.x < other.x + eps:
      return True
    elif fabs(self.x - other.x) < eps and self.y < other.y + eps:
      return True
    elif fabs(self.x - other.x) < eps and fabs(self.y - other.y) < eps and self.z < other.z + eps:
      return True
    else:
      return False

  def __str__(self):
    return '%.3f %.3f %.3f' % (self.x, self.y, self.z)
  def __repr__(self):
    return '<'+str(self)+'>'

class Face:
  def __init__(self, vertices):
    self.vertices = vertices
  def __str__(self):
    return ', '.join(map(str, self.vertices))
  def __repr__(self):
    return '<' + str(self) + '>'

vertices = []
faces = []

for line in f:
  if line[0] == '#':
    continue
  if line[0:2] == 'v ':
    # got a vertex
    vertices.append(Vertex(*(line[2:].split())))
  if line[0:2] == 'f ':
    faces.append(Face([int(x.split('/')[0]) for x in line[2:].split(' ')]))
