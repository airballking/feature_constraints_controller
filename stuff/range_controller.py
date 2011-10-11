import roslib
roslib.load_manifest('motionControl')
import rospy

from std_msgs.msg import Float64MultiArray

positions = None
weights = None



def chi_feedback(msg):
  if positions == None or weights == None:
    return

  chi = msg.data
  chidot = [control(r, w, v) for (r, w, v) in zip(positions, weights, chi)]
  chi_des = [c+cdot[0] for (c,cdot) in zip(chi, chidot)]

  msg_des = Float64MultiArray()
  msg_des.data = [c+cdot[0] for (c,cdot) in zip(chi, chidot)]
  chi_des_pub.publish(msg_des)

  msg_weight_des = Float64MultiArray()
  msg_weight_des.data = [cdot[1] for cdot in chidot]
  weight_des_pub.publish(msg_weight_des)
  

class Range:
  def __init__(self, lo, hi):
    self.lo = lo
    self.hi = hi

  def __repr__(self):
    return '['+str(self.lo)+' .. '+str(self.hi)+']'

  def __xor__(self, other):
    ''' intersection of two ranges '''
    return Range(max(self.lo, other.lo), min(self.hi, other.hi))

  def empty(self):
    return self.lo > self.hi


def control(crange, w, value):
  if w == 0:
    return (0, 0)

  K_p = 0.6 # p-controller gain
  s = 0.1   # how far "inside" the range should the controllers influence reach

  if crange.hi - crange.lo < 2.0*s:
    ss = (crange.hi - crange.lo) / 2.0
  else:
    ss = s

  if value > crange.hi - ss:
    qdot = K_p * (crange.hi - ss - value )
  elif value < crange.lo + ss:
    qdot = K_p * (crange.lo + ss - value )
  else:
    qdot = 0

  if value > crange.hi or value < crange.lo:
    weight = 1.0
  elif value < crange.lo:
    weight = 1.0
  else:
    weight = max( max(0, (1.0/s)*(-crange.hi + value)+1.0),
                  max(0, (1.0/s)*( crange.lo - value)+1.0) )


  return (qdot, weight)


def frange(start, stop, step):
     return (start + step*i for i in range(int(round((stop - start) / step))))

def test_control(r, start, stop, step):
  f = open('/tmp/data.log', 'w')
  for x in frange(start, stop, step):
    (qdot, w) = control(r, 1, x)
    if x > r.lo and x < r.hi:
      inside = 0
    else:
      inside = 1.45
    f.write("%f %f %f %f\n" % (x, qdot, w, inside))
  f.close()

def test():
  r = Range(0.0, 1.0)
  test_control(r, -1.0, 2.0, 0.005)
  import os
  os.system("""gnuplot -persist -e \\
       "plot '/tmp/data.log' using 1:2 with lines title 'control output', \\
             '/tmp/data.log' using 1:3 with lines title 'control weight', \\
             '/tmp/data.log' using 1:4 with lines title 'allowed range'" """)

## main ##

#rospy.init_node('range_control')
#chi_sub = rospy.Subscriber('/chi_f', Float64MultiArray, chi_feedback)
#chi_des_pub = rospy.Publisher('/chi_f_command', Float64MultiArray)
#weight_des_pub = rospy.Publisher('/weight_command', Float64MultiArray)
