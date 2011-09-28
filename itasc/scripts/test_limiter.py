from math import *

def limit(lo, hi, q):
  d=0.7
  h=0.8

  a = h / (d*d)

  qlo = lo + d
  qhi = hi - d

  if q < qlo and q < qhi:
    qdot = - (a * (qlo - q)*(qlo - q))
  elif q > qhi and q > qlo:
    qdot =   (a * (q - qhi)*(q - qhi))
  else:
    qdot = 0

  return qdot

def frange(start, stop, step):
  return (start + step*i for i in range(int(round((stop - start) / step))))


def test():
  lim_lo = -170.0/180.0*pi
  lim_hi =  170.0/180.0*pi

  f = open('/tmp/data.log', 'w')
  for q in frange(lim_lo, lim_hi, 0.002):
    qdot = limit(lim_lo, lim_hi, q)
    f.write("%f %f\n" % (q, qdot))

  f.close()

if __name__ == "__main__":
  test()
