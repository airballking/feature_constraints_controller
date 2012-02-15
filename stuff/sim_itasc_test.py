#!/usr/bin/python

import roslib ; roslib.load_manifest('motionControl')
import rospy

from std_msgs.msg import Int8

#import thing_tools

import PyKDL as kdl
import time

import constraint_interpreter as ci

import random


pancake_pr2   = kdl.Frame(kdl.Vector(0.700, 0.000, 0.602))
pancake_rosie = kdl.Frame(kdl.Vector(1.150, 0.000, 0.763))
pancake_frame = pancake_rosie
resetter = 'reset_arm.sh'
#resetter = 'reset_pr2.sh'

class Tester:
  def __init__(self, itasc):
        #self.pub_state = rospy.Publisher('/state', Int8)
        #self.itasc = thing_tools.handleItasc.HandleItasc(side=arm_side)
        self.itasc = itasc
        self.frames = {}
        self.set_frames()

  def set_frames(self):
        self.frames['spatula'] = kdl.Frame(
          kdl.Rotation.Quaternion(-0.092, 0.038, -0.381, 0.919),
          kdl.Vector(0.124, -0.266, 0.126))
        self.frames['baker'] = pancake_frame
        self.frames['pancake'] = pancake_frame
        self.frames['world'] = pancake_frame


  def generate_random_object_pose(self):
    rand = random.Random()

    vec_off = kdl.Vector(0.1, 0.0, 0.15)
    sigmas = [0.40, 0.40, 0.30]

    v = kdl.Vector(rand.uniform(-sigmas[0]/2.0, sigmas[0]/2.0),
                   rand.uniform(-sigmas[1]/2.0, sigmas[1]/2.0),
                   rand.uniform(-sigmas[2]/2.0, sigmas[2]/2.0))

    vec0 = pancake_frame.p

    self.sigmas = sigmas
    self.offsets = vec_off

    return kdl.Frame(vec0 + vec_off + v)

  def generate_random_object_poses(self, num=30):
    return [self.generate_random_object_pose() for i in range(num)]

  def set_object_pose(self, pose):
    self.frames['baker'] = pose
    self.frames['pancake'] = pose
    self.frames['world'] = pose


  def flipping_constraints(self):
    cp = ci.ConstraintHL('point towards', 'spatula', 'baker')
    cl = ci.ConstraintHL('from left', 'spatula', 'baker')
    ch = ci.ConstraintHL('keep horizontal', 'spatula')
    c1 = ci.ConstraintHL('move next to', 'spatula', 'pancake')
    c2 = ci.ConstraintHL('move under', 'spatula', 'pancake')
    c3 = ci.ConstraintHL('lift', 'spatula')
    co = ci.ConstraintHL('keep over', 'spatula', 'baker')
    c4 = ci.ConstraintHL('flip', 'spatula')

    c_end = ci.ConstraintLL('spatula', 'baker')
    #c_end.weight = [1.0, 1.0, 1.0, 1.0, 0.0, 1.0]
    #c_end.pos = [ci.Range(2.0, 2.7), ci.Range(0.05, 0.09), ci.Range(0.15, 0.45),
    #             ci.Range(-0.3, -0.2), ci.Range(1,-1), ci.Range(0.8, 1.0)]
    c_end.weight = [1.0, 1.0, 1.0, 1.0, 0.0, 1.0]
    c_end.pos = [ci.Range(2.0, 2.7), ci.Range(0.05, 0.09), ci.Range(0.15, 0.45),
                 ci.Range(-0.3, -0.2), ci.Range(1,-1), ci.Range(0.8, 1.0)]


    self.constraints = [[cp, ch, c1],
                        [cp, ch, c2],
                        [ch, co, c3],
                        [co, c4]]
    self.combined = translate_constraints(self.constraints)
    #self.combined[3] = c_end


  def reset(self):
    # run reset.sh in background
    import os
    os.system("$(rospack find stuff)/%s &" % resetter)
    time.sleep(0.5)
    for i in range(3):
      self.itasc.stop()
      time.sleep(0.3)

    # stop reset.sh
    for i in range(3):
      os.system("$(rospack find stuff)/kill_arm_reset.sh")
      time.sleep(0.5)  


  def random_trials_legacy(self, poses):
    #step1 = (1.6102605788675517, 0.17778159639651459, 0.0048724689030507085, -0.058302231368426514, 0.15910610854933, 0.06487486424616613)
    #step2 = (1.6178358563991464, 0.0980368330740086, -0.024395814494066914, -0.026980034870167452, 0.16101051982182205, 0.0625170202461505)
    #step3 = (0.6170461797905764, 0.13049826051589195, 0.2663126403779129, 0.08633443812516985, -0.8377855083785842, -0.07898206817005622)
    #step4 = (2.037086621493434, 0.07785705673287963, 0.18738751124550768, -0.243332800067817, 0.6127114433769019, 0.7951519294498621)
    step1 = (1.3112464089307285, 0.18534746742834488, 0.00986797965369357, -0.05653911834112353, -0.11207989488623388, 0.06083858302855105)
    step2 = (1.5170083324885608, 0.10424247349148634, -0.02114847615460813, 0.02072150453619373, 0.15926757470824818, 0.06704912668197137)
    step3 = (0.6757792966352902, 0.1549852564912317, 0.3120552360678561, 0.05397017718634451, -0.5120916081440464, -0.05656334093037339)
    step4 = (2.2030526002995527, 0.07295821219406826, 0.18960181514564411, -0.24820413216108117, 1.3063609734133752, 0.7904152829495947)



    successes = []
    for i,pose in enumerate(poses):
      print '############### TRIAL ' + str(i)
      self.set_object_pose(pose)
      self.itasc.setObject(self.frames['baker'])
      self.reset()
      ok = execute_legacy([step1, step2, step3, step4], self.itasc, self.frames)

      self.store(' '.join(map(str, map(int, ok))))

      successes.append(ok)
      self.reset()

    self.successes = successes
    return successes


  def random_trials_itasc(self, poses):
    self.flipping_constraints()
    successes = []
    for i,pose in enumerate(poses):
      print '############### TRIAL ' + str(i)
      self.set_object_pose(pose)
      self.itasc.setTool(self.frames['spatula'])
      self.itasc.setObject(self.frames['baker'])
      self.reset()
      ok = execute_constraints(self.combined, self.itasc, self.frames)

      self.store(' '.join(map(str, map(int, ok))))

      successes.append(ok)
      self.reset()

    self.successes = successes
    return successes

  def store(self, s):
    f = open('/tmp/current_run', 'a')
    f.write(str(s)+'\n')
    f.close()


  def pancake_flipping(self):
        self.flipping_constraints()
        print self.combined
        execute_constraints(self.combined, self.itasc, self.frames)

  def flipping_test(self):
    ''' test the task from random poses '''


def translate_constraints(constraints):
  constraints_ll = []
  for group in constraints:
    group_ll = []
    for constraint in group:
      group_ll.append(ci.translate(constraint))
    constraints_ll.append(group_ll)

  combined = []
  for group in constraints_ll:
    cc = ci.ConstraintLL(group[0].tool, group[0].obj)
    for c in group:
      cc = ci.combine(cc, c)
    combined.append(cc)
  return combined

def execute_constraints(constraints, itasc, frames):
  reached_stats = []
  for c in constraints:
    print 'going to:\n'+str(c)+'\n'
    itasc.setTool(frames[c.tool])
    itasc.setObject(frames[c.obj])
    itasc.setConstraint(c)
    for i in range(200):
      if itasc.happy():
        print str(itasc.getAngles())
        break
      else:
        time.sleep(0.1)

    reached_stats.append(itasc.happy())

    if not itasc.happy():
      print 'TIMEOUT:'
      print 'constraint:\n'+str(c)
      print 'angles:\n'+str(itasc.getAngles())
    else:
      print 'got there'
    time.sleep(2.5)

  itasc.stop()
  return reached_stats


def execute_legacy(task_angles, itasc, frames):
  reached_stats = []
  error = 1.0

  itasc.setWeights([1,1,1,1,1,1])
  for angles_des in task_angles:
    print 'going to:\n'+str(angles_des)+'\n'
    itasc.setDesired(angles_des)
    for i in range(200):
      angles = itasc.getAngles()
      error = max(abs(a - a_des) for a,a_des in zip(angles, angles_des))
      if error < 0.01:
        print str(itasc.getAngles())
        break
      else:
        time.sleep(0.1)

    reached_stats.append(error < 0.01)

    if not error < 0.01:
      print 'TIMEOUT:'
      print 'angles_des:\n'+str(angles_des)
      print 'angles:\n'+str(itasc.getAngles())
    else:
      print 'got there'
    time.sleep(2.5)

  itasc.stop()
  return reached_stats


def main():
  rospy.init_node('tester')

  import thing_tools.handleItasc
  thing_tools.handleItasc._use_real_rosie = False
  itasc = thing_tools.handleItasc.HandleItasc(side='')

  sim = Tester(itasc)

  sim.pancake_flipping()

if __name__ == '__main__':
  main()
