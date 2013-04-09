#!/usr/bin/python
# Copyright (c) 2011 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Author: Ingo Kresse <kresse at in.tum.de>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import roslib
roslib.load_manifest('motionControl')
roslib.load_manifest('motion_viz')
import rospy

from math import *

import yarp
import PyKDL as kdl
import time

from thing_tools import *

from robot_simulator import RobotSimulator

import signal
import random; random.seed()

import tool_utils

from std_msgs.msg import Int8

#import marker

# pose (in map coordinates) where rosie shall look for the tool
look_for_tool_pose =  [-2.98, 1.835, 0.01]


approx_tool = kdl.Frame(kdl.Rotation( 0.08 , -0.21 , -0.97 ,
                                     -0.12 ,  0.97 , -0.22 ,
                                      0.99 ,  0.13 ,  0.06 ),
                        kdl.Vector(   0.08 ,  0.31 ,  0.16 ))

_initialized = False

def init():
    global _initialized
    if not _initialized:
        rospy.init_node('tool_mover')
        yarp.Network.init()
        _initialized = True
        signal.signal(signal.SIGINT, signal_handler)

def signal_handler(signum, frame):
    print("Got a signal, calling KeyboardInterrupt!")
    raise KeyboardInterrupt

class ToolMover(object):
    def __init__(self):
        init() # initialize yarp and ros

        self.table_height = 0.855

        self.arm = HandleArm('lwr', 'right','/tmRightArm')
        self.larm = HandleArm('lwr', 'left', '/tmLeftArm')
        #tool_fr = list_to_frame(default_hand_tool)
        #self.arm.setTool(tool_fr)
        #self.arm.removeAllObstaclesBlind()

        config_file = 'config-lwr-right.py'
        self.hb = HandleBridge(config_file, 'tmdBridge')
        self.hbl = HandleBridge('config-lwr-left.py', 'tmdBridge')
        self.hjc = HandleJController(config_file, 'tmJC')
        self.robsim = RobotSimulator(config_file)

        self.nav = HandleNavP()
        self.hlo = HandleLO()
        self.hptu = HandlePTU()
        self.hcop = HandleCOP()
        self.rarm_action = HandleArmHandServer('right')
        self.larm_action = HandleArmHandServer('left')
        self.rgrasper = HandleGrasper('/tm_grasper_right','/grasper0')
        self.lgrasper = HandleGrasper('/tm_grasper_left','/grasper1')
        self.lweight = HandleWeight('/tm_weight_left','left')
        self.rweight = HandleWeight('/tm_weight_right','right')

        self.normal_vel = 29.9 * pi / 180.0  # Values from the armhand server
        self.hb.set_max_vel( self.normal_vel )

        self.state_pub = rospy.Publisher('/calib_state', Int8)

        #config area
        self.single_movement_timeout = 6.0

    def send_state(self, state):
        self.state_pub.publish(Int8(state))


    def arm_speed(self, vel, vf_vel):
        self.hb.set_max_vel( vel)
        self.hb.set_vf_max_vel(vf_vel)

    def set_arm_speed_fast(self):
        print('arm_speed: fast')
        self.arm_speed(39.0 * pi / 180.0, 0.39)

    def set_arm_speed_normal(self):
        print('arm_speed: normal')
        self.arm_speed(20.0 * pi / 180.0, 0.20)

    def set_arm_speed_slow(self):
        print('arm_speed: slow')
        self.arm_speed(20.0 * pi / 180.0, 0.10)

    def sleep(self, length):
        print "About to sleep: %4.2f" % (length)
        time.sleep(length)

    def arm_joint_distance(self, joints):
        '''return the sum of the distances of all the joint angles'''
        joints_now = self.arm.getActualJoints()
        diffv = self.vector_distance(joints_now, joints)

        diff = 0.0

        for i in diffv:
            diff += abs(i)

        return diff

    def vector_distance(self, v1, v2):
        'return difference vector v1 - v2'
        if len(v1) != len(v2):
            raise Exception('vectors should have same length')
        c = []

        for i in range(len(v1)):
            c.append(v1[i]-v2[i])

        return c



    def find_toolhandle(self):
        sensor_pose = self.hlo.getLOid('/openni_rgb_optical_frame')[0]
        answer = self.hcop.searchObjects('SpatulaHandle', 1, sensor_pose, 0)  # 0 is locate

        if answer != []:

            spatula_loid = answer[0].position
            fr = self.hlo.getF( spatula_loid, '/base_link')

            print "SpatulaHandle found at %4.2f %4.2f %4.2f" % (fr.p[0], fr.p[1], fr.p[2])
            #Height should be 0.92 - 1.01

            fr.p[2] = 1.02#0.976
            print "Forcing height to %4.2f" %(fr.p[2])
            return fr
        else:
            return None


    def find_lines(self, num=100):
        watch_spatula_map = (-3.4357326328702631, 1.2874688908691974, 0.26729483850908814)

        intermediate = [1.651159, 0.568041, -0.509002, 1.592075, -1.150115, -0.228047, -0.480893]
        watching_spatula = [0.097839, -0.915356, 0.360731, 1.941177, -1.786561, 0.469069, -0.475895]

        joints = self.arm.getActualJoints()
        eps = 0.1
        if self.arm_joint_distance(watching_spatula) > eps:
            self.rarm_action.joint_pose('open')
            self.hjc.setJointsBlocking(intermediate, self.hb, 5.0)
        self.nav.drive_map_pose(watch_spatula_map, self.hlo)
        self.hptu.look_at(self.hlo.getLOid('/right_arm_hand_link')[0], True)
        self.hjc.setJointsBlocking(watching_spatula, self.hb, 5.0)


        self.hptu.look_at(self.hlo.getLOid('/right_arm_hand_link')[0], True)
        self.hjc.setJointsBlocking(watching_spatula, self.hb, 5.0)

        self.sleep(2)

        self.find_lines_bare(num)


    def find_lines_bare(self, num=100, cov_val=0.06):
        right_arm = self.hlo.getLOid('/right_arm_hand_link')[0]
        search_space = self.hlo.updateFrame(approx_tool, right_arm, 0, '', 0, self.hlo.diagonalCov([cov_val]*6))

        self.hptu.look_at(search_space, False)
        answer = self.hcop.searchObjects('Line', num, search_space, 0)  # 0 is locate

        print 'found %d lines' % len(answer)
        return answer

    def find_cluster(self, num=5):
        search = self.hlo.getLOid('/openni_rgb_optical_frame')[0]

        answer = self.hcop.searchObjects('Cluster', num, search, 0)  # 0 is locate

        print 'found %d Cluster' % len(answer)
        return answer

    def find_obj(self, idobj, num=5):
        search = self.hlo.getLOid('/openni_rgb_optical_frame')[0]

        answer = self.hcop.searchObjects(idobj, num, search, 0)  # 0 is locate

        print 'found %d Cluster' % len(answer)
        return answer



    def find_and_grasp_tool(self):

        self.nav.drive_map_pose(look_for_tool_pose, self.hlo)
        #print "Reached map pose"

        self.hptu.look_at(self.hlo.getLOid('/kitchen-island-left')[0], True)
        self.sleep(2.0) # Getting old data from Kinect
        look_more = True

        while look_more:

            #Look at the pancake
            fr = self.find_toolhandle()

            if fr is not None:
                grab_loid = self.hlo.update_frame('/base_link','/spatula_handle', fr, 0)
                look_more = False

                self.rarm_action.move_to_reach("IceTea", "spatula", grab_loid)
                self.rarm_action.reach_primitive("IceTea", "spatula", grab_loid)

                #if the hand is empty, open_arm, look again
                #check grasp:
                def check_grasp(dist):
                    print "Finger grasp distance = %5.2f" % (dist)
                    # ~ 15 (grasp ok
                    # < 5 empty handed
                    # ~ 20 finger over thumb
                    # ~ 46 grasped 90deg off
                    if ((dist > 5.0) and (dist < 20.0)):
                        #grasped ok!
                        return True
                    else:
                        return False

                self.sleep(2.0)
                dist = self.rgrasper.getDistance( 3 )

                grasping_counter = 0

                good_grasp = check_grasp(dist)
                #good_grasp = True
                if ( not good_grasp ):
                    #retry!
                    while ((not good_grasp) and (grasping_counter < 2)) :
                        grasping_counter += 1
                        self.rarm_action.hand_primitive('open_relax')
                        self.sleep(3.0)
                        self.rarm_action.hand_primitive('spatula')
                        self.sleep(4.0)
                        dist = self.rgrasper.getDistance( 5 )
                        good_grasp = check_grasp(dist)

                        if not good_grasp:
                            self.sleep(1)

            else:
                print "Can't find the tool"
                #self.sleep(1)


    def move_to_manipulation_area(self):
        self.rarm_action.joint_pose('open')
        self.hb.cartesian_controller()
        f = kdl.Frame(kdl.Rotation.Quaternion(0.2027, 0.7216, 0.6267, 0.2130),
                      kdl.Vector(0.950, 0.035, 1.350))
        self.arm.gotoFrameBlocking(f, 15)


    def random_vector(self, magnitude):
        return kdl.Vector(*[random.uniform(-magnitude, magnitude) for i in range(3)])

    def random_transform(self, rot_max, trans_max):
        while True:
            axis = self.random_vector(1.0)
            if axis.Norm() <= 1.0:
                break
        rot = kdl.Rotation.Rot(axis, random.uniform(0, rot_max))
        trans = self.random_vector(trans_max)
        return kdl.Frame(rot, trans)


    def random_movements(self, max_iter=20, rot_max=30*pi/180, trans_max=0.08):

        start_pose = self.arm.getCurrentFrame(5)

        self.hb.cartesian_controller()

        def checkPose():
            pose = self.arm.getCurrentFrame()
            return (pose.p - start_pose.p).Norm() > 0.05

        for i in range(max_iter):
            pose = start_pose*self.random_transform(rot_max, trans_max)
            print pose
            if not self.robsim.can_reach(pose):
                continue
            self.arm.gotoFrameBlocking(pose, self.single_movement_timeout, checkPose)
        self.arm.gotoFrameBlocking(start_pose, self.single_movement_timeout*3, checkPose)


    def menu(self):
        go_on = True
        while go_on:
            try:
                print "Choose:"
                print "1. find and grasp tool"
                print "2. move to manipulation area"
                print "3. do random movements"
                print "rao. right-arm-open"
                print "lao. left-arm-open"
                print "lho. left-hand-open"
                print "rho. right-hand-open"
                print "lhc. left-hand-close"
                print "rhc. right-hand-close"
                print "0. exit"

                cmd = raw_input()

                if cmd == '1':
                    self.find_and_grasp_tool()
                elif cmd == '2':
                    self.move_to_manipulation_area()
                elif cmd == '3':
                    self.random_movements()
                elif cmd == 'rao':
                    self.rarm_action.joint_pose('open')
                elif cmd == 'lao':
                    self.larm_action.joint_pose('open')
                elif cmd == '0':
                    go_on = False
                elif cmd == 'lho':
                    self.larm_action.hand_primitive('open_relax')
                elif cmd == 'rho':
                    self.rarm_action.hand_primitive('open_relax')
                    weisswuerste_util.hide_wwcatcher_marker()
                elif cmd == 'lhc':
                    self.larm_action.hand_primitive('spatula')
                elif cmd == 'rhc':
                    self.rarm_action.hand_primitive('spatula')
                elif cmd == 'drop':
                    self.drop_ww_in_a_bowl()
            except Exception, e:
                return


# main
if __name__ == "__main__":

  init()

  tm = ToolMover()
  tm.menu()
