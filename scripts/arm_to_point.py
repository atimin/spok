#!/usr/bin/env python

import rospy
import re
import math
import sys
import os

path = os.path.dirname(__file__) + "/../src/"
sys.path.append(path)
import roslib; roslib.load_manifest('spok')
from spok.srv import *
from manipulator.arm import Arm
from manipulator.man import Man

def check_model(man, angles, pose):
    cfg = man.config

    test_man = Man(cfg)
    test_pose  = test_man.get_coord_joint(4)[0:3]

    print "Error: "
    print ("[ex = %0.2f; ey = %0.2f; ez = %0.2f]"
        % tuple([ x - y for x, y in zip(pose, test_pose)]))

def handle_move_to_point(req):
    arm = Arm(0)
    angles = arm.set_tool(req.x, req.y, req.z)
    check_model(arm, angles, [req.x, req.y, req.z])
    for i in xrange(4):
        print("Turn joint #%i by %0.2f, pheta = %0.2f" % (i, angles[i], arm.config[i][0]))
        try:
            turn_joint(i, angles[i], 0.5)
        except Exception, e:
            print e

    return MoveToPointResponse(0)

def spok_server():
    rospy.init_node('arm_to_point')
    rospy.Service('move_to_point', MoveToPoint, handle_move_to_point)
    
    print 'Ready to work'
    rospy.spin()


if __name__ == "__main__":
    print 'Search the /turn_joint service...'
    # Make client
    rospy.wait_for_service('turn_joint')
    try:
      turn_joint = rospy.ServiceProxy('turn_joint', TurnJoint)
    except Exception, e:
      raise e

    spok_server()
