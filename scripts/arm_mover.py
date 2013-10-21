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
from helpers.comm import *

def get_joint(i):
    return [0, 1, 3, 4, 5][i]

def handle_turn_joint(req):
    # Open port
    sp = open_serialport()    
    try:
        # Send command and close port
        if req.joint in [1]:
            # Servos number #1 and #2 are on a joint
            send_cmd(sp, "MOVE[SERV=1 ANGLE=%.3f VEL=%.3f]" 
                    % (math.pi - req.angle, req.velocity))
            respond = send_cmd(sp, "MOVE[SERV=2 ANGLE=%.3f VEL=%.3f]" 
                    % (req.angle, req.velocity))
        else:
            respond = send_cmd(sp, "MOVE[SERV=%i ANGLE=%.3f VEL=%.3f]" 
                    % (get_joint(req.joint), req.angle, req.velocity))
    finally:
        sp.close

    # Parse status
    p = re.compile(r"\d+")
    m = p.search(respond)
    status = int(m.group(0))

    return TurnJointResponse(status)

def handle_get_joint_status(req):
    sp = open_serialport()
    try:
        respond = send_cmd(sp, "STATUS[SERV=%i]\n" % (get_joint(req.joint)))
    finally:
        sp.close()

    # Parse status
    p = re.compile(r'^RESP\[SERV=(\d+) ANGLE=([\.\d]+) VEL=([\.\d]+)\]$')
    m = p.search(respond)
    joint = int(m.group(1))
    angle = float(m.group(2))
    velocity = float(m.group(3))

    return StatusJointResponse(0, joint, angle, velocity)


def spok_server():
    rospy.init_node('arm_mover')
    rospy.Service('turn_joint', TurnJoint, handle_turn_joint)
    rospy.Service('status_joint', StatusJoint, handle_get_joint_status)
    print 'Ready to move'
    rospy.spin()

if __name__ == "__main__":
    spok_server()
