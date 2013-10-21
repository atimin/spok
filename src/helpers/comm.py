# Mudule of helper methods for communication
import rospy
import time
import serial

def open_serialport():
    port = rospy.get_param('/spok/arm_mover/port')
    baud = rospy.get_param('/spok/arm_mover/baud')
    return serial.Serial(port, baud)

def send_cmd(sp, cmd):
    if rospy.get_param('/spok/arm_mover/debug') == 0:
        sp.write(cmd)
        respond = sp.readline()
    else:
        time_start = time.time()
        sp.write(cmd)
        print("[Send]: %s" % cmd.strip())
        respond = sp.readline()
        print("[Rec]: %s" % respond.strip())
        print("[Time %.3f ms]" % ((time.time() - time_start) * 1000.0))
    
    return respond
