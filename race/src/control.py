#!/usr/bin/env python

import rospy
from race.msg import drive_param
from race.msg import pid_input
from std_msgs.msg import Bool

import sys

kp = 14.0
kd = 0.09
servo_offset = 18.5	# zero correction offset in case servo is misaligned. 
prev_error = 0.0

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
estop = rospy.Publisher('eStop', Bool, queue_size=1)


def set_estop(val):
    estop.publish(val)


def control(data):
    global prev_error
    global kp
    global kd

    scale_factor = 5.0
    cur_error = scale_factor * data.pid_error
    angle = -(kp * cur_error + kd * (prev_error - cur_error))
    prev_error = cur_error

    if angle >= 100.0: angle = 100.0
    if angle <= -100.0: angle = -100.0

    msg = drive_param()
    msg.velocity = data.pid_vel
    msg.angle = angle
    pub.publish(msg)


def handler():
    set_estop(True)


if __name__ == '__main__':
    print "Listening to error for PID"
    
    kp = float(sys.argv[1]) if len(sys.argv) > 1 else kp
    kd = float(sys.argv[2]) if len(sys.argv) > 2 else kd

    rospy.init_node('pid_controller', anonymous=True)
    set_estop(False)
    rospy.on_shutdown(handler)
    rospy.Subscriber("error", pid_input, control)
    rospy.spin()
