#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input


# Some useful variable declarations.
angle_range = 240	# sensor angle range of the lidar
car_length = 1.5	# distance (in m) that we project the car forward for correcting the error. 
desired_trajectory = 1	# distance from the wall (left or right - we cad define..but this is defined for right)
vel = 15 
error = 0.0

pub = rospy.Publisher('error', pid_input, queue_size=10)


## Input: data: Lidar scan data
## theta: The angle at which the distance is requried
## OUTPUT: distance of scan at angle theta
def getRange(data, theta):
    # Find the index of the arary that corresponds to angle theta
    # Return the lidar scan value at that index
    # Do some error checking for NaN and ubsurd values
    if math.isnan(theta) or math.isinf(theta):
        print 'encountered invalid value in getRange'
        theta = 0.0
    if theta < 0.0: theta = 0.0
    if theta > 180.0: theta = 180.0

    idx_float = ((theta+30.0) / 240.0) * len(data.ranges)
    idx = int(round(idx_float))
    return data.ranges[idx]


def callback(data):
    theta = 45.0
    a = getRange(data, theta)
    b = getRange(data, 0)
    swing = math.radians(theta)
    
    # constants: velocity at 10.0, distance at 0.5m
    vel = 10.0
    dist_desired = 0.5

    # compute the desired distance
    alpha = math.atan((a * math.cos(swing)-b) / (a * math.sin(swing)))
    dist_AB = b * math.cos(alpha)
    # distance of projection is how much the car moves in 0.1 seconds
    dist_AC = vel * 0.1
    dist_CD = dist_AB + dist_AC * math.sin(alpha)
    error = dist_desired - dist_CD

    msg = pid_input()
    msg.pid_error = error
    msg.pid_vel = vel
    pub.publish(msg)
	

if __name__ == '__main__':
    print("Laser node started")
    rospy.init_node('dist_finder', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()
