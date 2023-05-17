#!/usr/bin/env python3

# program that subscribes to lidar laserscan (ros) and detects that min distance is less than 1m

import signal
import time

import rospy
from sensor_msgs.msg import LaserScan


def handler(signum, frame):
    exit(1)
    
signal.signal(signal.SIGINT, handler)
rospy.init_node('lidar', anonymous=True)


obstacle = False



def callback_lidar(data):
    _min = 100000000000
    for dist in data.ranges:
        if dist<_min and dist>0.07:
            _min = dist
    print("min distance = " + str(_min))

    global obstacle
    
    if _min < 0.30:
        obstacle = True
        print("STOPPPPPPPPPPPPP")
    else:
        obstacle = False


rospy.Subscriber("scan", LaserScan, callback_lidar)

rospy.spin() # todo enlever quand on utilisera le prog