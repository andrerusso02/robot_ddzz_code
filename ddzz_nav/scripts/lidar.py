#!/usr/bin/env python3

# program that subscribes to lidar laserscan (ros) and detects that min distance is less than 1m

import signal
import time

import rospy
from sensor_msgs.msg import LaserScan


def handler(signum, frame):
    exit(1)
    
signal.signal(signal.SIGINT, handler)


obstacle = False

def update_obstacle(data):
    global obstacle
    min = min(data.ranges)
    if min < 0.22:
        obstacle = True
    else:
        obstacle = False

def callback(data):
    print("time = " + str(time.time()))
    min = min(data.ranges)
    print("min distance = " + str(min))
    # if 




rospy.Subscriber("scan", LaserScan, callback)

rospy.spin() # todo enlever quand on utilisera le prog

