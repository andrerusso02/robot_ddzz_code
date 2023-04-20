#!/usr/bin/env python3

import signal
import sys
import time

print(sys.path)

import rospy
from gotogoal import Ddzzbot
from turtlesim.msg import Pose


def handler(signum, frame):
    exit(1)

if __name__ == '__main__':


    pose1 = Pose()
    pose1.x = 1.0
    pose1.y = 0.0
    pose1.theta = 0.0

    pose2 = Pose()
    pose2.x = 0.0
    pose2.y = 0.0
    pose2.theta = 0.0

    signal.signal(signal.SIGINT, handler)
    x = Ddzzbot()
    try:
        while True:
            x.move2goal(pose1)
            time.sleep(2.0)
            x.move2goal(pose2)
            time.sleep(2.0)
    except rospy.ROSInterruptException:
        pass
