#!/usr/bin/env python3

import signal
import time
from math import atan2, pi, pow, sqrt

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist

def handler(signum, frame):
    exit(1)

if __name__ == '__main__':

    signal.signal(signal.SIGINT, handler)

    rospy.init_node('turtlebot_controller', anonymous=True)

    velocity_publisher = rospy.Publisher('/diffbot/mobile_base_controller/cmd_vel',
                                                  Twist, queue_size=10)

    rate = rospy.Rate(20)

    try:
        while True:
            start = time.time()
            print("deb")  
            while time.time() - start < 2.0:
                rate.sleep()
                vel_msg = Twist()
                vel_msg.linear.x = 0.5
                vel_msg.angular.z = 0
                velocity_publisher.publish(vel_msg)
                
            
            print("fin")
            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
        
            time.sleep(2.0)

    except rospy.ROSInterruptException:
        pass