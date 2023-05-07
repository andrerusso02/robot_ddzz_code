# ros noetic node that subscribes to the /sensors/imu topic and publishes to the /imu topic.
# The node must replace every frame_id with "imu".

#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu


def callback(data):
    data.header.stamp = rospy.Time.now()

    data.header.frame_id = "imu"
    pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('convert')
    pub = rospy.Publisher('/imu', Imu, queue_size=10)
    rospy.Subscriber('/android/imu', Imu, callback)
    rospy.spin()