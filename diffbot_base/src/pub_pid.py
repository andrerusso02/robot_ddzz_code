# ros noetic node that subscribes to /measured_joint_states (sensor_msgs/JointState) and publishes independant float64 messages to /left_meas_joint_state and /right_meas_joint_state.

#fait tourner les moteurs avec une vitesse constante en utilistant le type de message diffbot_msgs/WheelsCmdStamped


import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from diffbot_msgs.msg import WheelsCmdStamped


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    left_meas_joint_state = Float64()
    left_meas_joint_state.data = data.velocity[0]
    right_meas_joint_state = Float64()
    right_meas_joint_state.data = data.velocity[1]
    pub_left.publish(left_meas_joint_state)
    pub_right.publish(right_meas_joint_state)



def handler(signum, frame):
    exit(1)

if __name__ == '__main__':
    print("entered")
    rospy.init_node('joint_state_subscriber', anonymous=True)
    pub_left = rospy.Publisher('measured_vel_left', Float64, queue_size=10)
    pub_right = rospy.Publisher('measured_vel_right', Float64, queue_size=10)
    pub_cmd_left = rospy.Publisher('cmd_vel_left', Float64, queue_size=10)
    pub_cmd_right = rospy.Publisher('cmd_vel_right', Float64, queue_size=10)
    rospy.Subscriber("diffbot/measured_joint_states", JointState, callback)
    pub_cmd = rospy.Publisher('diffbot/wheel_cmd_velocities', WheelsCmdStamped, queue_size=10)
    # publish commands at210Hz
    rate = rospy.Rate(20)

    cmd_vel_left = 5.0
    cmd_vel_right = -5.0

    start = time.time()

    while not rospy.is_shutdown() :
        time.sleep(3)
        print("\n\n\n DEPAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAART")
        while not rospy.is_shutdown() and time.time()<start+10.0:
            print("hello")
            # publish constant velocity commands
            msg = WheelsCmdStamped()
            msg.header.stamp = rospy.Time.now()
            msg.wheels_cmd.angular_velocities.joint.append(cmd_vel_left)
            msg.wheels_cmd.angular_velocities.joint.append(cmd_vel_right)
            pub_cmd.publish(msg)
            msg2 = Float64()
            msg2.data = cmd_vel_left
            pub_cmd_left.publish(msg2)
            msg3 = Float64()
            msg3.data = cmd_vel_right
            pub_cmd_right.publish(msg3)
            rate.sleep()

        cmd_vel_left = -cmd_vel_left
        cmd_vel_right = -cmd_vel_right
        start = time.time()