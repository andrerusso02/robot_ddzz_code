# ros noetic node that subscribes to /measured_joint_states (sensor_msgs/JointState) and publishes independant float64 messages to /left_meas_joint_state and /right_meas_joint_state.

#fait tourner les moteurs avec une vitesse constante en utilistant le type de message diffbot_msgs/WheelsCmdStamped


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
    rospy.init_node('joint_state_subscriber', anonymous=True)
    pub_left = rospy.Publisher('measured_vel_left', Float64, queue_size=10)
    pub_right = rospy.Publisher('measured_vel_right', Float64, queue_size=10)
    rospy.Subscriber("diffbot/measured_joint_states", JointState, callback)
    pub_cmd = rospy.Publisher('diffbot/wheel_cmd_velocities', WheelsCmdStamped, queue_size=10)
    # publish commands at210Hz
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # publish constant velocity commands
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.wheels_cmd.angular_velocities.joint.append(2.0)
        msg.wheels_cmd.angular_velocities.joint.append(-2.0)
        pub_cmd.publish(msg)
        rate.sleep()