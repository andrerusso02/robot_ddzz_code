#!/usr/bin/env python3

import signal
import time
from math import atan2, pi, pow, sqrt

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from turtlesim.msg import Pose


def handler(signum, frame):
        exit(1)

class Ddzzbot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/diffbot/mobile_base_controller/cmd_vel',
                                                  Twist, queue_size=10)

        # subscriber for rviz goal (posestamped)
        self.goal_subscriber = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.goal_callback)

        # goal_pose = Pose()

        # rate
        self.rate = rospy.Rate(10)

        self.pose = Pose()

        self.max_vel = 0.3
        self.max_ang_vel = 2.0
        self.ang_vel_disabling_lin_vel = 0.5
    
    def goal_callback(self, goal_pose):
        print("goal received")
        goal_pose = self.pose_stamped_to_pose(goal_pose)
        print(goal_pose)
        self.move2goal(goal_pose)
        # self.move2angle(goal_pose.theta, 0.1)

    def update_pose(self):
        """get pose from tf using get_pose()"""
        try:
            trans = self.tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time())
            # convert to Pose
            self.pose.x = trans.transform.translation.x
            self.pose.y = trans.transform.translation.y
            # convert quaternions to euler
            quaternion = trans.transform.rotation
            self.pose.theta = atan2(quaternion.z, quaternion.w)*2.0
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error, could not get pose")
    
    def move2angle(self, angle, tolerance):
        """go to angle in radians"""
        self.update_pose()
        # print("angle: " + str(angle))
        # print("pose.theta: " + str(self.pose.theta))
        # print("angle error: " + str(self.get_angle(self.pose, angle)))
        while abs(self.get_angle(self.pose, angle)) > tolerance:
            self.cmd_rotation(angle)
            # print("angle: " + str(angle))
            # print("pose.theta: " + str(self.pose.theta))
            # print("diff: " + str(self.get_angle(self.pose, angle)))
            # print()
        
        # lock position for 1 second
        t = time.time()
        while time.time() - t < 1.0:
            self.cmd_rotation(angle)

        # stop
        # vel_msg = Twist()
        # vel_msg.linear.x = 0
        # vel_msg.angular.z = 0
        # self.velocity_publisher.publish(vel_msg)
        
    
    def cmd_rotation(self, angle):
        """lock angle in radians"""
        self.update_pose()
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = self.angular_vel_rotation(angle)
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()
        
    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=2.0):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        cmd = constant * self.euclidean_distance(goal_pose)
        if abs(cmd) > self.max_vel:
            cmd = self.max_vel * self.sign(cmd)
        return cmd

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=2.0):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        # print(self.steering_angle(goal_pose))
        # print(self.pose.theta)
        cmd = (self.steering_angle(goal_pose) - self.pose.theta)
        if cmd > pi:
            cmd = cmd - 2*pi
        elif cmd < -pi:
            cmd = cmd + 2*pi
        cmd = constant * cmd
        return cmd
    
    def get_angle(self, start_pose, goal_pose):
        angle = goal_pose.theta - start_pose.theta
        if angle > pi:
            angle = angle - 2*pi
        elif angle < -pi:
            angle = angle + 2*pi
        return angle

    def get_angle(self, start_pose, goal_angle):
        angle = goal_angle - start_pose.theta
        if angle > pi:
            angle = angle - 2*pi
        elif angle < -pi:
            angle = angle + 2*pi
        return angle


    def angular_vel_rotation(self, goal_angle, constant=2.5):
        # print("goal_angle: " + str(goal_angle))
        # print("pose.theta: " + str(self.pose.theta))
        # print("diff: " + str(goal_angle - self.pose.theta))
        cmd = self.get_angle(self.pose, goal_angle)
        # print("cmd: " + str(cmd))
        cmd = constant * cmd
        if abs(cmd) > self.max_ang_vel:
            cmd =  self.max_ang_vel * self.sign(cmd)
        return cmd

    def sign(self, x):
        if x >= 0:
            return 1
        else:
            return -1

    def pose_stamped_to_pose(self, pose_stamped):
        """convert posestamped to pose"""
        pose = Pose()
        pose.x = pose_stamped.pose.position.x
        pose.y = pose_stamped.pose.position.y
        quaternion = pose_stamped.pose.orientation
        pose.theta = atan2(quaternion.z, quaternion.w)*2.0
        return pose

    def move2goal(self, goal_pose):

        self.update_pose()

        """Moves the turtle to the goal."""

        # Get the input from the user.
        # goal_pose.x = float(input("Set your x goal: "))
        # goal_pose.y = float(input("Set your y goal: "))

        # # Please, insert a number slightly greater than 0 (e.g. 0.01).
        # distance_tolerance = float(input("Set your tolerance: "))

        # goal_pose.x = 1.4
        # goal_pose.y = 0.0

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.1

        vel_msg = Twist()
        
        dist = 10000.0

        self.update_pose()

        print("pose :")
        print("x: ", self.pose.x)
        print("y: ", self.pose.y)
        print("theta: ", self.pose.theta)
        print("distance to goal: ", dist)
        print()

        self.move2angle(self.steering_angle(goal_pose), 0.05)

        print("moved to angle")

        while dist >= distance_tolerance:

            dist = self.euclidean_distance(goal_pose)

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            self.update_pose()

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # disable linear velocity if angular velocity is too high
            if abs(vel_msg.angular.z) > self.ang_vel_disabling_lin_vel:
                vel_msg.linear.x = 0.0
                print("angular velocity too high, disabling linear velocity")
            else:
                print("angular velocity low enough, enabling linear velocity")
        
            # print("vel_msg :")
            # print("linear.x: ", vel_msg.linear.x)
            # print("angular.z: ", vel_msg.angular.z)
            # print()


            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        print("moved to goal")
        print("pose :")
        print("x: ", self.pose.x)
        print("y: ", self.pose.y)
        print("theta: ", self.pose.theta)
        print("distance to goal: ", dist)
        print()

        self.move2angle(goal_pose.theta, 0.1)

        print("moved to angle")

        # # If we press control + C, the node will stop.
        # rospy.spin()

        print("goal reached")


if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    try:
        x = Ddzzbot()
        # x.move2goal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
