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

def map(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# convert to -pi / pi
def to_pi(angle):
    if angle > pi:
        angle = angle - 2*pi
    elif angle < -pi:
        angle = angle + 2*pi
    return angle

class Ddzzbot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)
        # # reinit position to zero
        # rospy.Service('set_pose',)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/diffbot/mobile_base_controller/cmd_vel',
                                                  Twist, queue_size=10)

        # subscriber for rviz goal (posestamped)
        # self.goal_subscriber = rospy.Subscriber(
        #     '/move_base_simple/goal', PoseStamped, self.goal_callback)

        # goal_pose = Pose()

        # rate
        self.rate = rospy.Rate(20)
        self.pose = Pose()

# LINEAR
        self.min_vel = 0.05
        self.max_vel = 0.4
        self.linear_coef = 1.5
        self.distanceAPartirDeLaquelleOnVaDoucement = 0.50 # si on est a distanceAPartirDeLaquelleOnVaDoucement du but, on ralentit

# ROTATION
        self.min_ang_vel = 0.1 # 0.3
        self.max_ang_vel = 2.0
        # self.rotation_coef = 1.5 # 2.5
        self.angleAPartirDeLaquelleOnRalenti = pi/2.0 # si on est a angleAPartirDeLaquelleOnRalenti du but, on ralentit

        # self.ang_vel_disabling_lin_vel = 0.5 # rad/s, si on tourne plus vite que ca, on ne bouge pas

        # self.vitesseMinDemarrage = 2 # rad/s

        self.lastTwistCommand = Twist() # c'est pour le boost de demarrage
        self.boostSpeed = 3.5
        self.framesSinceBoost = -1 # -1 = pas de boost
        self.maxFrames = 10

# LINEAR + ROTATION
        self.max_ang_vel_moving = 3.0 # si on veut rouler en même temps qu'on tourne

        while not self.update_pose():
            self.rate.sleep()
            pass
    
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
            return True
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error, could not get pose")
            return False
    
    def move2angle(self, angle, tolerance, time_lock=0.5):
        """go to angle in radians"""
        self.update_pose()
        # print("angle: " + str(angle))
        # print("pose.theta: " + str(self.pose.theta))
        # print("angle error: " + str(self.get_relative_angle(self.pose, angle)))
        
        while abs(self.get_relative_angle(self.pose, angle)) > tolerance:
            self.apply_rotation(angle)
            self.rate.sleep() # TODO METTRE LUI AVANT?
            print("angle: " + str(angle*180.0/3.1415))
            print("pose.theta: " + str(self.pose.theta*180.0/3.1415))
            print("diff: " + str(self.get_relative_angle(self.pose, angle)*180.0/3.1415))
            print()

        if time_lock != 0:
            # lock position for seconds
            t = time.time()
            while time.time() - t < time_lock:
                self.apply_rotation(angle)
                self.rate.sleep()
                # print("angle: " + str(angle*180.0/3.1415))
                # print("pose.theta: " + str(self.pose.theta*180.0/3.1415))
                # print("diff: " + str(self.get_relative_angle(self.pose, angle)*180.0/3.1415))
                # print()


        # stop
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.lastTwistCommand = vel_msg

        time.sleep(3)
        print("\n\nAPRES")
    
        print("angle: " + str(angle*180.0/3.1415))
        print("pose.theta: " + str(self.pose.theta*180.0/3.1415))
        print("diff: " + str(self.get_relative_angle(self.pose, angle)*180.0/3.1415))
        print()


        
    
    def apply_rotation(self, angle):
        """lock angle in radians"""
        self.update_pose()
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = self.compute_cmd_ang_vel(angle, self.max_ang_vel)
        self.velocity_publisher.publish(vel_msg)
        self.lastTwistCommand = vel_msg
    
    def apply_move(self, goal_pose):

        vel_msg = Twist()

        angle_to_reach = self.get_steering_angle(goal_pose)
        vel_msg.angular.z = self.compute_cmd_ang_vel(angle_to_reach, self.max_ang_vel_moving)

        threshold = pi/3.0
        print(self.get_relative_angle(self.pose, angle_to_reach))
        if abs(self.get_relative_angle(self.pose, angle_to_reach)) > threshold:
           vel_msg.linear.x = 0.0
        else: 
            dist_goal = self.get_distance(goal_pose)
            vel_msg.linear.x = self.compute_cmd_lin_vel(dist_goal)

        # vel_msg.angular.z = 0.0

        # print("vel_msg :")
        # print("linear.x: ", vel_msg.linear.x)
        # print("angular.z: ", vel_msg.angular.z)
        # print()

        self.velocity_publisher.publish(vel_msg)
        self.lastTwistCommand = vel_msg


        
        
    def get_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=2.0):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        cmd = constant * self.get_distance(goal_pose)
        if abs(cmd) > self.max_vel:
            cmd = self.max_vel * self.sign(cmd)
        return cmd

    def get_steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=2.0):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        # print(self.get_steering_angle(goal_pose))
        # print(self.pose.theta)
        cmd = (self.get_steering_angle(goal_pose) - self.pose.theta)
        if cmd > pi:
            cmd = cmd - 2*pi
        elif cmd < -pi:
            cmd = cmd + 2*pi
        cmd = constant * cmd
        return cmd
    
    # theta between -pi and pi
    def get_relative_angle(self, start_pose, goal_angle):
        angle = to_pi(goal_angle) - to_pi(start_pose.theta)
        angle = to_pi(angle)
        return angle


    def compute_cmd_lin_vel(self, goal_dist):
        if goal_dist > self.distanceAPartirDeLaquelleOnVaDoucement:
            return self.max_vel
        
        cmd = map(goal_dist, 0, self.distanceAPartirDeLaquelleOnVaDoucement, self.min_vel, self.max_vel)
        return cmd


    def compute_cmd_ang_vel(self, goal_angle, max_vel): # TODO voir
        # print("goal_angle: " + str(goal_angle))
        # print("pose.theta: " + str(self.pose.theta))
        # print("diff: " + str(goal_angle - self.pose.theta))
        angle = self.get_relative_angle(self.pose, goal_angle)

        if angle > self.angleAPartirDeLaquelleOnRalenti:
            cmd = max_vel * self.sign(angle)
        else:
            # angle < threshold 
            cmd = map(abs(angle), 0, self.angleAPartirDeLaquelleOnRalenti, self.min_ang_vel, max_vel) * self.sign(angle)


        # if (self.framesSinceBoost>-1 or self.lastTwistCommand.angular.z == 0.0 or self.lastTwistCommand.angular.z * cmd < 0): # si la derniere commande c'est zero ou si les directions sont inversees
        #     cmd = self.boostSpeed * self.sign(angle)
        #     print("booooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooost"+str(cmd)+" car")
        #     print("cmd = "+str(cmd)+"\t et lastZ = "+str(self.lastTwistCommand.angular.z))
        #     self.framesSinceBoost +=1

        # if (self.framesSinceBoost==self.maxFrames-1):
        #     self.framesSinceBoost = -1

        # print("cmd = "+str(cmd))
        return cmd

        # print("cmd: " + str(cmd))
        # cmd = self.rotation_coef * cmd
        # if abs(cmd) > self.max_ang_vel:
        #     cmd =  self.max_ang_vel * self.sign(cmd)
        # if abs(cmd) < self.min_ang_vel:
        #     cmd =  self.min_ang_vel * self.sign(cmd)
        # return cmd

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

        self.update_pose()

        # print("pose :")
        # print("x: ", self.pose.x)
        # print("y: ", self.pose.y)
        # print("theta: ", self.pose.theta)
        # print("distance to goal: ", dist)
        # print()

        self.move2angle(self.get_steering_angle(goal_pose), 0.02)

        print("moved to angle")

        dist = 10000.0
        distance_tolerance = 0.05
        while dist >= distance_tolerance:
            self.rate.sleep()
            self.update_pose()
            dist = self.get_distance(goal_pose)

            self.apply_move(goal_pose)

            # Publish at the desired rate.
        
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.lastTwistCommand = vel_msg

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

        time.sleep(3)

        while(1):
            x.move2angle(pi, 5.0*(pi/180.0))
            print("pi")
            time.sleep(3)
            x.move2angle(0.0, 5.0*(pi/180.0))
            print("0")
            time.sleep(3)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
