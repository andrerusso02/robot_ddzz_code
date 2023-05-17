#!/usr/bin/env python3

import signal
import time
import rospy
import tf2_ros
from math import atan2, pi, pow, sqrt
from geometry_msgs.msg import PoseStamped, Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def handler(signum, frame):
    exit(1)

# convert to -pi / pi


def to_pi(angle):
    if angle > pi:
        angle = angle - 2*pi
    elif angle < -pi:
        angle = angle + 2*pi
    return angle


def sign(x):
    if x >= 0:
        return 1
    else:
        return -1


def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min









class Ddzzbot:

    def __init__(self):
        self.obstacle = False
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.Subscriber("team_color", String, self.callback_couleur)
        rospy.Subscriber("scan", LaserScan, self.callback_lidar)


        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(
            '/diffbot/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        # rate
        self.rate = rospy.Rate(20)
        self.pose = Pose()

# LINEAR
        self.min_vel = 0.05
        self.max_vel = 0.4
        # self.linear_coef = 1.5
        # si on est a distanceAPartirDeLaquelleOnVaDoucement du but, on ralentit
        self.distanceAPartirDeLaquelleOnVaDoucement = 0.50

# ROTATION
        self.min_ang_vel = 0.3  # 0.3
        self.max_ang_vel = 1.0
        # si on est a angleAPartirDeLaquelleOnRalenti du but, on ralentit
        self.angleAPartirDeLaquelleOnRalenti = pi/2.0

        # self.boostSpeed = 3.5
        # self.framesSinceBoost = -1  # -1 = pas de boost
        # self.maxFrames = 10

# LINEAR + ROTATION
        self.max_ang_vel_moving = 6.0  # si on veut rouler en même temps qu'on tourne
        self.couleur = ""






    # souscription au topic ros /team_color pour connaitre la couleur de l'equipe : bleu ou vert
    def callback_couleur(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.couleur
        self.couleur = data.data
        print("couleur : ", self.couleur)

    def callback_lidar(self,data):
        _min = 100000000000
        for dist in data.ranges:
            if dist<_min and dist>0.07:
                _min = dist
        # print("min distance = " + str(_min))
        
        if _min < 0.50:
            self.obstacle = True
            print("STOPPPPPPPPPPPPP")
        else:
            self.obstacle = False

    def waitForConfig(self):
        print("Waiting for config")
        while not rospy.is_shutdown() and self.couleur == "":
            print("waiting...")
            rospy.sleep(0.1)
        print("Config received")

    def apply_rotation(self, angle):
        """lock angle in radians"""
        self.update_pose()
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = self.compute_cmd_ang_vel(angle, self.max_ang_vel)
        self.velocity_publisher.publish(vel_msg)

    def move2angle(self, angle, tolerance, time_lock=0.5):
        """go to angle in radians"""
        self.update_pose()
        # print("angle: " + str(angle))
        # print("pose.theta: " + str(self.pose.theta))
        # print("angle error: " + str(self.get_relative_angle(self.pose, angle)))

        while abs(self.get_relative_angle(self.pose, angle)) > tolerance:
            # verification des collisions
            if self.check_collision():
                vel_msg = Twist()
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                print("collision detected")

            while self.check_collision():
                # on attend ici tant qu'il y a un risque de collision
                self.rate.sleep()
            self.apply_rotation(angle)
            self.rate.sleep()  # TODO METTRE LUI AVANT?
            # print("angle: " + str(angle*180.0/3.1415))
            # print("pose.theta: " + str(self.pose.theta*180.0/3.1415))
            # print("diff: " + str(self.get_relative_angle(self.pose, angle)*180.0/3.1415))
            # print()

        if time_lock != 0:
            # lock position for seconds
            t = time.time()
            while time.time() - t < time_lock:
                print(self.obstacle)
                # verification des collisions
                if self.check_collision():
                    vel_msg = Twist()
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = 0
                    self.velocity_publisher.publish(vel_msg)
                    print("collision detected")

                while self.check_collision():
                    # on attend ici tant qu'il y a un risque de collision
                    self.rate.sleep()
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

        # time.sleep(3)
        print("\n\nAPRES")

        print("angle: " + str(angle*180.0/3.1415))
        print("pose.theta: " + str(self.pose.theta*180.0/3.1415))
        print("error: " + str(self.get_relative_angle(self.pose, angle)*180.0/3.1415))
        print()

####################################################################
#
    def move2goal(self, goal_pose_tmp, offset):
        self.update_pose()

    # goal
        goal_pose = Pose()
        goal_pose.x = goal_pose_tmp.x + offset.x
        goal_pose.y = goal_pose_tmp.y + offset.y
        goal_pose.theta = goal_pose_tmp.theta

        print("ON VEUT ALLER EN POSE :")
        print("x: ", goal_pose_tmp.x, "soit en repere robot : ", goal_pose.x)
        print("y: ", goal_pose_tmp.y, "soit en repere robot : ", goal_pose.y)
        print("theta: ", goal_pose.theta)
        print("ON EST EN POSE :")
        print("x: ", self.pose.x)
        print("y: ", self.pose.y)
        print("theta: ", self.pose.theta)
        # print("distance to goal: ", dist)
        print()

        print("On tourne deja en direction du but : " +
              str(self.get_steering_angle(goal_pose)*180.0/3.14)+"...")
        self.move2angle(self.get_steering_angle(goal_pose), 0.02)
        print("on a tourne en direction du but")

        print()

        print("On avance vers le but...")
        dist = self.get_distance(goal_pose)
        distance_tolerance = 0.05
        while dist >= distance_tolerance:
            print(self.obstacle)
            # verification des collisions
            if self.check_collision():
                vel_msg = Twist()
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                print("collision detected")

            while self.check_collision():
                # on attend ici tant qu'il y a un risque de collision
                print(dist)
                self.rate.sleep()
            


            self.rate.sleep()
            self.update_pose()
            # applique le mouvement permettant d'aller vers le but
            self.apply_move(goal_pose)
            dist = self.get_distance(goal_pose)

        # STOP
        vel_msg = Twist()
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

        # # If we press control + C, the node will stop.
        # rospy.spin()

        print("goal reached")

    """
    Renvoie un angle entre -pi et pi entre la position actuelle et la position desiree
    """

    def get_steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def update_pose(self):
        """get pose from tf using get_pose()"""
        try:
            trans = self.tfBuffer.lookup_transform(
                'odom', 'base_footprint', rospy.Time())
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

    def goal_callback(self, goal_pose):
        print("goal received")
        goal_pose = self.pose_stamped_to_pose(goal_pose)
        print(goal_pose)
        self.move2goal(goal_pose)
        # self.move2angle(goal_pose.theta, 0.1)

    def get_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def check_collision(self):
        # si y'a un adversaire qui est trop proche
        if self.obstacle:
            print("self.obstacle detecte !!")
        #return False
        return self.obstacle

    # theta between -pi and pi
    def get_relative_angle(self, start_pose, goal_angle):
        angle = to_pi(goal_angle) - to_pi(start_pose.theta)
        angle = to_pi(angle)
        return angle

    def apply_move(self, goal_pose):
        vel_msg = Twist()

        # si on a un angle supérieur au threshold, on arrete le mouvement et on fait un move2angle
        threshold = pi/3.0
        angle_to_reach = self.get_steering_angle(goal_pose)
        angle_error = abs(self.get_relative_angle(self.pose, angle_to_reach))
        if angle_error > threshold:
            print("angle error too big, stopping linear x and correcting angle, erreur = "+str(angle_error))
            self.move2angle(angle_to_reach, 0.02)

        angle_error = abs(self.get_relative_angle(self.pose, angle_to_reach))
        print("angle error apres : "+str(angle_error))

        # on a corrige l'angle, on peut avancer
        vel_msg.angular.z = self.compute_cmd_ang_vel(angle_to_reach, self.max_ang_vel_moving)

        dist_goal = self.get_distance(goal_pose)
        vel_msg.linear.x = self.compute_cmd_lin_vel(dist_goal)

        # print("vel_msg :")
        # print("linear.x: ", vel_msg.linear.x)
        # print("angular.z: ", vel_msg.angular.z)
        # print()

        self.velocity_publisher.publish(vel_msg)

    def compute_cmd_lin_vel(self, goal_dist):
        if goal_dist > self.distanceAPartirDeLaquelleOnVaDoucement:
            return self.max_vel

        cmd = map(goal_dist, 0, self.distanceAPartirDeLaquelleOnVaDoucement,
                  self.min_vel, self.max_vel)
        return cmd

    def compute_cmd_ang_vel(self, goal_angle, max_vel):  # TODO voir
        angle = self.get_relative_angle(self.pose, goal_angle)

        if angle > self.angleAPartirDeLaquelleOnRalenti:
            cmd = max_vel * sign(angle)
        else:
            cmd = map(abs(angle), 0, self.angleAPartirDeLaquelleOnRalenti,
                      self.min_ang_vel, max_vel) * sign(angle)

        return cmd

    def pose_stamped_to_pose(self, pose_stamped):
        """convert posestamped to pose"""
        pose = Pose()
        pose.x = pose_stamped.pose.position.x
        pose.y = pose_stamped.pose.position.y
        quaternion = pose_stamped.pose.orientation
        pose.theta = atan2(quaternion.z, quaternion.w)*2.0
        return pose



    
if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    print("starting homologation...")
    try:
        
        x = Ddzzbot()
        print("ddzzbot initiated")
        x.waitForConfig()
        while not x.update_pose():
            print("updating pose...")
            x.rate.sleep()
            pass

        print("END OF INIT")
        offset = Pose()
        offset.x = 0.0
        offset.y = 0.0
        offset.theta = 0.0

        pose1 = Pose()
        pose1.x = 0.0
        pose1.y = 0.0
        pose1.theta = 0.0

        pose2 = Pose()
        pose2.x = 1.0
        pose2.y = 0.0
        pose2.theta = 0.0

        pose3 = Pose()
        pose3.x = 1.0
        pose3.y = 1.0
        pose3.theta = 0.0

        time.sleep(3)
        # test, on veut que le robot fasse un carre
        print("on fait un carre")
        # x.move2goal(pose1,offset)
        # time.sleep(3)
        x.move2goal(pose2,offset)
        time.sleep(3)
        x.move2goal(pose1,offset)
        time.sleep(3)
        x.move2goal(pose3,offset)
        print("fini")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
