#!/usr/bin/env python3

import signal
import sys
import time

import rospy
from gotogoalV2 import Ddzzbot
from std_msgs.msg import String
from turtlesim.msg import Pose


def handler(signum, frame):
    exit(1)

couleur = ""

# souscription au topic ros /team_color pour connaitre la couleur de l'equipe : bleu ou vert
def callback_couleur(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global couleur
    couleur = data.data
    print("couleur : ", couleur)

def waitForConfig():
    rospy.loginfo("Waiting for config")
    while not rospy.is_shutdown() and couleur == "":
        rospy.sleep(0.1)
    rospy.loginfo("Config received")


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

# Creates a node with name 'turtlebot_controller' and make sure it is a unique node (using anonymous=True).
rospy.Subscriber("team_color", String, callback_couleur)
rospy.spin()

try:
    waitForConfig()
    # on a maintenant la config du robot et la tirette est tirée on peut commencer le match

    x.move2goal(pose1)
    time.sleep(4.0)
    x.move2goal(pose2)
    time.sleep(4.0)
except rospy.ROSInterruptException:
    pass

