#!/usr/bin/env python3
import signal
import sys
import time
import rospy
from gotogoal import Ddzzbot
from turtlesim.msg import Pose
import math
from Actionneurs import Actionneurs

TEMPS_FIN_MATCH = 100  # secondes
TEMPS_FIN_MARGE = 80  # secondes, temps pour rentrer a la fin du match
TEMPS_DEGUISEMENT = 90  # secondes

DISTANCE_RECUL = 150.0  # mm


def handler(signum, frame):
    exit(1)


####################################
# VALEURS DEFS ICI, MAIS QUI SERONT LUES
EQUIPE = "VERT"  # "BLEU"

####################################
"""
# LE REPERE A POUR ORIGINE LE COIN HAUT GAUCHE
0 ------> x+
|
|
v
y+
"""

poseInit = Pose()
posePileRose1 = Pose()  # le palet1 rose du haut
posePileJaune1 = Pose()  # le palet2 jaune du haut
posePileMarron1 = Pose()  # le palet3 marron du haut
posePileMarron2 = Pose()  # le palet4 marron du bas
posePileJaune2 = Pose()  # le palet5 jaune du bas
posePileRose2 = Pose()  # le palet6 rose du bas

OFFSET = Pose()  # c'est le decalage du fait que le robot démarre pour lui en 0,0, il faut l'ajouter pour avoir la pose selon le robot


class ZoneDepose:
    def __init__(self, entryPose, pose1, pose2, pose3):
        self.entryPose = entryPose
        self.pose1 = pose1
        self.pose2 = pose2
        self.pose3 = pose3


zoneDeposeMilieu = ZoneDepose(Pose(), Pose(), Pose(), Pose())
zoneDeposeBas = ZoneDepose(Pose(), Pose(), Pose(), Pose())

posesPiles = [posePileRose1, posePileJaune1, posePileMarron1]
zonesDepose = [zoneDeposeMilieu, zoneDeposeBas]

startTime = 0
nombreDePilesRangees = 0  # 3 max


def waitForConfig():
    print("TODO attente de la config...")
    # un bouton HAUT BAS pour choisir l'equipe
    # un bouton pour dire que la config est OK
    # while not boutonConfigEnfonce:
    # time.sleep(1)
    print("Config OK")
    # lecture du bouton pour savoir si on est VERT ou BLEU

    if EQUIPE == "VERT":
        print("EQUIPE VERT")  # A DROITE
        poseInit.x = 2000.0-225.0  # mm
        poseInit.y = 225.0
        #
        posePileRose1.x = 2000.0-225.0
        posePileRose1.y = 575.0
        #
        posePileJaune1.x = 2000.0-225.0
        posePileJaune1.y = 575.0+200.0
        #
        posePileMarron1.x = 2000.0-350.0+375.0
        posePileMarron1.y = 3000.0-375.0-350.0
        # symetrie
        posePileMarron2.x = 2000.0-350.0+375.0
        posePileMarron2.y = 3000.0+375.0+350.0
        #
        posePileJaune2.x = 2000.0-225.0
        posePileJaune2.y = 3000.0-575.0-200.0
        #
        posePileRose2.x = 2000.0-225.0
        posePileRose2.y = 3000.0-575.0
        ####################
        # Zones de depot
        # zone cote droit, bas vert
        zoneDeposeBas.entryPose.x = 2000.0-450.0-50.0-450.0/2.0
        zoneDeposeBas.entryPose.y = 3000.0-450.0-100.0
        zoneDeposeBas.pose1.x = zoneDeposeBas.entryPose.x
        zoneDeposeBas.pose1.y = zoneDeposeBas.entryPose.y+100.0+60.0
        zoneDeposeBas.pose2.x = zoneDeposeBas.entryPose.x
        zoneDeposeBas.pose2.y = zoneDeposeBas.entryPose.y+100.0+220.0
        zoneDeposeBas.pose3.x = zoneDeposeBas.entryPose.x
        zoneDeposeBas.pose3.y = zoneDeposeBas.entryPose.y+100.0+370.0
        # zone coté droit, milieu vert
        zoneDeposeMilieu.entryPose.x = 2000.0-450.0+100.0
        zoneDeposeMilieu.entryPose.y = 3000.0-450.0-125.0-200.0-125.0-450.0/2.0
        zoneDeposeMilieu.pose1.x = zoneDeposeMilieu.entryPose.x-100.0-60.0
        zoneDeposeMilieu.pose1.y = zoneDeposeMilieu.entryPose.y
        zoneDeposeMilieu.pose2.x = zoneDeposeMilieu.entryPose.x-100.0-220.0
        zoneDeposeMilieu.pose2.y = zoneDeposeMilieu.entryPose.y
        zoneDeposeMilieu.pose3.x = zoneDeposeMilieu.entryPose.x-100.0-370.0
        zoneDeposeMilieu.pose3.y = zoneDeposeMilieu.entryPose.y

    else:  # EQUIPE == "BLEU"
        print("EQUIPE BLEU")  # A GAUCHE
        poseInit.x = 225.0
        poseInit.y = 225.0
        #
        posePileRose1.x = 225.0
        posePileRose1.y = 575.0
        #
        posePileJaune1.x = 225.0
        posePileJaune1.y = 575.0+200.0
        #
        posePileMarron1.x = 350.0+375.0
        posePileMarron1.y = 3000.0-375.0-350.0
        # symetrie
        posePileMarron2.x = 350.0+375.0
        posePileMarron2.y = 3000.0+375.0+350.0
        #
        posePileJaune2.x = 225.0
        posePileJaune2.y = 3000.0-575.0-200.0
        #
        posePileRose2.x = 225.0
        posePileRose2.y = 3000.0-575.0
        ###

    poseInit.theta = 0  # radians
    OFFSET.x = poseInit.x
    OFFSET.y = poseInit.y


def waitForTirette():
    print("TODO attente de la tirette...")
    while True:
        print("TODO lire la tirette")
    # TODO while not tiretteEnfonce:
    # time.sleep(1)
    print("Tirette OK")


def waitForInit(actionneurs):
    print("TODO attente de l'initialisation...")
    while True:
        print("TODO attendre l'initialisation")
    # TODO : communication avec les actionneurs et autres
    actionneurs.waitForInit()

    print("Init OK")


def dist(a, b):
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)


def verifTemps():
    # distanceDeRetour = distance(poseInit, positionActuelle)
    # tempsDeRetour = distanceDeRetour / vitesseRobot
    return time.time() - startTime < TEMPS_FIN_MARGE


def findNearestDeposeZone(positionActuelle):
    minDist = 100000000.0
    nearestDeposeZone = Pose()
    for zone in zonesDepose:
        d = dist(positionActuelle, zone)
        if d < minDist:
            minDist = d
            nearestDeposeZone = zone
    return nearestDeposeZone


def findNearestPile(positionActuelle):
    minDist = 100000000.0
    nearestPile = Pose()
    for pile in posesPiles:
        d = dist(positionActuelle, pile)
        if d < minDist:
            minDist = d
            nearestPile = pile
    return nearestPile


def main():
    signal.signal(signal.SIGINT, handler)
    robot = Ddzzbot()
    actionneurs = Actionneurs()

    waitForInit(actionneurs)
    waitForConfig()
    # on a maintenant la config du robot
    waitForTirette()
    # tirette enfoncee, on peut commencer le match de 100 secondes
    global startTime
    startTime = time.time()
    positionActuelle = Pose()

    try:
        while verifTemps():
            # on va chercher la premiere pile la plus proche tant qu'il reste des racks vides
            while nombreDePilesRangees < 3:
                positionGoal = findNearestPile(positionActuelle)
                print("On va a la pile la plus proche : ",
                      positionGoal.x, positionGoal.y, positionGoal.theta)
                robot.move2goal(positionGoal, OFFSET)
                time.sleep(5)
                actionneurs.prendrePile()
                nombreDePilesRangees += 1
                time.sleep(5)

            # TODO on a pris les piles, on va a la zone de depot la plus proche
            nearestDeposeZone = findNearestDeposeZone(positionActuelle)
            print("On va a la zone de depot la plus proche : ",
                  nearestDeposeZone.x, nearestDeposeZone.y, nearestDeposeZone.theta)
            robot.move2goal(nearestDeposeZone, OFFSET)
            time.sleep(5)
            actionneurs.deposerPile()
            time.sleep(5)
            # on recule un peu pour pas toucher la pile
            positionReculee = Pose()
            positionReculee.x = positionActuelle.x - \
                DISTANCE_RECUL*math.cos(positionActuelle.theta)
            positionReculee.y = positionActuelle.y - \
                DISTANCE_RECUL*math.sin(positionActuelle.theta)
            positionReculee.theta = positionActuelle.theta
            robot.move2goal(positionReculee, OFFSET)
            time.sleep(5)

        ##############################################
        # MARGE
        print("Temps fin marge atteint, retour a la maison")
        origin = Pose()
        origin.x = poseInit.x
        origin.y = poseInit.y
        origin.theta = 180.0
        # TODO voir pour le temps de retour, faut absolument désactiver les actionneurs avant la fin du match
        robot.move2goal(origin)

        # TODO on est sur la fin du match, on desactive les actionneurs
        print("desactivation des actionneurs ")
        actionneurs.desactiverActionneurs()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
