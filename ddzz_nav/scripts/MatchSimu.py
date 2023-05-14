import math
import time
from ActionneursSimu import Actionneurs
# from Simulateur import zonesDepose, posesPiles,OFFSET
from Simulateur import Ddzzbot

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


class Pose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0


class ZoneDepose:
    def __init__(self, entryPose, pose1, pose2, pose3):
        self.entryPose = entryPose
        self.pose1 = pose1
        self.pose2 = pose2
        self.pose3 = pose3


startTime = 0
nombreDePilesRangees = 0  # 3 max
positionActuelle = Pose()

actionneurs = Actionneurs()


def dist(a, b):
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)


def verifTemps():
    # distanceDeRetour = distance(poseInit, positionActuelle)
    # tempsDeRetour = distanceDeRetour / vitesseRobot
    print("Temps restant : ", TEMPS_FIN_MARGE - (time.time() - startTime))
    return time.time() - startTime < TEMPS_FIN_MARGE


def findNearestDeposeZone(positionActuelle, zonesDepose):
    minDist = 100000000.0
    nearestDeposeZone = Pose()
    for zone in zonesDepose:
        d = dist(positionActuelle, zone.entryPose)
        if d < minDist:
            minDist = d
            nearestDeposeZone = zone
    zonesDepose.remove(nearestDeposeZone)
    return nearestDeposeZone, zonesDepose


def findNearestPile(positionActuelle, posesPiles):
    minDist = 100000000.0
    nearestPile = Pose()
    for pile in posesPiles:
        d = dist(positionActuelle, pile)
        if d < minDist:
            minDist = d
            nearestPile = pile
    posesPiles.remove(nearestPile)
    return nearestPile, posesPiles


def match(robot, posesPiles1, zonesDepose1, OFFSET1, poseInit):

    posesPiles = posesPiles1
    zonesDepose = zonesDepose1
    OFFSET = OFFSET1

    global startTime, nombreDePilesRangees, positionActuelle
    startTime = time.time()
    tempsPause = 1
    while verifTemps():
        # on va chercher la premiere pile la plus proche tant qu'il reste des racks vides
        while nombreDePilesRangees < 3:
            positionGoal, posesPiles = findNearestPile(
                positionActuelle, posesPiles)
            print("On va a la pile la plus proche : ",
                  positionGoal.x, positionGoal.y, positionGoal.theta)
            robot.move2goal(positionGoal, OFFSET)
            time.sleep(tempsPause)
            actionneurs.prendrePile()
            nombreDePilesRangees += 1
            time.sleep(tempsPause)

        # TODO on a pris les piles, on va a la zone de depot la plus proche
        nearestDeposeZone, zonesDepose = findNearestDeposeZone(
            positionActuelle, zonesDepose)
        print("On va a la zone de depot la plus proche : ",
              nearestDeposeZone.entryPose.x, nearestDeposeZone.entryPose.y, nearestDeposeZone.entryPose.theta)
        robot.move2goal(nearestDeposeZone.entryPose, OFFSET)
        time.sleep(tempsPause)

        # on va a la 3e position de la zone de depot et on depose la pile
        print("On va a la 3e position de la zone de depot : ",
              nearestDeposeZone.pose3.x, nearestDeposeZone.pose3.y)
        robot.move2goal(nearestDeposeZone.pose3, OFFSET)
        time.sleep(tempsPause)
        actionneurs.deposerPile()
        nombreDePilesRangees -= 1
        print("On va a la 2e position de la zone de depot : ",
              nearestDeposeZone.pose2.x, nearestDeposeZone.pose2.y)
        robot.move2goal(nearestDeposeZone.pose2, OFFSET)
        time.sleep(tempsPause)
        actionneurs.deposerPile()
        nombreDePilesRangees -= 1
        print("On va a la 1e position de la zone de depot : ",
              nearestDeposeZone.pose1.x, nearestDeposeZone.pose1.y)
        robot.move2goal(nearestDeposeZone.pose1, OFFSET)
        time.sleep(tempsPause)
        actionneurs.deposerPile()
        nombreDePilesRangees -= 1

        time.sleep(tempsPause)
        # on recule un peu pour pas toucher la pile
        positionReculee = Pose()
        positionActuelle = robot.pose
        positionReculee.x = positionActuelle.x + \
            DISTANCE_RECUL*math.cos(-positionActuelle.theta)
        positionReculee.y = positionActuelle.y + \
            DISTANCE_RECUL*math.sin(-positionActuelle.theta)
        positionReculee.theta = positionActuelle.theta
        print("actuelle : ", positionActuelle.x, positionActuelle.y)
        print("On recule un peu pour pas toucher la pile vers : ",
              positionReculee.x, positionReculee.y)
        robot.move2goal(positionReculee, OFFSET)
        time.sleep(tempsPause)

        print("################################################FIN D'UN TOUR")
        print("Temps restant : ", TEMPS_FIN_MARGE - (time.time() - startTime))

        if (posesPiles == []):
            print("On a range toutes les piles")
            break

    ##############################################
    # MARGE
    print("Temps fin marge atteint ou toutes piles rangees, retour a la maison")
    origin = Pose()
    origin.x = poseInit.x
    origin.y = poseInit.y
    origin.theta = 180.0
    # TODO voir pour le temps de retour, faut absolument désactiver les actionneurs avant la fin du match
    robot.move2goal(origin, OFFSET)

    # TODO on est sur la fin du match, on desactive les actionneurs
    print("desactivation des actionneurs ")
    actionneurs.desactiverActionneurs()
