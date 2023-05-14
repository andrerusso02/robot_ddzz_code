import pygame, threading
import sys
from MatchSimu import *
import ActionneursSimu

EQUIPE = "VERT"

mul = 5.0


class Pose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0


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


# zoneDeposeMilieu = ZoneDepose(Pose(), Pose(), Pose(), Pose())
# zoneDeposeBas = ZoneDepose(Pose(), Pose(), Pose(), Pose())

posesPiles = []
zonesDepose = []
zonesDeposeInit = []


def waitForConfig():
    print("TODO attente de la config...")
    # un bouton HAUT BAS pour choisir l'equipe
    # un bouton pour dire que la config est OK
    # while not boutonConfigEnfonce:
    # time.sleep(1)
    print("Config OK")
    # lecture du bouton pour savoir si on est VERT ou BLEU

    global poseInit, posePileRose1, posePileJaune1, posePileMarron1, posePileMarron2, posePileJaune2, posePileRose2, zonesDepose, posesPiles, zonesDeposeInit
    if EQUIPE == "VERT":
        print("EQUIPE VERT")  # A DROITE
        poseInit.x = 2000.0-225.0  # mm
        poseInit.y = 225.0
        poseInit.theta = 90.0
        #
        posePileRose1.x = 2000.0-225.0
        posePileRose1.y = 575.0
        #
        posePileJaune1.x = 2000.0-225.0
        posePileJaune1.y = 775.0
        #
        posePileMarron1.x = 2000.0-350.0-375.0
        posePileMarron1.y = 3000.0/2.0-375.0
        # symetrie
        posePileMarron2.x = 2000.0-350.0-375.0
        posePileMarron2.y = 3000.0/2.0+375.0
        #
        posePileJaune2.x = 2000.0-225.0
        posePileJaune2.y = 3000.0-575.0-200.0
        #
        posePileRose2.x = 2000.0-225.0
        posePileRose2.y = 3000.0-575.0
        ####################
        # Zones de depot
        # zone cote droit, bas vert
        zoneDeposeBas = ZoneDepose(Pose(), Pose(), Pose(), Pose())
        zoneDeposeBas.entryPose.x = 2000.0-450.0-50.0-450.0/2.0
        zoneDeposeBas.entryPose.y = 3000.0-450.0-100.0
        zoneDeposeBas.pose1.x = zoneDeposeBas.entryPose.x
        zoneDeposeBas.pose1.y = zoneDeposeBas.entryPose.y+100.0+60.0
        zoneDeposeBas.pose2.x = zoneDeposeBas.entryPose.x
        zoneDeposeBas.pose2.y = zoneDeposeBas.entryPose.y+100.0+220.0
        zoneDeposeBas.pose3.x = zoneDeposeBas.entryPose.x
        zoneDeposeBas.pose3.y = zoneDeposeBas.entryPose.y+100.0+370.0
        # zone coté droit, milieu vert
        zoneDeposeMilieu = ZoneDepose(Pose(), Pose(), Pose(), Pose())
        zoneDeposeMilieu.entryPose.x = 2000.0-450.0-100.0
        zoneDeposeMilieu.entryPose.y = 3000.0-450.0-125.0-200.0-125.0-450.0/2.0
        zoneDeposeMilieu.pose1.x = zoneDeposeMilieu.entryPose.x+100.0+60.0
        zoneDeposeMilieu.pose1.y = zoneDeposeMilieu.entryPose.y
        zoneDeposeMilieu.pose2.x = zoneDeposeMilieu.entryPose.x+100.0+220.0
        zoneDeposeMilieu.pose2.y = zoneDeposeMilieu.entryPose.y
        zoneDeposeMilieu.pose3.x = zoneDeposeMilieu.entryPose.x+100.0+370.0
        zoneDeposeMilieu.pose3.y = zoneDeposeMilieu.entryPose.y


    else:  # EQUIPE == "BLEU"
        print("EQUIPE BLEU")  # A GAUCHE
        poseInit.x = 225.0
        poseInit.y = 225.0
        poseInit.theta = 90.0

        #
        posePileRose1.x = 225.0
        posePileRose1.y = 575.0
        #
        posePileJaune1.x = 225.0
        posePileJaune1.y = 575.0+200.0
        #
        posePileMarron1.x = 350.0+375.0
        posePileMarron1.y = 3000.0/2.0-150.0-350.0/2.0
        # symetrie
        posePileMarron2.x = 350.0+375.0
        posePileMarron2.y = 3000.0/2+150.0+350.0/2.0
        #
        posePileJaune2.x = 225.0
        posePileJaune2.y = 3000.0-575.0-200.0
        #
        posePileRose2.x = 225.0
        posePileRose2.y = 3000.0-575.0
        ###

        ####################
        # Zones de depot
        # zone cote droit, bas vert
        zoneDeposeBas = ZoneDepose(Pose(), Pose(), Pose(), Pose())
        zoneDeposeBas.entryPose.x = 450.0+50.0+450.0/2.0
        zoneDeposeBas.entryPose.y = 3000.0-450.0-100.0
        zoneDeposeBas.pose1.x = zoneDeposeBas.entryPose.x
        zoneDeposeBas.pose1.y = zoneDeposeBas.entryPose.y+100.0+60.0
        zoneDeposeBas.pose2.x = zoneDeposeBas.entryPose.x
        zoneDeposeBas.pose2.y = zoneDeposeBas.entryPose.y+100.0+220.0
        zoneDeposeBas.pose3.x = zoneDeposeBas.entryPose.x
        zoneDeposeBas.pose3.y = zoneDeposeBas.entryPose.y+100.0+370.0
        # zone coté droit, milieu vert
        zoneDeposeMilieu = ZoneDepose(Pose(), Pose(), Pose(), Pose())
        zoneDeposeMilieu.entryPose.x = 450.0+100.0
        zoneDeposeMilieu.entryPose.y = 3000.0-450.0-125.0-200.0-125.0-450.0/2.0
        zoneDeposeMilieu.pose1.x = zoneDeposeMilieu.entryPose.x-100.0-60.0
        zoneDeposeMilieu.pose1.y = zoneDeposeMilieu.entryPose.y
        zoneDeposeMilieu.pose2.x = zoneDeposeMilieu.entryPose.x-100.0-220.0
        zoneDeposeMilieu.pose2.y = zoneDeposeMilieu.entryPose.y
        zoneDeposeMilieu.pose3.x = zoneDeposeMilieu.entryPose.x-100.0-370.0
        zoneDeposeMilieu.pose3.y = zoneDeposeMilieu.entryPose.y

    zonesDepose = [zoneDeposeMilieu, zoneDeposeBas]
    posesPiles = [posePileRose1, posePileJaune1, posePileMarron1, posePileMarron2, posePileJaune2, posePileRose2]
    zonesDeposeInit = [zoneDeposeMilieu, zoneDeposeBas]


    poseInit.theta = 0  # radians
    OFFSET.x = poseInit.x
    OFFSET.y = poseInit.y


# interface pygame pour afficher toutes les poses
# init pygame
pygame.init()
screen = pygame.display.set_mode((2000/mul+20, 3000/mul+20))
pygame.display.set_caption('Interface de debug')
clock = pygame.time.Clock()

# couleurs
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (200, 200, 0)
PURPLE = (200, 0, 200)
BROWN = (100, 50, 0)
ORANGE = (255, 100, 0)


def drawPose(pose, color=RED, outlinedColor=BLACK):
    # print("drawPose", pose.x, pose.y)
    # draw a circle
    pygame.draw.circle(screen, color, (10+int(pose.x/mul),
                       int(10+pose.y/mul)), 6)  # radius 6, diameter 12
    pygame.draw.circle(screen, outlinedColor, (10+int(pose.x/mul),
                       int(10+pose.y/mul)), 6, 1)  # radius 6, diameter 12


def drawPoses():
    drawPose(poseInit, RED)
    drawPose(posePileRose1, PURPLE)
    drawPose(posePileJaune1, YELLOW)
    drawPose(posePileMarron1, BROWN)
    drawPose(posePileMarron2, BROWN)
    drawPose(posePileJaune2, YELLOW)
    drawPose(posePileRose2, PURPLE)

    # draw all zones one by one
    for zone in zonesDeposeInit:
        drawPose(zone.entryPose, GREEN)
        drawPose(zone.pose1, BLUE)
        drawPose(zone.pose2, BLUE)
        drawPose(zone.pose3, BLUE)


def drawTable():
    # draw a rectangle
    pygame.draw.rect(screen, BLACK, (10, 10, 2000/mul, 3000/mul), 4)

    # !! prendre en compte le decalage de 10 partout
    # draw a line in the rectangle at y = 300-45
    pygame.draw.line(screen, BLACK, (10, 3000/mul-450/mul+10),
                     (10+2000/mul, 3000/mul-450/mul+10), 1)
    # and a y=45
    pygame.draw.line(screen, BLACK, (10, 450/mul+10),
                     (10+2000/mul, 450/mul+10), 1)

    # draw a line in the rectangle at x = 200-45
    pygame.draw.line(screen, BLACK, (2000/mul-450/mul+10, 10),
                     (2000/mul-450/mul+10, 10+3000/mul), 1)
    # and a x=45
    pygame.draw.line(screen, BLACK, (450/mul+10, 10),
                     (450/mul+10, 10+3000/mul), 1)

    # lignes centrale horizontale en orange
    pygame.draw.line(screen, ORANGE, (10, 3000/mul/2+10),
                     (10+2000/mul, 3000/mul/2+10), 1)
    # lignes centrale verticale en orange
    pygame.draw.line(screen, ORANGE, (2000/mul/2+10, 10),
                     (2000/mul/2+10, 10+3000/mul), 1)

    # carre bleu en haut à gauche
    pygame.draw.rect(screen, BLUE, (10, 10, 450/mul, 450/mul), 2)
    # carre vert en haut à droite
    pygame.draw.rect(screen, GREEN, (2000/mul-450 /
                     mul+10, 10, 450/mul, 450/mul), 2)

    # carre vert a gauche en bas y=3000/2+375-450/2 OK
    pygame.draw.rect(screen, GREEN, (10, 3000/mul -
                     450/mul+10, 450/mul, 450/mul), 2)
    # carre bleu a droite en bas y=3000/2+375-450/2 OK
    pygame.draw.rect(screen, BLUE, (2000/mul-450/mul+10,
                                    3000/mul-450/mul+10, 450/mul, 450/mul), 2)

    # carre bleu a droite en y=3000/2-375-450/2
    pygame.draw.rect(screen, BLUE, (2000/mul-450/mul+10, 3000 /
                     mul/2-375/mul-450/mul/2+10, 450/mul, 450/mul), 2)
    # carre vert a gauche en y=3000/2-375-450/2
    pygame.draw.rect(screen, GREEN, (10, 3000/mul/2-375 /
                     mul-450/mul/2+10, 450/mul, 450/mul), 2)
    # carre vert a droite en y=3000/2+375-450/2
    pygame.draw.rect(screen, GREEN, (2000/mul-450/mul+10,
                     3000/mul/2+375/mul-450/mul/2+10, 450/mul, 450/mul), 2)
    # carre bleu a gauche en y=3000/2+375-450/2
    pygame.draw.rect(screen, BLUE, (10, 3000/mul/2+375 /
                     mul-450/mul/2+10, 450/mul, 450/mul), 2)

    # carre blue milieu en bas en x=450+50 OK
    pygame.draw.rect(screen, BLUE, (450/mul+10+50/mul, 3000 /
                     mul-450/mul+10, 450/mul, 450/mul), 2)
    # carre vert milieu en bas en x=2000-450-50 OK
    pygame.draw.rect(screen, GREEN, (2000/mul-450/mul+10-50 /
                     mul-450/mul, 3000/mul-450/mul+10, 450/mul, 450/mul), 2)

trajectoires = []

def drawRobot(lastPose):
    #le robot est un carré de 250mm de côté centré sur robot.pose
    pygame.draw.rect(screen, RED, (robot.pose.x/mul-250/2/mul+10, robot.pose.y/mul-250/2/mul+10, 250/mul, 250/mul), 2)
    # le robot a une ligne a l'avant qui indique son orientation
    pygame.draw.line(screen, RED, (robot.pose.x/mul+10, robot.pose.y/mul+10), 
                     (robot.pose.x/mul+10+250/mul*math.cos(robot.pose.theta), 
                      robot.pose.y/mul+10+250/mul*math.sin(robot.pose.theta)), 2)

    
    trajectoires.append((lastPose.x/mul+10, lastPose.y/mul+10, robot.pose.x/mul+10, robot.pose.y/mul+10))
    for t in trajectoires:
        pygame.draw.line(screen, RED, (t[0], t[1]), (t[2], t[3]), 2)
    lastPose = robot.pose


class Ddzzbot:
    def __init__(self) -> None:
        self.pose = poseInit

    def move2goal(self, goal, offset):
        print("\tmove2goal : ", goal.x, goal.y)
        # on calcule le theta
        theta = math.atan2(goal.y - self.pose.y, goal.x - self.pose.x)
        self.pose = goal
        self.pose.theta = theta


robot = Ddzzbot()

# on fait la boucle de dessin dans un thread
def drawLoop():
    # boucle principale
    lastPose = robot.pose
    while True:
        #flush
        screen.fill(WHITE)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            # if press Q or esc
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()
        drawTable()
        drawRobot(lastPose)
        drawPoses()
        lastPose = robot.pose
        pygame.display.flip()

        clock.tick(1)


def startMatch():
    match(robot, posesPiles, zonesDepose, OFFSET,poseInit)


if __name__ == '__main__':
    waitForConfig()

    # fenetre pygame
    screen.fill(WHITE)

    drawTable()
    drawPoses()
    pygame.display.flip()

    # on lance le thread
    threading.Thread(target=startMatch).start()
    # on lance la boucle de dessin
    drawLoop()


