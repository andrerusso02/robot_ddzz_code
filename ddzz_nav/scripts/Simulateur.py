import pygame
import sys
EQUIPE = "BLEU"


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

posesPiles = [posePileRose1, posePileJaune1, posePileMarron1]
zonesDepose = []


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
        zoneDeposeMilieu.entryPose.x = 2000.0-450.0+100.0
        zoneDeposeMilieu.entryPose.y = 3000.0-450.0-125.0-200.0-125.0-450.0/2.0
        zoneDeposeMilieu.pose1.x = zoneDeposeMilieu.entryPose.x-100.0-60.0
        zoneDeposeMilieu.pose1.y = zoneDeposeMilieu.entryPose.y
        zoneDeposeMilieu.pose2.x = zoneDeposeMilieu.entryPose.x-100.0-220.0
        zoneDeposeMilieu.pose2.y = zoneDeposeMilieu.entryPose.y
        zoneDeposeMilieu.pose3.x = zoneDeposeMilieu.entryPose.x-100.0-370.0
        zoneDeposeMilieu.pose3.y = zoneDeposeMilieu.entryPose.y

        global zonesDepose
        zonesDepose = [zoneDeposeMilieu, zoneDeposeBas]

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


# interface pygame pour afficher toutes les poses
# init pygame
pygame.init()
screen = pygame.display.set_mode((1000, 1000))
pygame.display.set_caption('Interface de debug')
clock = pygame.time.Clock()

# couleurs
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)


def drawPose(pose):
    print("drawPose", pose.x, pose.y)
    # draw a circle
    pygame.draw.circle(screen, RED, (int(pose.x/10.0), int(pose.y/10.0)), 6)


def drawPoses():
    print("poses")
    for pose in posesPiles:
        drawPose(pose)
    print("zones")
    for zone in zonesDepose:
        drawPose(zone.entryPose)
        drawPose(zone.pose1)
        drawPose(zone.pose2)
        drawPose(zone.pose3)


if __name__ == '__main__':
    waitForConfig()

    # fenetre pygame
    screen.fill(WHITE)
    # drawPose(poseInit)
    # drawPose(posePileRose1)
    # drawPose(posePileJaune1)
    # drawPose(posePileMarron1)
    # drawPose(posePileMarron2)
    # drawPose(posePileJaune2)
    # drawPose(posePileRose2)
    drawPoses()
    pygame.display.flip()

    # boucle principale
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            # if press Q or esc
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()
        clock.tick(60)
