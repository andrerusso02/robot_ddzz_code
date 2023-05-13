#!/usr/bin/env python3

import signal
import rospy


def handler(signum, frame):
    exit(1)


class Actionneurs:

    def __init__(self):
        # Creates a node with name 'actionneurs' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('actionneurs', anonymous=True)

        # rate
        self.rate = rospy.Rate(20)

    def waitForInit(self):
        while True:
            print("TODO waiting for init of actionneurs")

    def prendrePile(self):
        while True:
            print("TODO prendrePile")

    def deposerPile(self):
        while True:
            print("TODO deposerPile")


if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    try:
        print("nothing to do here")
        # x = Ddzzbot()

        # time.sleep(3)

        # while (1):
        #     x.move2angle(pi, 5.0*(pi/180.0))
        #     print("pi")
        #     time.sleep(3)
        #     x.move2angle(0.0, 5.0*(pi/180.0))
        #     print("0")
        #     time.sleep(3)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
