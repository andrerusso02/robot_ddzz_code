import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ImagePublisher(object):

  def __init__(self):
    self.node_rate = 1

    self.image_pub = rospy.Publisher("image_topic", Image)
    cv_image = cv2.imread('/Pictures/image.png',0)
    self.bridge = CvBridge()

    try:
      self.image_message = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
    except CvBridgeError as e:
      print(e)

    time.sleep(5)
    self.image_pub.publish(self.image_message)

  def doSmth(self):
    rospy.loginfo('its working!')
    # self.image_pub.publish(self.image_message)

  def run(self):
    loop = rospy.Rate(self.node_rate)
    while not rospy.is_shutdown():
      self.doSmth()
      loop.sleep()