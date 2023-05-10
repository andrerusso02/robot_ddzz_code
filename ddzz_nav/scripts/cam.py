import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

rospy.init_node('camera_loclization', anonymous=True)
image_pub = rospy.Publisher("camera/cv2", Image)
bridge = CvBridge()

def pub_ros(cv_image):
    try:
        image_message = bridge.cv2_to_imgmsg(cv_image, "passthrough")
        image_pub.publish(image_message)
    except CvBridgeError as e:
        print(e)

cap = cv2.VideoCapture(2)
while cap.isOpened():
    ret, image = cap.read()
    output = image.copy()
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Find circles
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT,1, 100,
                            param1=50,param2=30,minRadius=170,maxRadius=210)
    # If some circle is found
    if circles is not None:
    # Get the (x, y, r) as integers
        circles = np.round(circles[0, :]).astype("int")
        print(circles)
        print()
        # loop over the circles
        for (x, y, r) in circles:
            cv2.circle(output, (x, y), r, (0, 255, 0), 2)
    # show the output image
    cv2.imshow("circle",output)
    pub_ros(output)

    if cv2.waitKey(1) == ord('q'):
        break