#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

debug = False

class Thresholder:
    """
    This class is used for preparing the image according to assignment 7-2.
    Since we recorded several different rosbags, it is necessary to pass the corresponding bag-name to this class since the correction is static.

    The cropped and binarised img is published under topic /img_nice
    """
    def __init__(self, name_of_bag):
        rospy.init_node("a7_thresholder", anonymous = True)
        self.name_of_bag = name_of_bag
        self.sub_img = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, self.threshold_img, queue_size = 25)
        self.pub_img = rospy.Publisher("/img_nice", Image, queue_size=10)
        self.bridge = CvBridge()


    def threshold_img(self, img):
        img = self.bridge.imgmsg_to_cv2(img, "mono8")
        img = self.rectangelize(img)



    def rectangelize(self, img):
        global debug
        if debug: print("[Thresholder] received picture")

        if self.name_of_bag == "camera_middle.bag":
            img = cv2.rectangle(img, (0, 0), (640, 140), 0, cv2.FILLED)
            img = cv2.rectangle(img, (0, 280), (640, 480), 0, cv2.FILLED)

        _, img = cv2.threshold(img, 205, 255, cv2.THRESH_BINARY)

        self.pub_img.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))
        return







# --- MAIN ---
if __name__ == "__main__":
    bags = ["camera_middle.bag", "camera_short.bag"]
    thresholder = Thresholder(bags[0])
    while not rospy.is_shutdown():
        pass
