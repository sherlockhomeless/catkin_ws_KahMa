#!/usr/bin/env python
from sets import Set
from random import choice
import rospy
import numpy as np
from numpy import ones,vstack, cross
from numpy.linalg import lstsq, norm
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2


debug = False

class LineDetector:
    """
    LineDetector implements the RANSAC algorithm to estimate lines on an image it received.

    Only points are used to draw the line because there does not seem to be support for drawing a line based on an equation:
    https://answers.opencv.org/question/127460/draw-a-line-on-an-image-using-equation-of-line-rather-than-2-points/
    """
    def __init__(self, s=2,t=30, N=10, d=0.4):
        # number of points to fit the model
        self.s = s
        # distance threshold t so probability of inliners is some amount p
        self.t = t
        # number of tries so that random sample is free from outliers
        self.N = N
        # consensus set size represented as %
        self.d = d

        rospy.init_node("a7_ransacer", anonymous = True)
        self.sub_img = rospy.Subscriber("/img_nice", Image, self.find_line, queue_size = 25)
        self.pub_img = rospy.Publisher("/img_ransaced", Image, queue_size=10)

        self.bridge = CvBridge()

    def find_line(self, img):
        global debug
        if debug is True: print("[LineFinder] received picture")
        img_bin = self.bridge.imgmsg_to_cv2(img, "mono8")
        #img = self.bridge.imgmsg_to_cv2(img, "rgb8")
        p1, p2 = self.ransac_img(img_bin)
        print("best model: ", p1, p2)
        img_color = self.bridge.imgmsg_to_cv2(img, "rgb8")
        img_color = self.draw_line(img_color, (p1[0], p1[1]), (p2[0], p2[1]))
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_color, "rgb8"))

    def ransac_img(self, img):
        global debug
        p1 = (0,0)
        p2 = (0,0)
        err = None

        point_set = Set([])

        # https://docs.scipy.org/doc/numpy/reference/arrays.nditer.html
        it = np.nditer(img, flags=["multi_index"])
        while not it.finished:
            # ! FORMAT (y, x)
            #if debug: print(str(it.multi_index) + ": " + str(it[0]))
            if it[0] == 255:
                # swap x and y so (y,x) => (x,y)
                point_set.add((it.multi_index[1], it.multi_index[0]))
            it.iternext()

        number_of_points = len(point_set)
        found_fitting_model = False
        round_count = 0
        # format [p1, p2, consensus in %]
        best_model = [[],[],0]
        consensus = 0

        while not found_fitting_model and round_count < self.N:
            inlier_count = 0.0
            p1, p2 = self.draw_random_points(point_set, self.s)
            #m, c = self.get_line_equation_from_points(p1, p2)
            for p in point_set:
                # https://stackoverflow.com/a/39840218
                distance_to_line =  norm(cross(p2-p1, p1-p))/norm(p2-p1)
                if debug: print(p1,p2, p, distance_to_line)
                if distance_to_line < self.t:
                    inlier_count += 1

            consensus = inlier_count/number_of_points
            if debug: print(number_of_points, consensus, best_model[2], self.d)
            if consensus >= self.d and best_model[2] < consensus:
                # replace best model
                best_model = (p1, p2, consensus)
                if debug: print("new best model", best_model)
            if debug: print("round: " , round_count, "inliers: ", inlier_count, "points: ", p1, p2)
            round_count += 1

        # TODO: find both lines
        # TODO: If there are dor more inliers, accept the line and refit using all inliers
        print ("ransac done!", consensus)
        return best_model[0], best_model[1]



    def draw_line(self, img, p1, p2):
        assert len(p1) == 2 and len(p2) == 2
        m, b = self.get_line_equation_from_points(p1,p2)
        p1y = 480
        p2y = 0
        p1x = int((p1y-b)/m)
        p2x = int((p2y-b)/m)
        print((p1x, p1y), (p2x, p2y), m, b)
        img = cv2.line(img, (p1x, p1y), (p2x, p2y), (255, 0, 0), 3)
        return img

    def draw_random_points(self, points, number_of_points):
        drawn_points = []
        for i in range(number_of_points):
            drawn_points.append(np.array(choice(tuple(points))))
        if debug: print("drawn points: " + str(drawn_points))
        return drawn_points

    def get_line_equation_from_points(self, p1, p2):
        # avoid division by 0
        if p1[0] - p2[0] == 0:
            p2 = (p2[0]+1, p2[1])
        m = float(p1[1] - p2[1]) / (p1[0] - p2[0])
        b = -(m*p1[0])+p1[1]
        if debug: print("Line Solution is y = {m}x + {b}".format(m=m,b=b))
        return m, b


# --- MAIN ---
if __name__ == "__main__":
    ld = LineDetector()
    while not rospy.is_shutdown():
        pass
