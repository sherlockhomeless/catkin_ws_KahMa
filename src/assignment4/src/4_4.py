#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
img = []
cluster_counter = 0
# callback function not available here
sub = None
pub = rospy.Publisher("/camera_correction", Image, queue_size=10)

# TODO: better naming
def setup():
    rospy.init_node("cc", anonymous=True)
    sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, process_img, queue_size=10)
    bridge = CvBridge()
    # TODO: queue-size?
    camera_mat = np.array([[383.7944641113281, 0.0, 322.3056945800781],
                          [0.0, 383.7944641113281, 241.67051696777344],
                          [0.0, 0.0, 1.0]])

    distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

def inc_cluster_counter():
    global cluster_counter
    cluster_counter += 1
    if cluster_counter == 6:
        cluster_counter = 0

def process_img(img):
    # convert ros-image to opencv-image
    # todo: check mono8 string
    img = bridge.imgmsg_to_cv2(img, "mono8")
    # cv2.rectanle => draw rectanle( img, pt1, pt2, color, thiknkess)
    # img := input image from robot
    # pt1 := lower left vertex of rectanlge
    # pt2 := opposite vertex to pt1
    # color :=  rectangle color 
    # thikness := thikness of lines that make up rectangle
    img = cv2.rectangle(img, (0, 0), (300, 120), 0, cv2.FILLED) 
    img = cv2.rectangle(img, (0,0), (640, 90), 0, cv2.FILLED)
    img = cv2.rectangle(img, (0, 248), (640, 480), 0, cv2.FILLED)
    #  If the pixel value is smaller than the threshold, it is set to 0, otherwise it is set to a maximum value
    # cv2.threshold(src, threshold, maxval, threshtype)
    # src := input img
    # threshold := threshold value
    # maxval := if value > threshold -> set to this value
    # thresholdtype: https://docs.opencv.org/master/d7/d1b/group__imgproc__misc.html#gaa9e58d2860d4afa658ef70a9b1115576

    _, img = cv2.threshold(img, 205, 255, cv2.THRESH_BINARY)
    pub.publish(bridge.cv2_to_imgmsg(img, "mono8"))
    #x_dimension = 640
    #y_dimension = 480
    white = []
    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
           if img[y][x] == 255:
               white.append([x,y,None])
#          clusters = [[],[],[],[],[],[]]
#        
#        for cluster_num in range(6):
#            for elem in white:
#                for other_elem in white:
#                    if is_neighbour(elem, other_elem):
#                        clusters[cluster_num].append(other_elem)
#                        white.remove(other_elem)
#        print(clusters)
    for elem in white:
        find_cluster_for_point(elem, white)

#TODO: somehow clustering is not working 100%
#        for elem in white:
#            if elem[2] >= 6:
#                white.remove(elem)
#                print("removed" + str(elem))
    pixel_buckets  = sort_pixel(white)
#    print(pixel_buckets[0])
#    print(pixel_buckets[1])

    for bucket_num in range(len(pixel_buckets)):
        bucket_size = len(pixel_buckets[bucket_num])
        sum_x = 0
        sum_y = 0
        for elem in pixel_buckets[bucket_num]:
            sum_x += elem[0]
            sum_y += elem[1]
        sum_x = sum_x / bucket_size
        sum_y = sum_y / bucket_size    
        print("center of cluster " + str(bucket_num) + ": (" + str(sum_x) + ", " + str(sum_y) + ")")
    print('#####')
    cluster_counter = 0


def sort_pixel( pixels):
    buckets = [[], [], [], [], [], []]
    for pixel in pixels:
        #print("adding " + str(pixel) + " to bucket " + str(pixel[2]))
        buckets[pixel[2]].append(pixel)
    return buckets

def find_cluster_for_point(elem, points):
    for other_elem in points:
         if other_elem[2] is None:
             continue
         if elem[0] + 1 == other_elem[0] or elem[1] + 1 == other_elem[1] or elem[0] - 1 == other_elem[0] or elem[1] -1 == other_elem[1]:
             elem[2] = other_elem[2]
             return
    elem[2] = cluster_counter
    inc_cluster_counter()

def is_neighbour( elem1, elem2):
    return elem1[0]+1 == elem2[0] or elem1[1]+1 == elem2[1] or elem1[0]-1 == elem2[1] or elem1[1]-1== elem2[1]

def print_coords(x, y):
    x = str(x)
    y = str(y)
    print('(' + x + ', ' + y + ')')

if __name__ == "__main__":
    cc = setup()
