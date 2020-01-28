#!/usr/bin/env python
# coding=utf8

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from genpy import Time

import numpy as np
import scipy.interpolate
import matplotlib.pyplot as plt
from numpy.linalg import norm
from os.path import exists
from random import randint
from sys import maxint
import os


"""
@K => Name node in catkin_ws "assignment10"!!!!

Lane arrays are of form:
arc-length, x, y:
...
[ 9.31823517  1.153894    3.258756  ]
[ 9.32823494  1.145771    3.252924  ]
[ 9.33823533  1.137734    3.246973  ]
[ 9.34823491  1.129698    3.241022  ]
...

"""


"""
--- HELPER FUNCTIONS ---
"""

def setup_marker(c, id=None, maker_type=Marker.LINE_STRIP, scale=(0.025,0.025)):
    assert type(c) is tuple and len(c) == 3
    m = Marker(type=maker_type, action=Marker.ADD)
    m.header.frame_id = "map"
    m.color.r, m.color.b, m.color.g = c
    m.color.a = 1.0
    m.scale.x, m.scale.y = scale
    """
    MARKERS REPLACE MARKER WITH SAME ID!!!!!!!!!!
    http://wiki.ros.org/rviz/DisplayTypes/Marker
    """
    if id == None:
        id = randint(0, 2**31)
    m.id = id
    return m

def d00(*s):
    print(s)


class LaneSpline:

    def __init__(self, supporting_points):
        self.supporting_points = supporting_points
        self.x_spline, self.y_spline = None, None

        arcs, xs, ys = [],[],[]
        for el in supporting_points:
            arcs.append(el[0])
            xs.append(el[1])
            ys.append(el[2])

        self.x_spline = scipy.interpolate.CubicSpline(arcs, xs, bc_type='periodic')
        self.y_spline = scipy.interpolate.CubicSpline(arcs, ys, bc_type='periodic')

    def spline_it(self, input):
        return (self.x_spline(input), self.y_spline(input))

    def get_farhest_arc_length(self):
        return int(self.supporting_points[-1][0])


class A10_1:
    def __init__(self):
        rospy.init_node("a10", anonymous=True)
        self.lane1 = self.load_np_array('lane1')
        self.lane2 = self.load_np_array('lane2')
        assert self.lane1 != self.lane2

        # [[lane1], [lane2]]
        self.supporting_lanes = self.pick_supporting_lanes()
        self.lane1_length = len(self.supporting_lanes[0])
        self.lane2_length = len(self.supporting_lanes[1])

        #self.show_map(extra_point=self.supporting_lanes)
        # TODO: check again
        self.spline_l1 = self.gen_spline(0)
        self.spline_l2 = self.gen_spline(1)

        # --- PUBS ---
        self.pub_marker = rospy.Publisher('visualization_msgs/Marker', Marker, queue_size=10)

        self.rate = rospy.Rate(1)


    def spline_lanes(self):
        """
        Use created splines from constructor to build lanes
        """
        # https://answers.ros.org/question/11135/plotting-a-markerarray-of-spheres-with-rviz/
        marker_l1 = setup_marker((1.0,0.,0.))
        marker_l2 = setup_marker((0.,1.,0.))

        is_lane_1 = True
        point = None
        for lane in self.supporting_lanes: # iterating over both lanes
            marker_l1.points = []
            marker_l2.points = []
            for i in range(self.lane1_length if is_lane_1 else self.lane2_length):
                #point = np.array([self.spline_l1[0](i), self.spline_l1[1](i)])
                point = np.array(self.spline_l1.spline_it(i))
                marker_l1.points.append(self.create_point(point[0], point[1]))
                #point = np.array([self.spline_l2[0](i), self.spline_l2[1](i)])
                self.pub_marker.publish(marker_l1)
                point = np.array(self.spline_l2.spline_it(i))
                marker_l2.points.append(self.create_point(point[0], point[1]))
                self.pub_marker.publish(marker_l2)
            is_lane_1 = not is_lane_1


    def gen_spline(self, lane):
        assert lane == 0 or lane == 1
        return LaneSpline(self.supporting_lanes[lane])

    def gen_spline_old(self, lane):
        """
        @param lane: Lane for which X & Y Spline should be constructed
        @return A tuple of both created splines

        https://docs.scipy.org/doc/scipy-0.18.1/reference/generated/scipy.interpolate.CubicSpline.html
        [[x0,x1,x2,...], [y0,y1,y2,...]]
        """
        xs, ys = [], []
        for point in self.supporting_lanes[lane]:
            xs.append(point[0])
            ys.append(point[1])

        # first x value == last x value
        ys[-1] = ys[0]
        assert len(xs) == len(ys)
        x_spline =  scipy.interpolate.CubicSpline(
        np.array(xs), np.array(ys), bc_type='periodic')

        ys = []
        for point in self.supporting_lanes[lane]:
            ys.append(point[2])

        # first point == last point
        ys[-1] = ys[0]
        assert len(xs) == len(ys)
        y_spline = scipy.interpolate.CubicSpline(
        np.array(xs), np.array(ys), bc_type='periodic')

        return (x_spline, y_spline)

    def pick_supporting_lanes(self):
        """
        Picks points for spline intertolation
        @return [[supporting points for lane 1], [supporting points for lane 2]]
        """
        num_sup_points = (len(self.lane1)/50, len(self.lane2)/50)
        sups_per_lane = [[],[]]
        for i in range(0, len(self.lane1), num_sup_points[0]):
            sups_per_lane[0].append(self.lane1[i])
        for i in range(0, len(self.lane2), num_sup_points[1]):
            sups_per_lane[1].append(self.lane2[i])

        sups_per_lane[0].append(self.lane1[len(self.lane1)-1])
        sups_per_lane[1].append(self.lane2[len(self.lane2)-1])
        return [np.array(sups_per_lane[0]), np.array(sups_per_lane[1])]


    def pick_supporting_lanes_old(self, lane1, lane2):
        """
        Picks Points of each lane
        """
        # TODO: Outer Lane has more points then inner
        res_all, res_split = [], [[],[]]
        indexes = range(len(lane1))
        for l1, l2, i in zip(lane1, lane2, indexes):
            if i % 30 == 0:
                res_all.append(l1)
                res_all.append(l2)
                res_split[0].append(l1)
                res_split[1].append(l2)
        #self.show_map(extra_point=res)
        return res_split

    def create_point(self, x, y):
        assert type(x) is np.float64 and type(y) is np.float64
        return Point(x,y,0)

    def load_np_array(self, lane):
        """
        Loads the arrays given as attachment
        """
        ros_package_path = os.environ['ROS_PACKAGE_PATH']
        path = ""
        paths = ros_package_path.split(':')
        for elem in paths:
            if 'assignment10' in elem:
                path = elem + '/src/' + lane + '.npy'
        assert exists(path)
        return np.load(path)

    def show_map(self, extra_point=None, draw_map=True):
        """
        just prints information from lane files
        """
        xs, ys, extra_x, extra_y = [],[],[],[]
        for el in self.lane1:
            xs.append(el[1])
            ys.append(el[2])
        for el in self.lane2:
            xs.append(el[1])
            ys.append(el[2])
        if extra_point is not None:
            for el in extra_point:
                extra_x.append(el[1])
                extra_y.append(el[2])
        if draw_map:
            plt.scatter(xs, ys)
        plt.scatter(extra_x, extra_y)
        plt.show()

    def write_lanes_to_f_to_f(self):
        with open('lanes', mode='w') as f:
            f.write(str(self.lane1))
            f.write("##### ---- ##### \n\n\n\n")
            f.write(str(self.lane2))

    def get_splines(self):
        return (self.spline_l1, self.spline_l2)

class A10_2:

    def __init__(self, splines):
        self.counter = 0
        self.color_clicked = (0.,0.,1.)
        self.color_closest = (1.,1.,0.)
        self.spline_l1, self.spline_l2 = splines

        # --- SUBS ---
        self.clicked_point_subscriber = rospy.Subscriber("/clicked_point", PointStamped, self.clicked, queue_size=1)

        # --- PUBS ---
        self.pub_clicked = rospy.Publisher('visualization_msgs/Marker', Marker, queue_size=1)
        self.pub_closesed = rospy.Publisher('visualization_msgs/Marker2', Marker, queue_size=1)
        self.fake_click_point = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)


    # FIX: Client [/a10_9133_1578846935063] wants topic /clicked_point to have datatype/md5sum [geometry_msgs/Point/4a842b65f413084dc2b10fb484ea7f17], but our version has [geometry_msgs/PointStamped/c63aecb41bfdfd6b7e1fac37c7cbe7bf]. Dropping connection.
    def clicked(self, p):
        self.pub_clicked_point(p)
        self.pub_closesed_point(p)


    def pub_closesed_point(self, p):
        x,y = p.point.x, p.point.y
        closest_point = None
        m = setup_marker(self.color_closest, id=self.get_counter(), maker_type=Marker.SPHERE, scale=(0.2,0.2))
        m.scale.z = 0.2

        best_distance = [10**10, 10**10]
        best_point = [None, None]
        for i, spline in enumerate((self.spline_l1, self.spline_l2)):
            m.id = self.get_counter()
            for k in range(spline.get_farhest_arc_length()*1000):
                res = spline.spline_it(k/1000.)
                distance =  self.distance_between_points((x,y), res)
                if distance < best_distance[i]:
                    best_distance[i] = distance
                    best_point[i] = res
                    d00('new_best_point!', 'distance:', best_distance[i], 'point: ', best_point[i])
            m.pose.position.x = best_point[i][0]
            m.pose.position.y = best_point[i][1]
            m.pose.position.z = p.point.z
            self.pub_closesed.publish(m)
            # d00("clicked: ", p.point, 'posted: ', m.pose.position)

    def pub_clicked_point(self, p):
        m = setup_marker(self.color_clicked, id=self.get_counter(), maker_type=Marker.SPHERE, scale=(0.2,0.2))
        m.scale.z = 0.2
        m.pose.position.x = p.point.x
        m.pose.position.y = p.point.y
        m.pose.position.z = p.point.z
        self.pub_clicked.publish(m)

    def distance_between_points(self, p1,p2):
        assert len(p1) == len(p2) == 2
        p1 = np.array(p1)
        p2 = np.array(p2)
        return norm(p1-p2)

    def get_counter(self):
        self.counter += 1
        return self.counter

    def fake_point_click(self):
        fake = PointStamped()
        fake.header.seq = 2*10
        fake.header.frame_id = "fake"
        fake.header.stamp = Time(1)
        fake.point = Point()
        fake.point.x = 3.0
        fake.point.y = 2.0
        fake.point.z = 0.
        self.fake_click_point.publish(fake)


if __name__ == '__main__':
    cool = A10_1()
    clickedyclick = A10_2(cool.get_splines())
    clickedyclick.fake_point_click()
    while not rospy.is_shutdown():
        cool.spline_lanes()
        rospy.sleep(1)
        #pass
