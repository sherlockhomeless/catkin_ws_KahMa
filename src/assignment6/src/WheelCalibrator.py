#!/usr/bin/env python

import rospy
from autominy_msgs.msg import Tick
from nav_msgs.msg import Odometry
from math import ceil, sqrt, pi
from time import time
import numpy as np

"""

This node should be able to calculate the distance of a driven trip and comparing that distance with the meassured ticks of the engine.

Since we were having trouble with the live cars we recorded several rosbags that we used for developing.
This Node has to be started before the replay of the rosbag starts.

The idea in the beginning was to implement the Intersecting chords theorem for calculating the distance traveled on a circle (in method length_of_curve_of_two_points). The problem with this attempt was that the car did not drive perfect circles so that we would get
arcin-parameters that were > 1 , which is undefined.

After this attempt failed we implemented the very general idea of just calculating the length between two points meassured, that works for
all the steering_angle cases (1, 0, -1).

We also made the assumption that if a object is published to the ticks topic it is a tick, no matter the value-field. As it can be seen
in the included rosbags, the same value gets published multiple times and even can decrease again.


"""

rosbag_playtime_straight = 7.605473
rosbag_playtime_left = 8.934871
rosbag_playtime_right = 6.928194

# --- SET PARAMETERS HERE ---
rosbag_playtime = rosbag_playtime_left
steering_angle = 1

def give_n_list(n):
        list = []
        for i in range(n):
            list.append([])
        return list


class WheelCalibrator:
    def __init__(self):
        rospy.init_node('a6', anonymous=True)
        # wrapper for being able to use rosbags
        self.cur_time = 0
        self.end_time = 0
        self.received_first_callback = False
        self.stop_receiving = False

        self.sub_ticks = rospy.Subscriber("/sensors/arduino/ticks", Tick, self.register_tick, queue_size = 10)
        self.count_ticks = 0
        self.sub_gps = rospy.Subscriber("/communication/gps/7", Odometry, self.register_gps, queue_size = 10)
        # [[x0,x1,...], [y0,y1,...],[timestamp0, timestamp1,...]]
        self.coordinate_history = [[],[],[]]
        self.distance_driven = 0

    # just counts ticks received
    def register_tick(self, data):
        global rosbag_playtime
        self.count_ticks += 1
        # useful when using loop option with rosbag play
        if self.stop_receiving:
            return
        if self.received_first_callback is False:
            self.end_time = int(time()) + rosbag_playtime
            self.received_first_callback = True
        self.cur_time = int(time())
        if self.end_time - self.cur_time <= 0:
            self.stop_receiving = True
            self.wheel_calibration()

    # just writes all received coordinates to a coordinate-history
    def register_gps(self, odo):
        if not self.stop_receiving:
            self.coordinate_history[0].append(odo.pose.pose.position.x)
            self.coordinate_history[1].append(odo.pose.pose.position.y)
            self.coordinate_history[2].append(odo.header.stamp.secs)

    #  Returns the point stored at the given index in coordinate history
    def get_point_from_index(self, index):
        return (self.coordinate_history[0][index], self.coordinate_history[1][index], self.coordinate_history[2][index])

    # Returns the first point that can be found in coordinate history with the given second-timestamp
    def get_first_point_for_timestamp(self, timestamp):
        length_gps_history = len(self.coordinate_history[0])
        for i in range(length_gps_history):
            if self.coordinate_history[2][i] == timestamp:
                return (self.coordinate_history[0][i], self.coordinate_history[1][i], self.coordinate_history[2][i])

    # Picks 3 points from coordinate history to do circle calculations
    def pick_three_points(self):
        p1 = self.get_point_from_index(0)
        p1_timestamp = p1[2]
        p2_timestamp = p1_timestamp + 1
        p3_timestamp = p1_timestamp + 2
        p2 = self.get_first_point_for_timestamp(p2_timestamp)
        p3 = self.get_first_point_for_timestamp(p3_timestamp)
        return p1, p2, p3
    # Returns the distance between two points
    def distance_of_two_points(self,p1,p2):
        return sqrt((p2[0] - p1[0])**2 + (p2[1] - p2[1])**2)

    # calculate the length of the curve e between 2 points
    # https://www.dummies.com/education/math/geometry/how-to-determine-the-length-of-an-arc-2/
    def length_of_curve_of_two_points(self,p1,p2):
        radius = self.calc_circle_parameters()[1]
        distance_of_two_points = self.distance_of_two_points(p1,p2)
        sine_half_alpha = distance_of_two_points/(2*radius)
        print(sine_half_alpha)
        angle = np.arcsin(sine_half_alpha)
        assert -1 <= sine_half_alpha <= 1
        return 2*pi*radius*angle/360

    def pick_points_half_second_apart(self):
        list = []
        # add first entry to list
        number_timestamps = 0
        found_timestamps = []
        for i in range(len(self.coordinate_history[0])):
            point = self.get_point_from_index(i)
            if self.get_point_from_index(i)[2] not in found_timestamps:
                found_timestamps.append(point[2])
                number_timestamps += 1

        buckets = give_n_list(number_timestamps)
        # iterate over all points in coordinate_history
        for i in range(len(self.coordinate_history[0])):
            point = self.get_point_from_index(i)
            # TODO: EXTERMLY UGLY

            for bucket in buckets:
                if len(bucket) == 0 or point[2] == bucket[0][2]:
                    bucket.append(point)
        for bucket in buckets:
            list.append(bucket[0])
            list.append(bucket[int(len(bucket)/2)])
        for el in list:
            print(el)
            assert len(el) == 3
        return list

    # Returns every nth point from the meassured gps coordinates
    def pick_nth_point_from_history(self, n):
        length_history = len(self.coordinate_history[0])
        return_list = []
        for i in range(length_history):
            if i%n == 0:
                return_list.append(self.get_point_from_index(i))
        return return_list


    # http://mathforum.org/library/drmath/view/54323.html
    # https://stackoverflow.com/a/50974391
    def calc_circle_parameters(self):
        p1, p2, p3 =  self.pick_three_points()
        """
        Returns the center and radius of the circle passing the given 3 points.
        In case the 3 points form a line, returns (None, infinity).
        """
        temp = p2[0] * p2[0] + p2[1] * p2[1]
        bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
        cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
        det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

        if abs(det) < 1.0e-6:
            return (None, np.inf)

        # Center of circle
        cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
        cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

        radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
        return ((cx, cy), radius)


    # Method doing all the calculations after rosbag played fully
    def wheel_calibration(self):
        global steering_angle
        print("Do something useful here")
        print("# Ticks: {:d}".format(self.count_ticks))
        print("# GPS_Meassures: {:d}".format(len(self.coordinate_history[0])))

        # in case of driving straight forward
        if steering_angle == 0:
            p0 = (self.coordinate_history[0][0], self.coordinate_history[1][0])
            pn = (self.coordinate_history[0][-1], self.coordinate_history[1][-1])
            distance = sqrt((p0[0]-pn[0])**2 - (p0[1]-pn[1])**2)
            print("# Distance driven straight forward: " + str(distance))
            ticks_distance_ratio = distance/self.count_ticks
            print("# Ratio distance/ticks: " + str(ticks_distance_ratio))
        # in case of driving to the left
        if steering_angle == 1 or steering_angle == -1:
            center, radius = self.calc_circle_parameters()
            print("Center: " +  str(center) + "\nRadius: " + str(radius))
            point_list = self.pick_nth_point_from_history(25)
            curve_distance = 0
            distance_driven = 0
            for i in range(len(point_list)-1):
                #distance_driven = self.length_of_curve_of_two_points(point_list[i], point_list[i+1])
                distance_driven += self.distance_of_two_points(point_list[i], point_list[i+1])
                #print("distance driven: " + str(distance_driven))
                #curve_distance += distance_driven
            #print("Curved distance driven: " + str(curve_distance))
            print("Distance driven: " + str(distance_driven))
            print("Distance per Tick: " + str( distance_driven/ self.count_ticks))
            rospy.signal_shutdown("done")


# --- MAIN ---
if __name__ == "__main__":
    wc = WheelCalibrator()
    while not rospy.is_shutdown():
        pass
