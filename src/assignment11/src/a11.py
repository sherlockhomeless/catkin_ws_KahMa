#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PointStamped
from autominy_msgs import NormalizedSteeringCommand
from autominy_msgs.msg import SteeringCommand
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int16
from tf.transformations import euler_from_quaternion
import numpy as np

from math import degrees, atan2, sqrt, sin, cos, asin

def d(*msg):
    print(msg)

class Connector:
    """
    Exercise 11:
    This class is used to combine the output of map and pid by:
    1. It looks at the next lookahead point and the current position
    2. Calculates the required yaw angle to drive from current position to steering angles
    3. Sends Angle to Steering-PID using topic
    """
    def __init__(self, debug=False):

        self.debug = debug
        rospy.init_node("A11", anonymous=True)
        # has (x,y)
        self.target_angle = None
        # has odometry object
        self.current_position = None
        self.lookahead_point = [None,None]
        self.current_lane = 0

        # --- SUBS ---
        self.sub_lookahead_0 = rospy.Subscriber("/lookahead_0", Marker, self.got_lookahead)
        self.sub_lookahead_1 = rospy.Subscriber("/lookahead_1", Marker, self.got_lookahead)
        self.sub_odo = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.got_position, queue_size=10)
        self.sub_cur_lane = rospy.Subscriber("/current_lane", Int16, self.got_lane)

        # --- PUBS ---
        self.pub_angle = rospy.Publisher("/desired_angle", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/actuators/steering", SteeringCommand, queue_size=10)

    def calc_target_angle(self):
        if self.current_position is None: return
        current_position = np.array([self.current_position.pose.pose.position.x, self.current_position.pose.pose.position.y])
        x = self.lookahead_point[self.current_lane][0] - current_position[0]
        #x = self.lookahead_point[0]
        y = self.lookahead_point[self.current_lane][1] - current_position[1]
        #y = self.lookahead_point[1]
        sigma = atan2(y,x)

        # TODO: check if euler_from_quaternion returns radions or degrees
         _,_,theta = euler_from_quaternion(self.build_quaternion())
        print("sigma: ", sigma, "theta: ", theta)
        dif = sigma - theta

        # if dif > 0:
        #     abs(sigma)
        # elif dif == 0:
        #     sigma = 0.
        self.steer_pub.publish(steer)
        msg = Float64()
        msg.data = dif
        self.pub_angle.publish(msg)
        if self.debug: print("published new target angle: ", msg.data)

    def got_lookahead_0(self, marker):
        self.got_lookahead(marker, 0)

    def got_lookahead_1(self, marker):
        self.got_lookahead(marker, 1)

    def got_lookahead(self, marker, lane):
        # if False: print("got marker", '(', marker.pose.position.x, marker.pose.position.y,  ')' )
        self.lookahead_point[lane] = (marker.pose.position.x, marker.pose.position.y)
        if lane == self.current_lane:
            self.calc_target_angle()

    def got_position(self, odo):
        self.current_position = odo

    def got_lane(self, lane_int):
        self.current_lane = lane_int

    def build_quaternion(self):
        ori = self.current_position.pose.pose.orientation
        return [ori.x, ori.y, ori.z, ori.w]


if __name__ == '__main__':
    a11 = Connector()
    while not rospy.is_shutdown():
        pass
