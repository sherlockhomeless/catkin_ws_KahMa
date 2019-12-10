#!/usr/bin/env python
# coding=utf8

import rospy
from autominy_msgs.msg import Speed, SteeringAngle
from nav_msgs.msg import Odometry
import time
from math import cos, sin, tan, acos, pi
from subprocess import Popen
import os

"""
TODOS:
[ ] do theta for real
[ ] fix input for theta_dot call in calc_next_position

"""

bag_files = [
"lanefollowing.bag",
"circle.bag"
"figure8.bag",
]


RATE = 50
DEBUG = True

rospy.init_node("a8", anonymous=True)
r = rospy.Rate(RATE) # 100 Hz

class Ackermann_Odometry:

    def __init__(self):
        # Keep State
        self.coords = [[] for x in range(2)]
        self.steer_angles = []
        self.cur_speed = 0.0
        self.cur_steer = 0.0
        self.cur_theta = 0.0
        self.last_x, self.last_y, self.last_theta = 0.0, 0.0, 0.0
        self.cur_odo = None
        self.msg_count = 0
        self.inited = False

        # Subscriber
        self.sub_steer = rospy.Subscriber("/sensors/steering", SteeringAngle, self.got_steering, queue_size = 25)
        self.sub_speed = rospy.Subscriber("/sensors/speed", Speed, self.got_speed, queue_size = 25)
        self.sub_filtered_map = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.got_filtered_map, queue_size = 10)

        # Publisher
        self.pub_odo = rospy.Publisher("/nav_msgs/Odometry", Odometry, queue_size = 20)

        # Car-Constants
        self.L = 0.27

    def calc_next_position(self):
        global RATE
        odo = Odometry()
        odo.header.frame_id = 'map'
        odo.child_frame_id = 'base_link'

        delta_t = 1.0 / RATE
        x_dot = self.calc_x_dot(self.cur_speed, self.cur_theta)
        y_dot = self.calc_y_dot(self.cur_speed, self.cur_theta)
        #theta_dot = self.calc_theta_dot(self.cur_speed, self.cur_theta)
        theta_dot = self.calc_theta_dot(self.cur_speed, self.cur_steer)

        new_x = self.last_x + delta_t * x_dot
        new_y = self.last_y + delta_t * y_dot
        new_theta = self.last_theta + delta_t * theta_dot

        odo = self.set_params(odo, new_x, new_y, new_theta)

        self.pub_odo.publish(odo)

        self.last_x = new_x
        self.last_y = new_y
        self.last_theta = new_theta

        if DEBUG: print("NEW ODO:( ", new_x, new_y, new_theta, ')')
        self.cur_odo = odo
        self.msg_count -=- 1


    def set_params(self, odo, x, y, theta):
        odo.pose.pose.position.x = x
        odo.pose.pose.position.y = y
        odo.pose.pose.position.z = 0
        # Sammelmappe S. 157
        odo.pose.pose.orientation.w = theta

        return odo

    def got_steering(self, steerAngle):
        #print("steering angle:", steerAngle.value)
        self.cur_steer = steerAngle.value
        self.steer_angles.append(steerAngle.value)

    def got_speed(self, speed):
        #print("speed:", speed.value)
        self.cur_speed = speed.value

    def got_filtered_map(self, odo):
        if self.inited is False:
            self.inited = True
            self.cur_x = odo.pose.pose.position.x
            self.cur_y = odo.pose.pose.position.y
            self.cur_theta = self.calc_theta(odo)
            self.cur_odo = odo

    def calc_theta_dot(self, v, steering_angle):
        return (v/self.L) * tan(steering_angle)

    # slide 21/32 Quaternion/ Euler Parameters (epsilon0)
    def calc_theta(self, odo):
        w = odo.pose.pose.orientation.w
        theta = acos(w) * 2
        return theta

    def calc_x_dot(self, v, theta):
        return v * cos(theta)

    def calc_y_dot(self, v, theta):
        #print(v, theta, v * sin(theta))
        return v * sin(theta)


    def max_steer_angle(self):
        max = 0
        for el in self.steer_angles:
            if el > max:
                max = el
        return max

def shutdown_works():
    print "Shutdown cool"


# --- MAIN ---
if __name__ == "__main__":
    ao = Ackermann_Odometry()
    # play rosbag one time
    time_start = int(time.time())
    time_end = time_start + 42 # longest rosbag is ~ 42sec
    p = Popen(['rosbag', 'play', bag_files[0]], stdout=open(os.devnull, 'wb'))
    while not rospy.is_shutdown():
        if ao.inited is False:
            continue

        ao.calc_next_position()
        if int(time.time()) > time_end:
            rospy.signal_shutdown("rosbag played through")
        r.sleep()
    print("MSGS sent: ", ao.msg_count)
    p.terminate()
