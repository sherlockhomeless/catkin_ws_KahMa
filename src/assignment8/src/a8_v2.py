#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from autominy_msgs.msg import Speed, SteeringAngle
from nav_msgs.msg import Odometry
from math import acos, sin, tan, cos


"""
deadline: 23:50
"""
# ----- STATE KEEPING -----
initialized = False
cur_speed = 0.0
cur_steer = 0.0
cur_theta = 0.0
cur_x = 0.0
cur_y = 0.0

l = 0.27


def got_speed(speed_msg):
    global cur_speed
    cur_speed = speed_msg.value
def got_steering(steer_msg):
    global cur_steer
    cur_steer = steer_msg.value
def got_init(odo):
    global initialized
    global cur_x
    global cur_y
    global cur_theta

    if initialized == False:
        cur_x = odo.pose.pose.position.x
        cur_y = odo.pose.pose.position.y
        # Angular Representation: 21/31 Quaternion
        cur_theta = acos(odo.pose.pose.orientation.w) * 2
        initialized = True

# ----- ROSPY SHIT ------
rate = 100.0
rospy.init_node("ue8_v2", anonymous = True)
r = rospy.Rate(rate)

# ------ SUBS ------
sub_speed = rospy.Subscriber("/sensors/speed", Speed, got_speed, queue_size=10)
sub_steering = rospy.Subscriber("/sensors/steering", SteeringAngle, got_steering , queue_size=10)
sub_odo = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, got_init, queue_size=10)

# ------ PUBS ------

pub = rospy.Publisher("/nav_msgs/Odometry", Odometry, queue_size = 10)



if __name__ == "__main__":
    while not initialized:
        rospy.sleep(1)

    while not rospy.is_shutdown():
        odo = Odometry()
        odo.header.frame_id = "map"
        odo.child_frame_id = "base_link"

        v = cur_speed
        phi = cur_steer
        delta_t = 1.0 / rate

        print(v, phi, delta_t)

        # siehe Aufgabenblatt
        x_dot = v * cos(cur_theta)
        y_dot = v * sin(cur_theta)
        theta_dot = (v / l) * tan(phi)

        #print(x_dot, y_dot, theta_dot)

        new_x = cur_x + delta_t * x_dot
        new_y = cur_y + delta_t * y_dot
        new_theta = cur_theta + delta_t * theta_dot

        #print(new_x, new_y, new_theta)

        odo.pose.pose.position.x = new_x
        odo.pose.pose.position.y = new_y
        odo.pose.pose.position.z = 1.0

        odo.pose.pose.orientation.w = cos(new_theta/2)
        # rostopic echo /sensors/localization/filtered_map
        odo.pose.pose.orientation.z = sin(new_theta/2)

        pub.publish(odo)
        print("published")

        cur_x, cur_y, cur_theta = new_x, new_y, new_theta
        rospy.sleep(delta_t)
