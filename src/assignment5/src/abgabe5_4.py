#!/usr/bin/env python

import rospy
from math import sqrt, tan
import numpy as np
from nav_msgs.msg import Odometry
# https://github.com/AutoMiny/AutoMiny/tree/master/catkin_ws/src/autominy_msgs/msg
from autominy_msgs.msg import SteeringPWMCommand, SpeedCommand, SteeringPWMCommand

# --- helper function that just returns an list of lists of length n ---
def give_n_list(n):
        list = []
        for i in range(n):
            list.append([])
        return list

class CircleDriver:
    def __init__ (self):
        # [[x0,x1,x2,...],[y0,y1,y2,...],[z0,z1,z2,...]]
        self.coordinate_history = give_n_list(3)
        # used to keep track for calculating distance driven
        self.last_coordinate_seen = None
        rospy.init_node("assignment_5", anonymous=True)
        # Subscriber gets gps from ceilings camera
        # topic for Subscriber /communication/gps/<ID>
        # TODO: enter correct ID
        self.gps_subscriber = rospy.Subscriber("/communication/gps/5", Odometry, self.get_points, queue_size=10)
        # Subscriber to voltage information of motor
        self.steering_angle = rospy. Subscriber("/sensors/arduino/steering_angle", SteeringPWMCommand, self.print_voltage, queue_size=10)
        # Pubisher publishes steering angle
        self.steer_publisher = rospy.Publisher("/actuators/steering_pwm", SteeringPWMCommand,queue_size=10)
        self.steer_cmd = SteeringPWMCommand()
        # Publisher publishes speed
        self.speed_publisher = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.speed_cmd = SpeedCommand()

        # The spin() keeps things going
        #rospy.spin()
        self.rate = rospy.Rate(10) # 10ghz

    # saves points received by the camera to tracking file
    def get_points(self, odo):
        self.coordinate_history[0].append(odo.pose.pose.position.x)
        self.coordinate_history[1].append(odo.pose.pose.position.y)
        self.coordinate_history[2].append(odo.pose.pose.position.z)
        #self.print_latest_coordinates()

    # prints voltage information of motor
    def print_voltage(self, pwm):
        #print("voltage: " + str(pwm.value))
        pass

    # prints the most current coordinates received by ceiling camera
    def print_latest_coordinates(self):
        print( "x:" + str(self.coordinate_history[0][-1]))
        print( "y:" + str(self.coordinate_history[1][-1]))

    # gets the latest gps-coordinates from the history
    def get_latest_coordinates(self):
        return (self.coordinate_history[0][-1], self.coordinate_history[1][-1], self.coordinate_history[2][-1])

    # drives car in circle and meassures three points for each 1s
    def drive_circle(self, steering, speeding):
        point_list = []
        #starting position
        point_list.append(self.get_latest_coordinates())
        self.start_drive_in_circle(steering, speeding)
        self.short_stop_car()
        rospy.sleep(5)
        # 2nd point
        point_list.append(self.get_latest_coordinates())
        self.short_stop_car()
        # 3rd point
        point_list.append(self.get_latest_coordinates())
        return point_list

    def start_drive_in_circle(self, steering, speeding):
        self.steer_cmd.value = steering
        self.speed_cmd.value = speeding
        self.steer_publisher.publish(self.steer_cmd)
        self.speed_publisher.publish(self.speed_cmd)


    def short_stop_car(self):
        cur = self.speed_cmd.value
        self.speed_cmd.value = 0
        self.speed_publisher.publish(self.speed_cmd)
        rospy.sleep(5)
        self.speed_cmd.value = 0.5
        self.speed_publisher.publish(self.speed_cmd)

    def stop_driving_circle(self):
        self.speed_cmd.value = 0
        self.steer_cmd.value = 0
        self.steer_publisher.publish(self.steer_cmd)
        self.speed_publisher.publish(self.speed_cmd)

    def reset_coord_history(self):
        self.coordinate_history = give_n_list(3)

    def update_distance_traveled(self):
        while len(self.coordinate_history) > 0:
            old_coordinate = self.last_coordinate_seen
            new_coordinate = self.coordinate_history[0]
            del(self.coordinate_history[0])
            # distance = sqrt((x1-x0)^2 + (y1-y0)^2)
            distance = sqrt((new_coordinate[0] - old_coordinate[0])**2 + (new_coordinate[1] - old_coordinate[1])**2)
            self.last_coordinate_seen = new_coordinate


    def drive_distance_in_circle(self, distance):
        self.reset_coord_history()
        distance_traveled = 0
        while(distance_traveled < distance):
            self.update_distance_traveled()
        self.stop_driving_circle()

    # Ackerman steering
    # ai = tan(wheelbase/(radius - track/2))^-1
    # a0 = tan(wheelbase/(radius + track/2))^-1
    # angle = (ai -a0)/2 + a0
    # http://datagenetics.com/blog/december12016/index.html
    def calculate_steering_anlge(wheelbase, track, radius):
        ai = tan(wheelbase/(radius - track/2))**-1
        a0 = tan(wheelbase/(radius + track/2))**-1
        return (ai - a0)/2 +a0

    def calculate_turning_radius(point0, point1, point2):
        x0 = point0[0]
        y0 = point0[1]
        x1 = point1[0]
        y1 = point1[1]
        x2 = point2[0]
        y2 = point2[1]

        a = np.array([[2*(x1-x0), 2*(y1-y0)], [2*(x2-x0), 2*(y2-y0)]])
        b = np.array([(x1**2 - x0**2) - (y1**2 - y0**2), (x2**2 - x0**2) -(y2**2-y0**2)])
        result = np.linalg.solve(a,b)
        x = result[0]
        y = result[1]

        r = sqrt(x**2 - 2*x*x0 + x0**2 + y**2 - 2*y*y0 +y0**2)
        return r


    steering_angles = (-1,0,1)

if __name__ == '__main__':

    # --- find out the radius here by driving and taking three random point measssurements to calculate circle radius --- #
    cdriver = CircleDriver()
    print("taking meassurement points")
    rospy.sleep(2)
    meassured_points = cdriver.drive_circle(1,0.3)
    rospy.sleep(5)
    print("3 points meassuered: " + str(meassured_points))
    cdriver.stop_driving_circle()
    r = calculate_turning_radius(meassured_points[0], meassured_points[1], meassured_points[2])
    print("radius: " + str(r))
    print("first:" +  str(cdriver.coordinate_history[0][0]))
    print("middle:" +  str(cdriver.coordinate_history[0][len(cdriver.coordinate_history[0])/2]))
    print("last:" +  str(cdriver.coordinate_history[0][-1]))

    # drive half a circle by using radius
    distance_half_circle = (numpy.pi * 2 * r)/2
    cdriver.drive_distance_in_circle(distance_half_circle)

    if drive_half_circle:
        rospy.sleep(5)
