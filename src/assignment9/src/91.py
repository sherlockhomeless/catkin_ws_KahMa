#!/usr/bin/env python
# coding=utf8

import rospy
from autominy_msgs.msg import Speed, SteeringAngle, SpeedCommand, NormalizedSteeringCommand
from nav_msgs.msg import Odometry

from math import atan, asin, atan2
import pdb

# ------ SET CAR GPS ID ------
CAR_GPS_ID = 7
MAX_STEER = 30
DEBUG = True
rospy.init_node("a9", anonymous=True)

class Quanternion:
    def __init__(self, odo):
        assert type(odo) is Odometry
        o = odo.pose.pose.orientation
        self.w = o.w
        self.x = o.x
        self.y = o.y
        self.z = o.z

    def get_euler_angles(self):
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        # maybe problem with angle with more/less then pi/2
        denominator = 2*(self.w*self.x + self.y*self.z)
        enumerator = 1 - 2*(self.x**2+self.y**2)
        phi = atan2(denominator, enumerator)

        theta = asin(2*(self.w*self.y - self.z*self.x))

        denominator = 2*(self.w * self.z + self.x * self.y)
        enumerator = 1-2*(self.y**2 + self.z**2)

        psi = atan2(denominator, enumerator)

        return (phi, theta, psi)


class PD_Controller():
    def __init__(self, target_yaw_angle, gps_id):


        # --- CONFIGURE --- #
        self.K_p = 1.5
        self.K_d = 1.0
        #self.K_i = 0
        self.target_yaw_angle = target_yaw_angle
        self.logF = open("91.log", "w")
        rospy.on_shutdown(self.shutdown)
        self.rate = 5.0 # MUST BE float

        # --- KEEP STATE --- #
        self.inited = False
        self.cur_speed = 0.0
        self.yaw_differences = [0.0, 0.0]
        self.integral_error = 0.0

        # --- SUBS --- #
        self.cur_steer = 0.0
        self.cur_yaw_angle = 0.0
        self.sub_steer = rospy.Subscriber("/sensors/steering", SteeringAngle, self.got_steering, queue_size = 25)
        self.sub_gps = rospy.Subscriber("/communication/gps/" + str(gps_id), Odometry, self.get_orientation_from_gps, queue_size = 10)

        # --- PUBS --- #
        self.pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
        self.pub_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)

    def start_yaw_correction(self):
        global MAX
        while not self.inited:
            continue
        while self.cur_yaw_angle != self.target_yaw_angle:
            u = self.do_pd()
            self.steer(u)
            rospy.sleep(1/self.rate)

        print("DONE CORRECTION (cur/target)", self.cur_yaw_angle, self.target_yaw_angle)

    def do_pd(self):

        # https://forum.arduino.cc/index.php?topic=433030.0
        # http://www.chemgapedia.de/vsengine/vlu/vsc/de/ch/7/tc/regelung/grundlagen/regelung_grundlagen.vlu/Page/vsc/de/ch/7/tc/regelung/grundlagen/regler/pid_ctrl.vscml.html

        x_star = self.target_yaw_angle
        # ~ -1 - 1
        x = self.cur_yaw_angle
        yaw_divergence = x_star - x
        self.yaw_differences.append(yaw_divergence)
        ouput_p = self.K_p * yaw_divergence

        self.log("P/D")
        # Change of error
        x_deri_star = yaw_divergence
        x_deri = self.yaw_differences[-2] # pick 2nd newest error
        output_d = (x_deri_star - x_deri) * self.K_d

        #self.integral_error += yaw_divergence * 1/self.rate
        #output_i = self.integral_error * self.K_i

        self.log(str(ouput_p) + "/" + str(output_d))
        return  ouput_p + output_d #+ output_i

    def steer(self, angle):
        steer_msg = NormalizedSteeringCommand()
        if angle > 1:
            angle = 1
        if angle < -1:
            angle = -1
        steer_msg.value = angle
        self.pub_steering.publish(steer_msg)
        self.log("STEER-ANGLE: " + str(angle))

    def get_orientation_from_gps(self, odo):
        quanternion = Quanternion(odo)
        self.cur_orientation = quanternion.get_euler_angles()
        self.cur_yaw_angle = self.cur_orientation[2]
        #print("CURRENT YAW ANGLE", self.cur_yaw_angle)
        self.inited = True

    def got_steering(self, steer_msg):
        self.cur_steer = steer_msg.value

    def drive(self, speed):
        speed_msg = SpeedCommand()
        speed_msg.value = speed
        rospy.sleep(1)
        self.pub_speed.publish(speed_msg)
        print("SPEED_CMD:", speed)

    def stop_drive(self):
        self.drive(0)

    def log(self, msg):
        self.logF.write(msg + '\n')

    def shutdown(self):
        pass



if __name__ == "__main__":
    pd = PD_Controller(0.0, CAR_GPS_ID)
    pd.drive(0.5)
    pd.start_yaw_correction()
    pd.stop_drive()



"""
pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
speed_msg = SpeedCommand()
speed_msg.value = 0.2
pub_speed.publish(speed_msg)
"""
