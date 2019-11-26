#!/usr/bin/env python
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand, Speed

class D:
    def __init__(self):
        print("initializing Driver")
        rospy.init_node("assignment_6", anonymous = True)
        self.rate = rospy.Rate(10)
        self.steer_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
        self.speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.steer_cmd = NormalizedSteeringCommand()
        self.speed_cmd = SpeedCommand()
        self.speed_sub = rospy.Subscriber("/actuators/speed", Speed, self.print_speed_given, queue_size=10)
        print("initialized complete")

    def print_speed_given(self, data):
        print("received msg from speed topic, speed is: " +  str(data.value))


    def drive(self, steering, speed):
        print("START DRIVING WITH STEERING/SPEED: " + str(steering) + "/" + str(speed))
        self.steer_cmd.value = steering
        self.speed_cmd.value = speed
        self.steer_pub.publish(self.steer_cmd)
        self.speed_pub.publish(self.speed_cmd)
        rospy.sleep(2/speed)

    def stop_driving(self):
        self.speed_cmd.value = 0
        self.steer_cmd.value = 0
        self.steer_pub.publish(self.steer_cmd)
        self.speed_pub.publish(self.speed_cmd)

    def drive_straight(self):
        self.drive(0, 0.3)
        self.stop_driving()

    def drive_right(self):
        self.drive(-1, 0.3)
        self.stop_driving()

    def drive_left(self):
        self.drive(1, 0.3)
        self.stop_driving()

if __name__ == "__main__":
    d = D()
    while not rospy.is_shutdown():
        rospy.sleep(5)
        print("prepare drive straight")
        rospy.sleep(1)
        d.drive_straight()
        rospy.sleep(15)
        print("prepare drive left")
        rospy.sleep(1)

        d.drive_left()
        rospy.sleep(15)
        print("prepare drive right")
        rospy.sleep(1)
        d.drive_right()
