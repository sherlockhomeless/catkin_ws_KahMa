#!/usr/bin/env python

# --- imports ---
import rospy
from sensor_msgs.msg import CameraInfo

def extract_camera_info():
    rospy.init_node("print_camera_info", anonymous=True)
    sub = rospy.Subscriber("/sensors/camera/infra1/camera_info", CameraInfo, print_info, queue_size=10)
    while not rospy.is_shutdown():
        pass

def print_info(data):
    print(data)



if __name__ == "__main__":
    try:
        extract_camera_info()
    except rospy.ROSInterruptException:
        pass
