import rospy
import numpy as np

def get_gps_position():
    sub = rospy.Subscriber("/communication/gps/15", 



def do_smth():
    rospy.init_node('assignment5', anonymous=True)
    1msr = get_gps_position()
    angles = [-1.0, 0.0, 1.0]


if __name__ == '__main__':
    do_smth()
