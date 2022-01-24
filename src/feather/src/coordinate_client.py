#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
from feather.srv import *

def send_coordinates(x, y):
    rospy.wait_for_service('coordinate_service')
    try:
        coordinate_service = rospy.ServiceProxy('coordinate_service', PathPlanning)
        resp1 = coordinate_service(x, y)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
	send_coordinates(2,3)
