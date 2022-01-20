#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from feather.srv import *

def add_two_ints_client(x, y, z):
    rospy.wait_for_service('add_3')
    try:
        add_two_ints = rospy.ServiceProxy('add_3', AddThreeInts)
        resp1 = add_two_ints(x, y, z)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    x = 1
    y = 34
    z = 3
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y, z)))
