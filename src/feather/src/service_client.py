#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from feather.srv import *

def add_two_ints_client(x):
    rospy.wait_for_service('send_command')
    try:
        add_two_ints = rospy.ServiceProxy('send_command', SendCommand)
        resp1 = add_two_ints(x)
        return resp1.check
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
	add_two_ints_client(2)
	"""
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
	"""
