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

if __name__ == "__main__":
    msg_to_send = "50,30,0,0"
	add_two_ints_client(msg_to_send)
