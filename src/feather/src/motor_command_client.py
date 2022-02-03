#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from feather.srv import *

def send_command_client(x):
    rospy.wait_for_service('send_command')
    try:
        client = rospy.ServiceProxy('send_command', SendCommand)
        resp1 = client(x)
        return resp1.check
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
	send_command_client(2)
