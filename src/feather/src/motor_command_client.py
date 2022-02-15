#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from feather.srv import *
import time

def send_command_client(com,v1,v2):
    rospy.wait_for_service('send_command')
    try:
        client = rospy.ServiceProxy('send_command', SendCommand)
        resp1 = client(v1,v2,com)
        return resp1.check
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":

	send_command_client(4,230,230)
	time.sleep(5)
	send_command_client(5,230,230)
	time.sleep(5)
	send_command_client(3,230,230)
	time.sleep(5)
	send_command_client(5,230,230)
	time.sleep(5)
	

    
