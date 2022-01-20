#!/usr/bin/env python
from __future__ import print_function
from distutils import command
import string

import sys
import rospy
from feather.srv import *

def send_commands_client(com):
    rospy.wait_for_service('send_command')
    try:
        command = rospy.ServiceProxy('send_command', SendCommand)
        resp1 = command(com)
        return resp1.check
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [com]"%sys.argv[0]

if __name__ == "__main__":
    # if len(sys.argv) == 2:
    #     com = int(sys.argv[1])
    # else:
    #     print(usage())
    #     sys.exit(1)
    com = 25
    print("Requesting %s"%(com))
    print("%s is %s"%(com, send_commands_client(com)))
    