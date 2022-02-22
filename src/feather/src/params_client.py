#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from feather.srv import SetParams

## Service client for semiautomatic mode
# This sends the coordinates to the server (params_server.cpp)
# @param max_speed Set maximum speed of the robot
# @param max_acceleration Set maximum acceleration of the robot
# @param max_angular_speed set maximum angular speed of the robot
def set_params(
        max_speed,
        max_acceleration,
        max_angular_speed
        ):
    rospy.wait_for_service('params_server')
    try:
        params_server = rospy.ServiceProxy(
                'params_server',
                SetParams
                )
        check = params_server(
            max_speed,
            max_acceleration,
            max_angular_speed
            )
        return check.check
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    """Main function
    This function tests the functions below
    """
    set_params(2,3,5)
