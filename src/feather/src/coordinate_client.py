#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from feather.srv import *

def send_coordinates(x, y):
    rospy.wait_for_service('coordinate_service')
    try:
        coordinate_service = rospy.ServiceProxy('coordinate_service', PathPlanning)
        resp1 = coordinate_service(x, y)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
def pub_coordinates(x,y):
    pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
    rospy.init_node('goal_sender', anonymous=True)
    rate = rospy.Rate(1)
    goal_position = PoseStamped()
    while not rospy.is_shutdown():
        goal_position.pose.position.x = x
        goal_position.pose.position.y = y 
        rospy.loginfo("x=%s y=%s", goal_position.pose.position.x, goal_position.pose.position.y)
        pub.publish(goal_position)
        rate.sleep()

if __name__ == "__main__":
	#send_coordinates(2,3)
    pub_coordinates(2,3)