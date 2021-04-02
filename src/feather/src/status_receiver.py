#!/usr/bin/env python3
import rospy
from feather.msg import Status
#from std_msgs.msg import String

def subscriber():
    sub = rospy.Subscriber(
            "status",
            Status,
            callback_function
            )
    rospy.spin()

def callback_function(message):
    rospy.loginfo(F"GOT {message.error} and {message.battery}%")


if __name__ == "__main__":
    rospy.init_node("status_receiver")
    subscriber()
