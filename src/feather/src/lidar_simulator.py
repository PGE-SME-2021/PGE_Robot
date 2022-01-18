#!/usr/bin/env python3
import rospy
import time
#from std_msgs.msg import Int32MultiArray
from feather.msg import LidarData

from random import randrange

def publisher():
    pub = rospy.Publisher("lidar_points", LidarData, queue_size = 10)
    rate = rospy.Rate(3)#1Hz
    msg_to_publish = LidarData()
    points_number = 80
    while not rospy.is_shutdown():
        x = 0
        y = 0
        for i in range(points_number):
            delta_x = randrange(-5, 5)
            delta_y = randrange(-5, 5)
            x += delta_x
            y += delta_y
            msg_to_publish.points[i].x = x
            msg_to_publish.points[i].y = y
            msg_to_publish.points[i].z = 0

        pub.publish(msg_to_publish)
        rospy.loginfo(msg_to_publish)
        time.sleep(3)

if __name__ == "__main__":
	rospy.init_node("lidar_sim")
	publisher()
