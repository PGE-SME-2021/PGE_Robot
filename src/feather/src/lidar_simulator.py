#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Int32MultiArray

from random import randrange

def publisher():
    pub = rospy.Publisher("lidar_points", Int32MultiArray, queue_size = 10)
    rate = rospy.Rate(3)#1Hz
    counter = 0
    battery_level = 50
    msg_to_publish = Int32MultiArray()
    points_number = 70
    while not rospy.is_shutdown():
        points = []
        p = [4, 4]
        points.append(p)
        for i in range(points_number):
            delta_x = randrange(-5, 5)
            delta_y = randrange(-5, 5)
            p = [p[0] + delta_x, p[1] + delta_y]
            points.append(p)

        msg_to_publish.data = points
        pub.publish(msg_to_publish)
        rospy.loginfo(points)
        time.sleep(3)

if __name__ == "__main__":
	rospy.init_node("lidar_sim")
	publisher()
