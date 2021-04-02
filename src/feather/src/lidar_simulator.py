#!/usr/bin/env python
import rospy
import time
from csv_functions import *
from feather.msg import LidarData

def publisher():
    pub = rospy.Publisher("lidar_points", LidarData, queue_size = 10)
    rate = rospy.Rate(3)#1Hz
    msg_to_publish = LidarData()
    BASE_DIR = Path(__file__).resolve().parent.parent
    file_name = BASE_DIR + "/data/gui_2022_2_1_15_4_20_val.csv"
    #file_name = F"{BASE_DIR}/data/gui_2022_2_1_15_4_20_val.csv"
    values = read_csv_data(file_name)
    while not rospy.is_shutdown():
        i = 0
        for point in values:
            msg_to_publish.points[i].x = point.x
            msg_to_publish.points[i].y = point.y
            msg_to_publish.points[i].z = 0
            i += 1
            if i >= 400:
                break

        pub.publish(msg_to_publish)
        #rospy.loginfo(msg_to_publish)
        time.sleep(3)

if __name__ == "__main__":
	rospy.init_node("lidar_sim")
	publisher()
