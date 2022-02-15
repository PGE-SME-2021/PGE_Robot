#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 #3.2.0
from pathlib import Path
import time

def publisher():
    pub = rospy.Publisher("image_sim", Image, queue_size= 10)
    rate = rospy.Rate(0.5)
    msg_to_publish = Image()
    BASE_DIR = Path(__file__).resolve().parent
    while not rospy.is_shutdown():
        file_name = F"{BASE_DIR}/img1.jpg"
        print(file_name)
        img = cv2.imread(file_name)
        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(img, encoding = 'passthrough')
        pub.publish(img_msg)
        time.sleep(4)
        #-------------------------------------
        file_name = F"{BASE_DIR}/img2.jpg"
        print(file_name)
        img = cv2.imread(file_name)
        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(img, encoding = 'passthrough')
        pub.publish(img_msg)
        time.sleep(4)


if __name__ == '__main__':
    rospy.init_node("img_sim_node")
    publisher()
