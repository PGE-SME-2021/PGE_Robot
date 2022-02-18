#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 #3.2.0
import time
import os

def publisher():
    '''Image publisher simulator node
    This node publishes 4 images every 4 seconds, the images must be placen in the same file folder
    '''
    pub = rospy.Publisher("image_sim", Image, queue_size= 10)
    rate = rospy.Rate(0.5)
    msg_to_publish = Image()
    BASE_DIR = os.path.dirname(os.path.realpath(__file__))
    #BASE_DIR = Path(__file__).resolve().parent
    while not rospy.is_shutdown():
        for i in range(1,5):
            file_name = BASE_DIR + "/img" + str(i) + ".jpg"
            #file_name = F"{BASE_DIR}/img{i}.jpg"
            print(file_name)
            img = cv2.imread(file_name)
            bridge = CvBridge()
            img_msg = bridge.cv2_to_imgmsg(img, encoding = 'passthrough')
            pub.publish(img_msg)
            time.sleep(2)


if __name__ == '__main__':
    '''Main function
    This function tests the image publisher simulator node
    '''
    rospy.init_node("img_sim_node")
    publisher()
