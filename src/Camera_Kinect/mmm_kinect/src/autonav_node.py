#!/usr/bin/env python
from __future__ import division
import rospy
from mmmros.msg import Movement
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import random
import threading

class AutonavNode(object):

def __init__(self):

		self.min_distance = 500  #millimetres

		self.speed = rospy.get_param("mmm/leftWheelSpeed/max")/2

		self.avoiding = False

		rospy.init_node("autonav_node")

		self.cv_bridge = CvBridge()

		self.sub = rospy.Subscriber("camera/depth_registered/image_raw", Image, self.process_depth_image)

		self.pub = rospy.Publisher("mmm/move_commands", Movement, queue_size=1)

		rospy.sleep(4)  #nécessaire pour laisser le temps à la node de se lancer

		self.pub.publish(Movement(leftWheelSpeed=self.speed, rightWheelSpeed=self.speed)) #au démarrage le robot avance vers l'avant jusqu'à rencontrer un obstacle

		rospy.spin()

def process_depth_image(self, msg):

	im = self.cv_bridge.imgmsg_to_cv2(msg)

	min_point = im[im.nonzero()].min()

	print(min_point)

	if min_point < self.min_distance:
		if not self.avoiding:
			threading.Thread(target=self.avoid_obstacle).start()
		else:
			self.avoiding = False

def avoid_obstacle(self):

	print("AVOIDING")

	self.avoiding = True

	self.pub.publish(Movement(leftWheelSpeed=self.speed, rightWheelSpeed=-self.speed))

	while self.avoiding:

		rospy.sleep(1/30)

		self.pub.publish(Movement(leftWheelSpeed=self.speed, rightWheelSpeed=self.speed))

		print("DONE AVOIDING")

if __name__ == '__main__':

autonav_node = AutonavNode()
