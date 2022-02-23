!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 #3.2.0
## Callback for Image Subscriber
# @param data This is the message we receive from the topic
def callback(data):
    br = CvBridge()                                                 #convert between ROS and OpenCV images
    current_frame = br.imgmsg_to_cv2(data)                          #convert ROS image message to OpenCV image
    bgr_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGRA2BGR)
    cv2.imshow("Camera Kinect", bgr_frame)
    cv2.waitKey(1)

def receive_message():
    '''ROS node
    This node is subscribed to 'kinect_sub_py' and he shows the image
    '''
    rospy.init_node('kinect_sub_py', anonymous=True)

    #rospy.Subscriber('camera/rgb/image_color', Image, callback)     #subscribing to the camera/rgb/image_color topic
    rospy.Subscriber('image_sim', Image, callback)     #subscribing to the camera/rgb/image_color topic

    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    '''Main function
    This function tests Image subscriber node
    '''
    receive_message()
