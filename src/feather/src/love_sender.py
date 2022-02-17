#!/usr/bin/env python
import rospy
import time
#from feather.msg import Status
from std_msgs.msg import String

def publisher():
    """Love node
    This is a publisher that sends a string with love every second.
    """
    pub = rospy.Publisher("love", String, queue_size = 10)
    rate = rospy.Rate(1)#1Hz
    counter = 0
    msg_to_publish = String()
    while not rospy.is_shutdown():
        str_to_pub = "Love => %d" % (counter)
        counter += 1
        if counter > 7:
            counter = 0
        msg_to_publish.data = str_to_pub
        pub.publish(str_to_pub)
        rospy.loginfo(str_to_pub)
        time.sleep(1)

if __name__ == "__main__":
    '''Main Function
    This function tests the love sender
    '''
	rospy.init_node("feelings_pub")
	publisher()
