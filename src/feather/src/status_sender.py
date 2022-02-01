#!/usr/bin/env python3
import rospy
import time
from feather.msg import Status
#from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher("status", Status, queue_size = 10)
    rate = rospy.Rate(1)#1Hz
    counter = 0
    battery_level = 50
    msg_to_publish = Status()
    msg_to_publish.error = "Ok"
    while not rospy.is_shutdown():
        str_to_pub = "counter = %d, baterry = %d" % (counter, battery_level)
        counter += 1
        if counter > 10:
            counter = 0
            battery_level += 1
        msg_to_publish.battery = battery_level
        pub.publish(msg_to_publish)
        rospy.loginfo(str_to_pub)
        time.sleep(1)

if __name__ == "__main__":
	rospy.init_node("status_pub")
	publisher()
