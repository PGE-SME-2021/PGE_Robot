#!/usr/bin/env python3
import rospy
from feather.msg import Status
from std_msgs.msg import String
class NodeSub():
    def __init__(self, topic_name, message_type, node_name):
        self.topic_name = topic_name
        self. message_type = message_type
        self.node_name = node_name
        self.test_value = 0

    def start_node(self):
        rospy.init_node(self.node_name)

    def subscriber(self):
        print("Creating sub node")
        sub = rospy.Subscriber(
                self.topic_name,
                self.message_type,
                self.callback_function
                )
        print("spining sub node")
        rospy.spin()

    def callback_function(self, message):
        print("result:")
        rospy.loginfo(F"GOT {message}")
        #print(F"GOT {message.error} and {message.battery}%")
        self.test_value = message


if __name__ == "__main__":
    #love_node_sub = NodeSub('love', String, 'love_recv')
    #love_node_sub.start_node()
    #love_node_sub. subscriber()

    node_sub = NodeSub('status', Status, 'status_recv')
    node_sub.start_node()
    node_sub.subscriber()

