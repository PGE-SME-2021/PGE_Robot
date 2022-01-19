import rospy

class NodeSub():
    def __init__(self, topic_name, message_type, node_name):
        self.topic_name = topic_name
        self. message_type = message_type
        self.node_name = node_name
        self.data = LidarData()

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
        self.data = message
        print("result:")
        rospy.loginfo(F"GOT {self.data}")
