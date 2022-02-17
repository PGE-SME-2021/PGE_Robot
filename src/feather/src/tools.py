import rospy

class NodeSub():
    def __init__(self, node_name):
        self.node_name = node_name


    def start_node(self):
        rospy.init_node(self.node_name)

    def subscriber(self, topic_name, message_type, callback_function):
        print("Creating sub node")
        #self.topic_name = topic_name
        #self. message_type = message_type
        self.data = message_type()
        subs_list =[]
        sub = rospy.Subscriber(
                self.topic_name,
                self.message_type,
                callback_function
                )
        sub_list.append([topic_name, sub])
        print("spining sub node")
        rospy.spin()

    def callback_function(self, message):
        self.data = message
        print("result:")
        rospy.loginfo(F"GOT {self.data}")


