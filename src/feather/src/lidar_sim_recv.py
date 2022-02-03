#!/usr/bin/env python
import rospy
#from std_msgs.msg import Int32MultiArray
from feather.msg import LidarData
from std_msgs.msg import String
from nav_msgs import Path


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    import pdb; pdb.set_trace()
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener')

    rospy.Subscriber("trajectory", Path, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()