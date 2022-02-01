#!/usr/bin/env python3
import rospy
#from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
#from nav_msgs import Path
from sensor_msgs.msg import PointCloud
import inspect

from csv_functions import *

def callback(data):
    BASE_DIR = Path(__file__).resolve().parent.parent.parent
    #rospy.loginfo(rospy.get_caller_id() + "I heard %u", data)
    #exit(1)
    new_points = data.points
    file_name = generate_file_name()
    save_csv_data(file_name, new_points)
    import pdb; pdb.set_trace()
    print("got %d points" % len(new_points))
    print("new points = %d , %d" % (new_points[80].x, new_points[83].y))

"""
['__class__', '__delattr__', '__doc__', '__eq__', '__format__', '__getattribute__', '__getstate__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__setstate__', '__sizeof__', '__slots__', '__str__', '__subclasshook__', '_check_types', '_connection_header', '_full_text', '_get_types', '_has_header', '_md5sum', '_slot_types', '_type', 'channels', 'deserialize', 'deserialize_numpy', 'header', 'points', 'serialize', 'serialize_numpy']
"""
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener')

    rospy.Subscriber("slam_cloud", PointCloud, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
