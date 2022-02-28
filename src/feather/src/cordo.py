#!/usr/bin/env python3
#ros dependencies
import rospy
from feather.msg import Status, LidarData
from sensor_msgs.msg import PointCloud, Image
from feather.msg import Status
from feather.srv import SendCommand

#simulator
from galpon import CordobesSimulator
import time

def handle_move(req):
    global sim

    if req.command == 1:
        sim.gallito.move_forward()
    elif req.command == 2:
        sim.gallito.move_backward()
    elif req.command == 3:
        sim.gallito.turn_left()
    elif req.command == 4:
        sim.gallito.turn_right()
    elif req.command == 5:
        sim.gallito.stop_move()
        sim.gallito.stop_rotating()
    return 12


if __name__ == "__main__":
    global sim

    rospy.init_node('feather_sim')
    status_pub = rospy.Publisher("status", Status, queue_size = 3)
    lidar_pub = rospy.Publisher("lidar_points", LidarData, queue_size = 3)
    move_server = rospy.Service('send_command', SendCommand, handle_move)
    sim = CordobesSimulator(
            screen_size = [650, 650],
            params = {
                'speed': 0.6,
                'angle_speed': 1,
                'bullet_speed': 1,
                }
            )

    status_msg = Status()
    lidar_msg = LidarData()
    rate = rospy.Rate(100)#Hz
    while sim.running and not rospy.is_shutdown():
        sim.gameloop()
        #fill status message
        status_msg.speed = sim.gallito.speed
        status_msg.angular_speed = sim.gallito.angle_speed
        status_msg.battery = 75
        status_msg. acceleration = 0

        if len(sim.lidar_points) > 0:
            i = 0
            for point in sim.lidar_points:
                if not point == [0, 0]:
                    lidar_msg.points[i].x = point[0]
                    lidar_msg.points[i].y = point[1]
                    lidar_msg.points[i].z = 0
                    i += 1
                if i >= 400:
                    break
            rospy.loginfo(lidar_msg)

        lidar_pub.publish(lidar_msg)
        status_pub.publish(status_msg)
        #rate.sleep()

