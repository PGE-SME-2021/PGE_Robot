#!/usr/bin/env python3
#ros dependencies
import rospy
from feather.msg import Status, LidarData
from sensor_msgs.msg import PointCloud, Image
from feather.msg import Status
from feather.srv import SendCommand

#simulator
from galpon import CordobesSimulator

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
    status_pub = rospy.Publisher("status", Status, queue_size = 10)
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
    rate = rospy.Rate(100)#Hz
    while sim.running:
        sim.gameloop()
        status_msg.speed = sim.gallito.speed
        status_msg.angular_speed = sim.gallito.angle_speed
        status_msg.battery = 75
        status_msg. acceleration = 0

        #rospy.loginfo(status_msg)
        status_pub.publish(status_msg)
        #rate.sleep()

