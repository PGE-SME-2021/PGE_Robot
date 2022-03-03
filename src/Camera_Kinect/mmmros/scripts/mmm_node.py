#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import String
from mmmros.msg import Movement, SensorData
from MMM import MMM

# Main arduino node, listens for movement commands, sends them to the arduino
# Also listens to the arduino for sensor data and rebroadcasts it on another topic
def mmm_node():
    # Initialize the mmm node
    rospy.init_node("mmm")

    # Connect to MMM
    rospy.loginfo("Connecting...")
    try:
        mmm = MMM(rospy.get_param("mmm/port"))
        rospy.sleep(3)
        rospy.loginfo("Connected to MMM robot")
    except:
        rospy.loginfo("Could not connect to MMM robot, exiting node")
        return

    # Function for sending movement commands to Arduino, will be called whenever a message is sent over the topic
    def send_commands(msg):
        rospy.loginfo("Command received:\n" + str(msg))
        mmm.setWheelVelocity(msg.leftWheelSpeed, msg.rightWheelSpeed)
        mmm.rotateShoulders(msg.leftShoulderAngle, msg.rightShoulderAngle)
        mmm.rotateElbows(msg.leftElbowAngle, msg.rightElbowAngle)
        mmm.extendArms(msg.leftArmExtension, msg.rightArmExtension)
        mmm.setLeftGrippers(*msg.leftGripperAngles)
        mmm.setRightGrippers(*msg.rightGripperAngles)
        rospy.loginfo("Command relayed to robot.")
        # Update the robot
        mmm.clampAll()
        mmm.update()

    # Create the subscriber object for receiving movement commands
    sub = rospy.Subscriber("mmm/move_commands", Movement, send_commands)

    # Check if the node is supposed to be able to talk with the face
    if rospy.get_param("mmm/face/active"):
        from MMM_Speaker import Speaker
        # Create the MMM Speaker object
        speaker = Speaker()
        # Callback for the speaker
        def speaker_callback(msg):
            speaker.speak(msg.data)
        # Create the speaker Subscriber
        speaker_sub = rospy.Subscriber("mmm/speaker_msgs", String, speaker_callback)

    # Check if the node is supposed to be collecting ultrasonic sensor data
    if rospy.get_param("mmm/sensor/active"):
        # Create the publisher object for broadcasting sensor data
        sensor_pub = rospy.Publisher("mmm/sensor_data", SensorData, queue_size=10)
        # Object for controlling loop rate
        rate = rospy.Rate(rospy.get_param("mmm/sensor/rate"))
        # Action loop for publisher
        while not rospy.is_shutdown():
            # Get sensor data and publish it to the topic
            mmm.parseData()
            sensor_pub.publish(SensorData(mmm.getLeftRange(), mmm.getRightRange()))
            # Sleep for the remainder of the loop time
            rate.sleep()

    # Keep the node going until it is shut down
    rospy.spin()


if __name__ == '__main__':
    mmm_node()
