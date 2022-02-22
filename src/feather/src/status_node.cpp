/** \file  conversionc_dataLidar.cpp
 * 	This node publish the status of the robot (battery, speed, acceleraration, 
 *  angular_speed) in the topic robot_dinamics
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "feather/Status.h"


int main(int argc, char **argv)
{
char node_name[] = "motor_node", topic_name[] = "robot_dinamics";
ros::init(argc, argv, node_name);

ros::NodeHandle n;

ros::Publisher chatter_pub = n.advertise<feather::Status>(topic_name, 1000);

ros::Rate loop_rate(4);//hz

while (ros::ok()){
	feather::Status msg;

	//get battery function
	msg.battery = 45;

	//get speed
	msg.speed = 9;//ms/s
	msg.acceleration = 3;//m/s2
	msg.angular_speed = 2;//rad/s

	
	//if any error you can send them here
	msg.error = "ok";

	ROS_INFO("bat = %d", msg.battery);

	chatter_pub.publish(msg);

	ros::spinOnce();

	loop_rate.sleep();
	}

return 0;

}
