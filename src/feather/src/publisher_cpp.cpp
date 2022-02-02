#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <sstream>

int main(int argc, char **argv)
{
ros::init(argc, argv, "talker");

ros::NodeHandle n;

ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("chatter", 1000);

ros::Rate loop_rate(0.5);

int count = 0;
while (ros::ok()){
	std_msgs::Int32 msg;

	std::stringstream ss;
	ss << "hello world " << count;
	msg.data = count;

	ROS_INFO("%d", msg.data);

	chatter_pub.publish(msg);

	ros::spinOnce();

	loop_rate.sleep();
	++count;
	}

return 0;

}
