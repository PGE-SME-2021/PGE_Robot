#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

int inter_v = 0;
void chatterCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("<--- I heard: [%d]", msg->data);
  inter_v = msg->data;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  std_msgs::Int32 sender;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::Publisher pub = n.advertise<std_msgs::Int32>("hello2", 1000);
  ros::Rate loop_rate(0.5);
  while(ros::ok()){
	  sender.data =  inter_v + 2;
	  ROS_INFO("---> sending %d", sender.data);
	  pub.publish(sender);
	  loop_rate.sleep();
	  ros::spinOnce()
  }

  return 0;
}
