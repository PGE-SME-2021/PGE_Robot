#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

void myCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

 ROS_INFO("height : %d |width : %d| %d", msg->height, msg->width);
}


int main(int argc, char** argv){

 ros::init(argc, argv, "my_PointCloud");
 ros::NodeHandle nh;
 ros::Subscriber my_subscriber = nh.subscribe("points",100, myCallback);
 ros::spin();
return 0;
}



