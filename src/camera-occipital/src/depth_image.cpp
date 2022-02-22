#include "ros/ros.h"
#include "sensor_msgs/Image.h"

void myCallback(const sensor_msgs::Image::ConstPtr& msg){

 //ROS_INFO("height | width | encoding | is_bigendiand | step |");
 ROS_INFO("%d |%d| %d", msg->height, msg->width);
}


int main(int argc, char** argv){

 ros::init(argc, argv, "my_depth");
 ros::NodeHandle nh;

 ros::Subscriber my_subscriber = nh.subscribe("camera/depth/image",100, myCallback);
 ros::spin();
return 0;
}



