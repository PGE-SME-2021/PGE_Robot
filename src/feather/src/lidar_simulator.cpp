#include "ros/ros.h"
#include "feather/LidarData.h"

#include <sstream>

int main(int argc, char **argv)
{
float x=0, y=0;
int i, count;

ros::init(argc, argv, "lidar_sim_cpp");

ros::NodeHandle n;

ros::Publisher chatter_pub = n.advertise<feather::LidarData>("lidar_points", 1000);

ros::Rate loop_rate(10);
count = 0;
while (ros::ok()){
	x = count; y = count - 30;
	feather::LidarData msg;

	
	for(i = 0; i < 80; i++){
		x = x + 5;
		y = y + 1;
		msg.points[i].x = x;
		msg.points[i].y = y;
		msg.points[i].z = 0;
		}	

//ROS_INFO("%f %f", msg.points[0].x, msg.points[0].y);

	chatter_pub.publish(msg);

	ros::spinOnce();

	loop_rate.sleep();
	count++;
	if(count > 200) count = 0;
	}

return 0;

}
