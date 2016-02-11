#include "ros/ros.h"
#include "std_msgs/String.h"
#include "feather/LidarData.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Path.h"

#include <sstream>
//float traject[80]={};
float* coordonnees;
void trajectorycallback (const nav_msgs::Path traject)
{
	ROS_INFO("It is good %d", traject);
	// coordonnees= traject;

}

int main(int argc, char **argv)
{

ros::init(argc, argv, "lidar_sim_cpp");
ros::init(argc, argv, "sub_cartesian");

ros::NodeHandle n;
ros:: Subscriber subsc=n.subscribe("trajectory",10,trajectorycallback);
ros::Publisher chatter_pub = n.advertise<feather::LidarData>("lidar_points", 1000);
float x=0, y=0;
int i, count;
//float *coor;
//float** traject;
//std::vector<geometry_msgs::PoseStamped> const & poses = &traject;
//ros::Rate loop_rate(10);
while (ros::ok()){
	feather::LidarData msg;
	nav_msgs::Path coordonnees;
	//coor= &trajectory_.trajectory.poses;
	
	//std::stringstream coor;
	//std:: << "yes" << coor;
	//msg.data = coor;


	

	ROS_INFO("%f", coordonnees);

	for(i = 0; i < 1000; i++){
		msg.points[i].x = 5;
		msg.points[i].y = 5;
		msg.points[i].z = 0;
		}

	//ROS_INFO("%f %f", msg.points[0].x, msg.points[0].y);

	chatter_pub.publish(coordonnees);

	ros::spinOnce();

	//loop_rate.sleep();
}	

return 0;

}
