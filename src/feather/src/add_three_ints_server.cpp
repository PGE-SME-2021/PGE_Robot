//By MELET CHIRINO
#include "ros/ros.h"
#include "feather/AddThreeInts.h"

bool add(
	feather::AddThreeInts::Request &req,
	feather::AddThreeInts::Response &res
	)
{
	res.sum = req.a + req.b + req.c;
	ROS_INFO(
		"REQ: a=%ld, b=%ld, c=%ld", 
		(long int)req.a,
		(long int)req.b,
		(long int)req.c
		);
	if (res.sum < 40){
		ROS_INFO("sum = %ld", (long int)res.sum);
		return true;
		}
	else{
		ROS_INFO("Too big SUM = %ld", (long int)res.sum);
		return false;
		}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "add_3_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService(
			"add_3",
			add
			);
	ROS_INFO("Give it to me");
	ros::spin();
	return 0;
	}

