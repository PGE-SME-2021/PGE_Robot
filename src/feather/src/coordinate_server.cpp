//By MELET CHIRINO
#include "ros/ros.h"
#include "feather/PathPlanning.h"

bool add(
	feather::PathPlanning::Request &req,
	feather::PathPlanning::Response &res
	)
{
	//one million dollar code
	//path = path_planning_algorithm(x,y)
	//move(path)
	//res.result = get_to_point?
	ROS_INFO(
		"Coordinates: x=%ld, y=%ld", 
		(long int)req.x,
		(long int)req.y,
		);
	if (req.x < 180){
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

