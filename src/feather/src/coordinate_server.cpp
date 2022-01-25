//By MELET CHIRINO
#include "ros/ros.h"
#include "feather/PathPlanning.h"

bool path_planning(
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
		(long int)req.y
		);
	res.result = 34;
	if (req.x < 180){
		return true;
		}
	else{
		return false;
		}
	}

int main(int argc, char **argv){
	ros::init(argc, argv, "coordinate_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService(
			"coordinate_service",
			path_planning
			);
	ROS_INFO("Ready to send coordinates to robot");
	ros::spin();
	return 0;
	}

