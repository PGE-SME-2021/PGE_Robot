//By MELET CHIRINO

/** \file  coordinate_server.cpp
 * This function sends the 2D coordinates
 * of the target position
 */

#include "ros/ros.h"
#include "feather/PathPlanning.h"

/*! \fn bool path_planning(
	feather::PathPlanning::Request &req,
	feather::PathPlanning::Response &res
	) 
 *  \brief This fonction sends the goal position coordinates from Raspy to ESP32.
 *  \param req request object of the PathPlannig service.
 *  \param res reponse object of the PathPlannig service.
 *  \return execution check boolean.
 */
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

/*! \fn int main(int argc, char **argv) 
 *  \brief Main function that starts the node and connects 
 *	to the PathPlannig service
 *	\return 0 after execution	 
 */
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

