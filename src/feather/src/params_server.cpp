//By MELET CHIRINO

/** \file params_server.cpp
 * Server that sets parameters of the robot
 */

#include "ros/ros.h"
#include "feather/SetParams.h"

/*! \fn bool path_planning(
	feather::PathPlanning::Request &req,
	feather::PathPlanning::Response &res
	) 
 *  \brief This function sets dinamic parameters like maximum speed or acceleration. You can add more parameters changing the file feather/srv/SetParams.srv
 *  \param req request object of the PathPlannig service. It holds max_speed, max_acceleration, max_angular_speed.
 *  \param res reponse object of the PathPlannig service. It holds check.
 *  \return execution check boolean.
 */
bool set_params(
	feather::SetParams::Request &req,
	feather::SetParams::Response &res
	)
{
	ROS_INFO(
		"Params: \nmax speed = %ld\nmax acceleration = %ld\nmax angular speed = %ld", 
		(long int)req.max_speed,
		(long int)req.max_acceleration,
		(long int)req.max_angular_speed
		);
	res.check = 34;
	if (req.max_speed < 180){
			//if not errors
			//send data as parameters to esp32
		return true;
		}
	else{
			//iff errors
		return false;
		}
	}

/*! \fn int main(int argc, char **argv) 
 *  \brief Main function that starts the node and connects 
 *	to the SetParams service
 *	\return 0 after execution	 
 */
int main(int argc, char **argv){
	ros::init(argc, argv, "params_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService(
			"params_server",
			set_params
			);
	ROS_INFO("Ready to set params to robot");
	ros::spin();
	return 0;
	}

