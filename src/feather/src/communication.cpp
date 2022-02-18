/** \file  communication.cpp
 * This is a node that allows you to configure the speed
 *  parameter of the robot in manual mode and move it in
 *  the direction desired by the user.
 */

// library
#include "ros/ros.h"
//#include "serial/serial.h"
#include "stdio.h"
#include "feather/SendCommand.h"
#include "serial_cmd.cpp"
/*! \fn bool raspi_to_esp(feather::SendCommand::Request  &req, feather::SendCommand::Response &res) 
 *  \brief This fonction sends commands from Raspy to ESP32 about
 *  speed of the wheels and robot movement direction.
 *  \param req request object of the SendComands service.
 *  \param res reponse object of the SendComands service.
 *  \return execution check boolean.
 */

bool raspi_to_esp(feather::SendCommand::Request  &req, feather::SendCommand::Response &res)
{ 
	if ((220 <= req.vitesse1) && (req.vitesse1 <= 255)) {
		if ((220 <= req.vitesse2) && (req.vitesse2 <= 255)) {
			send_serial_cmd_vitesse_motor( req.vitesse1, req.vitesse2);
	      		res.check = 1;
		} else  {
        		res.check = 0;
    		}
    	} else  {
        	res.check = 0;
    	}
    	if ((0 <= req.command) && (req.command <= 7)) {
		send_serial_cmd_motor(req.command);
      		res.check = 1;
    	} else  {
        	res.check = 0;
    	}
    	
    ROS_INFO("request: com=%ld", (long 	int)req.command);
    ROS_INFO("request: vitesse1 =%ld", (long int)req.vitesse1);
    ROS_INFO("request: vitesse2=%ld", (long int)req.vitesse2);
    ROS_INFO("sending back response: [%ld]", (long  int)res.check);
  return true;
}
/*! \fn int main(int argc, char **argv) 
 *  \brief Main function that starts the node and connects 
 *	to the SendCommands service
 *	\return 0 after execution	 
 */
int main(int argc, char **argv){
  	ros::init(argc, argv, "communication");
  	ros::NodeHandle n; // déclaration de noeud 
  	ros::ServiceServer service = n.advertiseService("send_command", raspi_to_esp); // le noeud est abonné au send command
	
  	ROS_INFO("Ready to send commands to esp."); // = printf 
  	ros::spin();//loop
  
  return 0;
}
