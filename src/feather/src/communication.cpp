// library
#include "ros/ros.h"

#include "serial/serial.h"
#include "stdio.h"
#include "feather/SendCommand.h"
#include "serial_cmd.cpp"

bool raspi_to_esp(feather::SendCommand::Request  &req,
         feather::SendCommand::Response &res)
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

void odomCallback(){
	ROS_INFO("Data odometrie receive"); // = printf 
}

int main(int argc, char **argv){
  	ros::init(argc, argv, "communication");
  	ros::NodeHandle n; // déclaration de noeud 
	//*******Suscriber************************
  	ros::ServiceServer service = n.advertiseService("send_command", raspi_to_esp); // le noeud est abonné au send command
	//ros::Suscriber data_euler=n.subscribe("/odom_data_euler",1, odomCallback);
	//ros::Suscriber data_quot=n.subscribe("/odom_data_quot",1, odomCallback);
 	//*******Publisher************************
	// le node va publier des messages de type std_msgs/int16 sur le topic "/rightticks"et sur le topic "/leftticks"
	//Après 100 images , les data les plus anciennnes sont abandonnées
	//ros::Publisher Encoder_right_pub=n.advertise<std_msgs::int16>("/right_ticks",100); 
	//ros::Publisher Encoder_left_pub=n.advertise<std_msgs::int16>("/leftt_ticks",100); 

	
  	ROS_INFO("Ready to send commands to esp."); // = printf 
  	ros::spin();//loop
  
  return 0;
}
