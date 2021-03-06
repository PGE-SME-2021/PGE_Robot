// library
/** \file  conversionc_dataLidar.cpp
 * This is a node that allows you to configure the speed
 *  parameter of the robot in manual mode and move it in
 *  the direction desired by the user.
 */
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "serial/serial.h"
#include "stdio.h"
#include "feather/SendCommand.h"
#include "serial_cmd.cpp"
#include <std_msgs/Int16.h>


void odomCallback_euler(){
	ROS_INFO("Data receive"); // = printf 

}

//fonction conversion
/*! \fn bool raspi_to_esp(feather::SendCommand::Request  &req, feather::SendCommand::Response &res) 
 *  \brief This function converts the lidar data into integer data.
 */

int main(int argc, char **argv){
  	ros::init(argc, argv, "conversion_dataLidar_communication");
  	ros::NodeHandle n; // déclaration de noeud 
	//*******Suscriber************************
	ros::Suscriber data_float_int=n.subscribe("conv64",1, Callback);
 	//*******Publisher************************
	ros::Publisher data_int=n.advertise<std_msgs::Int16>("convInt",1); 
  	ros::spin();//loop
  
  return 0;
}
