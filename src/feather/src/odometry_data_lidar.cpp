// library
/** \file  odometry_data_lidar.cpp
 * This node show the odometry data 
 */
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "serial/serial.h"
#include "stdio.h"
#include "feather/SendCommand.h"
#include "serial_cmd.cpp"
#include <std_msgs/Int16.h>

/*! \fn void odomCallback_euler(const nav_msgs::Odometry& data_euler) 
 *  \brief This function shows the position and velocity data of type eurler.
 *  \param data_euler contains the actual position and speed of robot
 */


void odomCallback_euler(const nav_msgs::Odometry& data_euler){
	ROS_INFO("Data odometrie receive"); // = printf 
	for (int i = 0; i < 36 ; i++  ) {
		ROS_INFO("Data euler position %i  =  %lf ", i , data_euler->pose->covariance[i]);
	}
	for (int i = 0; i < 36 ; i++  ) {
		ROS_INFO("Data euler Rotation %i  =  %lf ", i , data_euler->twist->covariance[i]);
	}
}

/*! \fn void odomCallback_quaternion(const nav_msgs::Odometry& data_quot) 
 *  \brief This function shows the position and velocity data of type quaternion.
 *  \param data_quat  contains the actual position and speed of robot
 */

void odomCallback_quaternion(const nav_msgs::Odometry& data_quot){
	ROS_INFO("Data odometrie receive"); // = printf 
	for (int i = 0; i < 36 ; i++  ) {
		ROS_INFO("Data quaternion position %i  =  %lf ", i , data_quot->pose->covariance[i]);
	}
	for (int i = 0; i < 36 ; i++  ) {
		ROS_INFO("Data quaternion Rotation %i  =  %lf ", i , data_quot->twist->covariance[i]);
	}
}

int main(int argc, char **argv){
  	ros::init(argc, argv, "odometry");
  	ros::NodeHandle n; // déclaration de noeud 
	//*******Suscriber************************
	ros::Suscriber data_euler=n.subscribe("odom_data_euler",1, odomCallback_euler);
	ros::Suscriber data_quot=n.subscribe("odom_data_quot",1, odomCallback_quaternion);

 	//*******Publisher************************
	// le node va publier des messages de type std_msgs/int16 sur le topic "/rightticks"et sur le topic "/leftticks"
	//Après 100 images , les data les plus anciennnes sont abandonnées
	ros::Publisher Encoder_right_pub=n.advertise<std_msgs::Int16>("right_ticks",100); 
	ros::Publisher Encoder_left_pub=n.advertise<std_msgs::Int16>("left_ticks",100); 

	
  	
  	ros::spin();//loop
  
  return 0;
}
