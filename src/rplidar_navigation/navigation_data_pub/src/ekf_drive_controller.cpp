/*
 * Automatic Addison
 * Date: May 23, 2021
 * ROS Version: ROS 1 - Melodic
 * Website: https://automaticaddison.com
 * Takes the current position and desired waypoint 
 *   as inputs and outputs a velocity command. It does not consider 
 *   obstacle avoidance.
 * Subscribe: ROS node that subscribes to the following topics:
 *   robot_pose_ekf/odom_combined : Current position and velocity estimate. 
 *                     The orientation.z variable needs to be converted to 
 *                     an Euler angle representing the yaw angle 
 *                     (geometry_msgs/PoseWithCovarianceStamped).
 *   waypoint_2d : The desired locations where the robot needs to go 
 *                     (geometry_msgs/PoseStamped).
 *
 * Publish: This node will publish to the following topics:
 *   cmd_vel : Linear & angular velocity command (geometry_msgs/Twist Message)
 *
 * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */

// Include the relevant libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <cstdlib>
#include <math.h>
#include <iostream>

using namespace std;

// Declare the important ROS variables
ros::Publisher pubVelocity; // Publishes velocity command
geometry_msgs::PoseWithCovarianceStamped odom; // Holds odometry information
geometry_msgs::Twist cmdVel; // Velocity command
geometry_msgs::PoseStamped desired; // Desired waypoint

// Declare constants
const double PI = 3.141592;
const double Ka = .65; // Proportional constant for angular velocity 
const double Klv = .95; // Proportional constant for linear velocity 
const double initialX = 0.0;
const double initialY = 0.0;
const double angularTolerance = .09; // Radians
const double distanceTolerance = 0.0001; // Meters
const double MIN_LINEAR_VEL = 0.10; // meters per second
const double MAX_LINEAR_VEL = 0.172; // meters per second
bool waypointActive = false;

// Update the x, y, and theta pose
void update_pose(const geometry_msgs::PoseWithCovarianceStamped &currentOdom) {
  odom.pose.pose.position.x = currentOdom.pose.pose.position.x; // meters
  odom.pose.pose.position.y = currentOdom.pose.pose.position.y; // meters
	
  // Convert from Euler to quaternion
  double yaw_z = 0;
  double t0 = 0;
  double t1 = 0;
  double xval = currentOdom.pose.pose.orientation.x;
  double yval = currentOdom.pose.pose.orientation.y;
  double zval = currentOdom.pose.pose.orientation.z;
  double wval = currentOdom.pose.pose.orientation.w;
  t0 = 2.0 * (wval * zval + xval * yval);
  t1 = 1.0 - 2.0 * (yval * yval + zval * zval);
  yaw_z = atan2(t0, t1);
  odom.pose.pose.orientation.z = yaw_z; // radians
}

// Update the desired waypoint x, y, and theta
void update_goal(const geometry_msgs::PoseStamped &desiredPose) {

  //cout<<"got new goal!"<<endl;
  desired.pose.position.x = desiredPose.pose.position.x;
  desired.pose.position.y = desiredPose.pose.position.y;
  desired.pose.orientation.z = desiredPose.pose.orientation.z;
  waypointActive = true;
  //cout<<"waypoint active set true"<<endl;
}

// Calculate the distance to the waypoint
double getDistanceError() {
  double deltaX = desired.pose.position.x - odom.pose.pose.position.x;
  double deltaY = desired.pose.position.y - odom.pose.pose.position.y;
  //cout<<"Distance error = "<<sqrt(pow(deltaX, 2) + pow(deltaY, 2))<<endl;
  return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}

// Calculate the yaw angle error
double getAngularError() {
  double deltaX = desired.pose.position.x - odom.pose.pose.position.x;
  double deltaY = desired.pose.position.y - odom.pose.pose.position.y;
  double thetaBearing = atan2(deltaY, deltaX);
  double angularError = thetaBearing - odom.pose.pose.orientation.z;
  angularError = (angularError > PI)  ? angularError - (2*PI) : angularError;
  angularError = (angularError < -PI) ? angularError + (2*PI) : angularError;
  //cout<<"angular error = " <<angularError<<endl;
  return angularError;
}

// Set the velocity command values
void set_velocity() {
  cmdVel.linear.x = 0;
  cmdVel.linear.y = 0;
  cmdVel.linear.z = 0;
  cmdVel.angular.x = 0;
  cmdVel.angular.y = 0;
  cmdVel.angular.z = 0;

  static bool angle_met = true;
  static bool location_met = true;
  double final_desired_heading_error = desired.pose.orientation.z - odom.pose.pose.orientation.z;

  // See if we have arrived at the waypoint
  if(abs(getDistanceError()) >= distanceTolerance) {
    location_met = false;
  }
  else {
    location_met = true;
  }

  double angularError = (location_met == false) ? getAngularError() : final_desired_heading_error;
	
  // See if we have reached our desired angle  
  if (abs(angularError) >= angularTolerance) {
    angle_met = false;
  }
  else {
    angle_met = true;
  }
  
	// Adjust angular velocity if we haven't yet met the angle target
	if (waypointActive == true && angle_met == false) {
    cmdVel.angular.z = Ka * angularError;
    cmdVel.linear.x = 0;
  }
	// Adjust linear velocity if we aren't yet at the destination.
  else if (waypointActive == true && abs(getDistanceError()) >= distanceTolerance && location_met == false) {
    cmdVel.linear.x = Klv * getDistanceError();
    cmdVel.angular.z = 0;
  }
  else {
    //cout << "********I'm HERE, now set final desired heading! **********"<<endl;
    location_met = true;
  }

  if (location_met && abs(final_desired_heading_error) < angularTolerance) {
    //cout<<"Target Achieved"<<endl;
    waypointActive = false;
  }

  // Publish the velocity command
  pubVelocity.publish(cmdVel);

}

int main(int argc, char **argv) {
	
	desired.pose.position.x = -1;
	
	// Connect to ROS
  ros::init(argc, argv, "drive_controller");
  ros::NodeHandle node;

  //Subscribe to ROS topics
  ros::Subscriber subCurrentPose = node.subscribe("robot_pose_ekf/odom_combined", 10, update_pose, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subDesiredPose = node.subscribe("waypoint_2d", 1, update_goal, ros::TransportHints().tcpNoDelay());
	
	// Publish to ROS topics
  pubVelocity = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Rate loop_rate(10);
  
	while (ros::ok()) {
    ros::spinOnce();
    if(desired.pose.position.x != -1) {
      set_velocity();
    }
    //cout << "goal = " << desired.pose.position.x << ", " << desired.pose.position.y << endl
    //     << "current x,y = " << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << endl
    //     << "  Distance error = " << getDistanceError() << endl;
    //cout << "cmd_vel = " << cmdVel.linear.x <<" ,  "<<cmdVel.angular.z<< endl;
    loop_rate.sleep();
  }
  return 0;
}
