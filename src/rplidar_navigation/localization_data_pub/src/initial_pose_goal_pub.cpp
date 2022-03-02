/*
 * Automatic Addison
 * Website: https://automaticaddison.com
 *   ROS node that asks the user to input either an initial pose or a goal pose.
 * Publish: This node publishes to the following topics:   
 *   goal_2d : Goal position and orientation (geometry_msgs::PoseStamped)
 *  
 *   move_base_simple/goal : Goal position and 
 *                           orientation (geometry_msgs::PoseStamped)
 *   initial_2d : The initial position and orientation of the robot using 
 *                Euler angles. (geometry_msgs/PoseStamped)
 *   initialpose : The initial position and orientation of the robot using 
 *                 quaternions. (geometry_msgs/PoseWithCovarianceStamped)
 * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */
 
// Include messages
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>

using namespace std;

// Declare our publishers
ros::Publisher pub;
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;

// Publish a pose or a goal in either Euler or quaternion format
void pub_msg(float x, float y, float yaw, int choice) {
  geometry_msgs::PoseWithCovarianceStamped pose;
  geometry_msgs::PoseStamped goal;
  geometry_msgs::Pose rpy;
  rpy.position.x = x;
  rpy.position.y = y;
  rpy.position.z = 0;
  rpy.orientation.x = 0;
  rpy.orientation.y = 0;
  rpy.orientation.z = yaw;
  rpy.orientation.w = 0;

  // Publish a pose in Euler format (roll, pitch, yaw) 
  // The pose is relative to the map coordinate frame.
  if (choice == 1) { 
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.pose = rpy;
    pub2.publish(pose);
  }
  // Publish a goal
  else {
    cout << "publishing goal" << endl;

    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose = rpy;
    pub.publish(goal);
  }

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  rpy.orientation.x = q.x();
  rpy.orientation.y = q.y();
  rpy.orientation.z = q.z();
  rpy.orientation.w = q.w();

  // Publish a pose in quaternion format.
  if(choice == 1) { 
    
    pose.pose.pose = rpy;
    pub3.publish(pose);
  }
  // Publish a goal
  else {     
    goal.pose = rpy;
    pub1.publish(goal);
  }
}

int main(int argc, char **argv) {
  
  // Connect to ROS
  ros::init(argc, argv, "initial_pose_goal_pub");
  ros::NodeHandle node;
	
  // Set up our ROS publishers
  pub = node.advertise<geometry_msgs::PoseStamped>("goal_2d", 10);
  pub1 = node.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  pub2 = node.advertise<geometry_msgs::PoseStamped>("initial_2d", 10);
  pub3 = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);

  // Keep looping as long as ROS is running
  while (ros::ok()) {
		
    // Initialize the pose variables
    float x = -1;
    float y = -1;
    float yaw = -11;
    int choice  = -1;

    while(choice < 1 || choice > 2
         || x < 0 || y < 0 || yaw < -3.141592 || yaw > 3.141592) {
      
      cout << "\nEnter positive float value for x : " << endl;
      cin >> x;
      cout << "\nEnter positive float value for y : " << endl;
      cin >> y;
      cout << "\nEnter float value for yaw in radians from -PI to +PI (e.g. 90 deg = 1.57 rad): " << endl;
      cin >> yaw;
      cout << "\nEnter 1 if this is a pose, 2 if it is a goal" << endl;
      cin >> choice;
      if (cin.fail()) {
        cin.clear();
        cin.ignore(50, '\n');
        choice = -1;
      }
    }
    
    // Populate the pose or goal fields and publish.
    pub_msg(x, y, yaw, choice);

  }
  return 0;
}
