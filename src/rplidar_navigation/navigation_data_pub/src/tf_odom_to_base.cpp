/*
 * Automatic Addison
 * Date: May 26, 2021
 * ROS Version: ROS 1 - Melodic
 * Website: https://automaticaddison.com
 * ROS node that subscribes to the pose of the robot and publishes 
 * an odom to base_footprint transform based on the data.
 * Subscribe:  
 *   robot_pose_ekf/odom_combined : Current position and velocity estimate. 
 *                     The orientation is a quaternion 
 *                     (geometry_msgs/PoseWithCovarianceStamped).
 *
 * Publish: This node will publish to the following topics:
 *  odom to base_footprint : The pose of the base_footprint coordinate frame
 *                           inside the odom coordinate frame (tf/tfMessage)
 * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */
 
 // Include the relevant libraries
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32.h"

using namespace std;

// Callback message that broadcasts the robot pose information as a transform
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped &currentOdom) {

  // Create a TransformBroadcaster object that will publish transforms over ROS
  static tf::TransformBroadcaster br;
  
  // Copy the information from the robot's current 2D pose into the 3D transform
  tf::Transform odom_base_tf;	
  odom_base_tf.setOrigin( tf::Vector3(currentOdom.pose.pose.position.x, currentOdom.pose.pose.position.y, 0.0) );
  tf::Quaternion tf_q(currentOdom.pose.pose.orientation.x, currentOdom.pose.pose.orientation.y,
                        currentOdom.pose.pose.orientation.z, currentOdom.pose.pose.orientation.w);

  // Set the rotation
  odom_base_tf.setRotation(tf_q);

  br.sendTransform(tf::StampedTransform(odom_base_tf, currentOdom.header.stamp, "odom", "base_footprint"));
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "tf_odom_to_base");
  ros::NodeHandle node;

  ros::Subscriber subOdom = node.subscribe("robot_pose_ekf/odom_combined", 10, pose_callback);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
		
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
