/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
//#include <pluginlib/class_list_macros.h>

//typedef pcl::PointXYZ PointType;
//typedef pcl::PointXYZI IntensityType;
//typedef pcl::Normal NormalType;
//typedef pcl::SHOT352 DescriptorType;
//typedef pcl::ReferenceFrame RFType;
//typedef pcl::PointNormal NormalPointType;

//bool got_cloud_ = false;

void depthCallback(const sensor_msgs::PointCloud2& cloud_msg)
{
  // got_cloud_ = true;
  //pcl::PCLPointCloud2 pcl_pc;
  //pcl_conversions::toPCL(cloud_msg,pcl_pc);
 //pcl::PointCloud::Ptr scene_cloud(new pcl::PointCloud);
 //pcl::fromPCLPointCloud2(pcl_pc,*scene_cloud);
 //std::cout << "Cloud size: " << scene_cloud->points.size() << std::endl;
}

void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img)
{
  //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
  cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg1)
{
  cv_bridge::CvImagePtr cv_ptr2;
  
  try
  {
        cv_ptr2 = cv_bridge::toCvCopy(msg1); 
  }
  catch (cv_bridge::Exception& e)
  {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", e.what());
	return;
  }
  
    cv::imshow("Depth Image",cv_ptr2->image);

    cv::Mat depth_float_img = cv_ptr2->image;
    cv::Mat depth_mono8_img;
    
    depthToCV8UC1(depth_float_img, depth_mono8_img);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg2)
{
  cv_bridge::CvImagePtr cv_ptr;

   try
   { 
   
    cv_ptr = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::BGR8);

    cv::Mat current_frame = cv_ptr->image;
     
    cv::imshow("Camera", current_frame); 
     
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", e.what());
    return;
  }
}

int main(int argc, char **argv)
{

  //begin =ros::Time::now().toSec();
  //current = ros::Time::now().toSec();
  
  ros::init(argc, argv, "image_converter");
  
  ros::NodeHandle kinect_node;  
  ros::Subscriber sub_depth;
  
  image_transport::ImageTransport it(kinect_node);
  image_transport::Subscriber sub_image;
  image_transport::Subscriber sub_depth_image;
  
  //sub_depth_image = it.subscribe("camera/depth/image", 1, depthImageCallback);
  sub_image = it.subscribe("camera/rgb/image_color", 1, imageCallback);
  //sub_depth = kinect_node.subscribe("camera/depth/points", 1, depthCallback);
  
  //while (current-begin < time_limit && !got_image_ && current-begin < time_limit && !got_cloud_)
  //{
  // current =ros::Time::now().toSec();
    
  ros::spin();
    // }
  //image_sub_.shutdown();
  //point_cloud_sub_.shutdown();
  cv::waitKey(10);
  cv::destroyWindow("view");
  
  return 0;
}
