#include "ros/ros.h"

#include "feather/SendCommand.h"

bool add(feather::SendCommand::Request  &req,
         feather::SendCommand::Response &res)
{
    if (0 <= req.com && req.com >= 7)
    {
        res.check = 1;
    }
    else 
    {
        res.check = 0;
    }  
  ROS_INFO("request: com=%ld", (long int)req.com);
  ROS_INFO("sending back response: [%ld]", (long  int)res.check);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_command_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("send_command", add);
  ROS_INFO("Ready to send commands.");
  ros::spin();

  return 0;
}
