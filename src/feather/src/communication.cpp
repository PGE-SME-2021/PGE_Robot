// library
#include "ros/ros.h"

#include "serial/serial.h"
#include "stdio.h"
#include "feather/SendCommand.h"
#include "serial_cmd.cpp"

bool raspi_to_esp(feather::SendCommand::Request  &req,
         feather::SendCommand::Response &res)
{ 

	if ((0 <= req.com) && (req.com <= 7)) {

	send_serial(req.com);
      	res.check = 1;
    } else  {
        res.check = 0;
    }
    
    ROS_INFO("request: com=%ld", (long int)req.com);
    ROS_INFO("sending back response: [%ld]", (long  int)res.check);
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "communication");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("send_command", raspi_to_esp);
  ROS_INFO("Ready to send commands to esp."); // = printf 
  ros::spin();

  return 0;
}
