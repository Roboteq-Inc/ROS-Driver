#include "ros/ros.h"
#include <roboteq_motor_controller_driver/config_srv.h>
 
int main(int argc, char **argv)
 {
   ros::init(argc, argv, "config_client");
   ros::NodeHandle nh;
   ros::ServiceClient client = nh.serviceClient<roboteq_motor_controller_driver::config_srv>("config_service");
   roboteq_motor_controller_driver::config_srv srv;
   srv.request.userInput = argv[1];
   srv.request.channel = atoll(argv[2]);
   srv.request.value = atoll(argv[3]);
   if (client.call(srv))
   {
     ROS_INFO("success!");
   }
   else
   {
     ROS_ERROR("Failed to call service");
     return 1;
   }
  
   return 0;
 }
