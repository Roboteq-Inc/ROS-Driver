#include "ros/ros.h"
#include <roboteq_controller/maintenance_srv.h>
 
int main(int argc, char **argv)
 {
   ros::init(argc, argv, "maintenance_client");
   ros::NodeHandle nh;
   ros::ServiceClient client = nh.serviceClient<roboteq_controller::maintenance_srv>("maintenance_service");
   roboteq_controller::maintenance_srv srv;
   srv.request.userInput = argv[1];

   
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
