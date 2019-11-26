#include <roboteq_motor_controller_driver/roboteq_motor_controller_driver_node.h>

bool Driver::configservice(roboteq_motor_controller_driver::config_srv::Request &request, roboteq_motor_controller_driver::config_srv::Response &response) 
{
	std::stringstream str;
	str << "^" << request.userInput << " " << request.channel << " " << request.value << "_ " << "%\clsav321654987";
	ser.write(str.str());
	response.result = str.str();
	
	ROS_INFO_STREAM(response.result);
	return true;
} 

bool Driver::commandservice(roboteq_motor_controller_driver::command_srv::Request &request, roboteq_motor_controller_driver::command_srv::Response &response) 
{
	std::stringstream str;
	str << "%" << request.userInput << " "<< "_";
	ser.write(str.str());
	response.result = str.str();
	
	ROS_INFO_STREAM(response.result);
	return true;
} 

bool Driver::maintenanceservice(roboteq_motor_controller_driver::maintenance_srv::Request &request, roboteq_motor_controller_driver::maintenance_srv::Response &response) 
{
	std::stringstream str;
	str << "%" << request.userInput << " " << "_";
	ser.write(str.str());
	
	response.result = ser.read(ser.available());
	
	ROS_INFO_STREAM(response.result);
	return true;
} 
