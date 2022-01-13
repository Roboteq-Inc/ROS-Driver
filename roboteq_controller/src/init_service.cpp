#include "roboteq_controller/init_service.h"

bool Driver::configservice(roboteq_controller::config_srv::Request &request, roboteq_controller::config_srv::Response &response) 
{
	std::stringstream str;
	str << "^" << request.userInput << " " << request.channel << " " << request.value << "_ " << "%\clsav321654987";
	ser.write(str.str());
	response.result = str.str();
	
	ROS_INFO_STREAM(response.result);
	return true;
} 

bool Driver::commandservice(roboteq_controller::command_srv::Request &request, roboteq_controller::command_srv::Response &response) 
{
	std::stringstream str;
	str << "%" << request.userInput << " "<< "_";
	ser.write(str.str());
	response.result = str.str();
	
	ROS_INFO_STREAM(response.result);
	return true;
} 

bool Driver::maintenanceservice(roboteq_controller::maintenance_srv::Request &request, roboteq_controller::maintenance_srv::Response &response) 
{
	std::stringstream str;
	str << "%" << request.userInput << " " << "_";
	ser.write(str.str());
	
	response.result = ser.read(ser.available());
	
	ROS_INFO_STREAM(response.result);
	return true;
} 
