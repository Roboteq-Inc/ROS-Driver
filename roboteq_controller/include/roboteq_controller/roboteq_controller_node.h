#pragma once

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <serial/serial.h>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <cassert>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "roboteq_controller/querylist.h"
#include "roboteq_controller/channel_values.h"
#include "roboteq_controller/config_srv.h"
#include "roboteq_controller/command_srv.h"
#include "roboteq_controller/maintenance_srv.h"


class RoboteqDriver
{
public:
	RoboteqDriver(ros::NodeHandle, ros::NodeHandle);
	~RoboteqDriver(){
		if (ser_.isOpen()){
			ser_.close();
		}
	}

private:
	ros::NodeHandle 		nh_, nh_priv_;

	ros::ServiceServer 		configsrv_;
	ros::ServiceServer 		commandsrv_;
	ros::ServiceServer 		maintenancesrv_;
	
	// Serial
	serial::Serial 			ser_;
	std::string 			serial_port_;
	int32_t 				baudrate_;

	// Pub & Sub
	ros::Subscriber 		cmd_vel_sub_;
	ros::Publisher 			serial_read_pub_;
	std::vector<ros::Publisher> query_pub_;

	ros::Timer 				timer_pub_;


	bool 					closed_loop_,
							diff_drive_mode_;

	double 					wheel_circumference_,
							track_width_,
							max_rpm_;

	std::string 			cmd_vel_topic_;

	int channel_number_1;
	int channel_number_2;
	int frequency_;


	void cmdSetup();
	void cmdVelCallback(const geometry_msgs::Twist &);
	void powerCmdCallback(const geometry_msgs::Twist &);
	bool configService(roboteq_controller::config_srv::Request &, roboteq_controller::config_srv::Response &);
	bool commandService(roboteq_controller::command_srv::Request &, roboteq_controller::command_srv::Response &);
	bool maintenanceService(roboteq_controller::maintenance_srv::Request &, roboteq_controller::maintenance_srv::Response &);
	void initializeServices();
	void run();

	void formQuery(std::string, 
				std::map<std::string,std::string> &, 
				std::vector<ros::Publisher> &,
				std::stringstream &);

	void queryCallback	(const ros::TimerEvent &);

};