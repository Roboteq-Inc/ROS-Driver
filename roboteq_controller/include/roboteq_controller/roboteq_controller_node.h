#pragma once

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "serial/serial.h"
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <cassert>
#include <mutex>
#include <math.h>
#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "roboteq_interfaces/msg/channelvalues.hpp"

// #include "roboteq_controller/msg/config.hpp"
// #include "roboteq_controller/msg/maintenance.hpp"

// #include "roboteq_controller/querylist.h"
// #include "roboteq_controller/channel_values.h"


using namespace std::chrono_literals;
using std::placeholders::_1;

class RoboteqDriver : public rclcpp::Node
{
public:
	explicit RoboteqDriver(const rclcpp::NodeOptions &);

	~RoboteqDriver(){
		if (ser_.isOpen()){
			ser_.close();
		}
	}

private:
	
	// Serial
	serial::Serial 			ser_;
	std::string 			serial_port_;
	int32_t 				baudrate_;

	// Pub & Sub
	rclcpp::Subscription<std_msgs::msg::Twist>::SharedPtr 		cmd_vel_sub_;
	rclcpp::Publisher<std_msgs::msg::Twist>::SharedPtr 			serial_read_pub_;
	std::vector<rclcpp::Publisher<roboteq_controller::msg::ChannelValues>::SharedPtr> query_pub_;

	rclcpp::TimerBase::SharedPtr 				timer_pub_;

	bool 					closed_loop_,
							diff_drive_mode_;

	double 					wheel_circumference_,
							track_width_,
							max_rpm_;

	std::string 			cmd_vel_topic_;

	// int channel_number_1;
	// int channel_number_2;
	int frequency_;
	std::mutex 				locker;

	void cmdSetup();
	void cmdVelCallback(const geometry_msgs::Twist &);
	void powerCmdCallback(const geometry_msgs::Twist &);
	// bool configService(roboteq_controller::config_srv::Request &, roboteq_controller::config_srv::Response &);
	// bool commandService(roboteq_controller::command_srv::Request &, roboteq_controller::command_srv::Response &);
	// bool maintenanceService(roboteq_controller::maintenance_srv::Request &, roboteq_controller::maintenance_srv::Response &);
	void initializeServices();
	void run();

	void formQuery(std::string, 
				std::map<std::string,std::string> &, 
				std::vector<ros::Publisher> &,
				std::stringstream &);

	void queryCallback	(const ros::TimerEvent &);

};