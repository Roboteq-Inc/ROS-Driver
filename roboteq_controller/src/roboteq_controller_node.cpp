// #include <roboteq_controller/roboteq_controller_node.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>

#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <sstream>
#include <typeinfo>
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
		if (ser.isOpen()){
			ser.close();
		}
	}


private:
	ros::NodeHandle 		nh_, nh_priv_;

	ros::ServiceServer 		configsrv_;
	ros::ServiceServer 		commandsrv_;
	ros::ServiceServer 		maintenancesrv_;
	
	// Serial
	serial::Serial 			ser;
	std::string 			serial_port_;
	int32_t 				baudrate_;

	// Pub & Sub
	ros::Publisher 			read_publisher;
	ros::Subscriber 		cmd_vel_sub_;


	bool 					closed_loop_,
							diff_drive_mode_;


	double 					wheel_circumference_,
							track_width_,
							max_rpm_;

	std::string 			cmd_vel_topic_;

	int channel_number_1;
	int channel_number_2;
	int frequencyH_;
	int frequencyL_;
	int frequencyG_;


	void cmd_setup();
	void cmd_vel_callback(const geometry_msgs::Twist &);
	void power_cmd_callback(const geometry_msgs::Twist &);
	bool configservice(roboteq_controller::config_srv::Request &, roboteq_controller::config_srv::Response &);
	bool commandservice(roboteq_controller::command_srv::Request &, roboteq_controller::command_srv::Response &);
	bool maintenanceservice(roboteq_controller::maintenance_srv::Request &, roboteq_controller::maintenance_srv::Response &);
	void initialize_services();
	void run();
};

RoboteqDriver::RoboteqDriver(ros::NodeHandle nh, ros::NodeHandle nh_priv):
	nh_(nh),
	nh_priv_(nh_priv),
	wheel_circumference_(0.),
	track_width_(0.),
	max_rpm_(0.),
	frequencyH_(0),
	frequencyL_(0),
	frequencyG_(0){
	
	nh_priv.param<std::string>("serial_port", serial_port_, "dev/ttyUSB0");
	nh_priv.param("baudrate", baudrate_, 112500);

	nh_priv_.param("closed_loop", closed_loop_, false);
	nh_priv_.param("diff_drive_mode", diff_drive_mode_, false);
	if (close){
		ROS_WARN("In CLOSED-LOOP mode!!!!");
	}
	else{
		ROS_WARN("In OPEN-LOOP mode!!!!");
	}

	nh.getParam("wheel_circumference", wheel_circumference_);
	if (wheel_circumference_ <=0.0 ){
		ROS_ERROR("Inproper configuration! wheel_circumference need to be greater than zero.");
	}
	nh.getParam("track_width", track_width_);
	if (track_width_ <=0.0 ){
		ROS_ERROR("Inproper configuration! track_width need to be greater than zero.");
	}
	nh.getParam("max_rpm", max_rpm_);
	if ( max_rpm_ <=0.0 ){
		ROS_ERROR("Inproper configuration! max_rpm need to be greater than zero.");
	}

	nh_priv_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
	if (diff_drive_mode_){
		cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 10, &RoboteqDriver::cmd_vel_callback, this);
	}
	else{
		cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 10, &RoboteqDriver::power_cmd_callback, this);
	}

	// Initiate communication to serial port
	try{	
		ser.setPort(serial_port_);
		ser.setBaudrate(baudrate_); //get baud as param
		serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(timeout);
		ser.open();
	}
	catch (serial::IOException &e){
		ROS_ERROR_STREAM("Unable to open port ");
		ROS_INFO_STREAM("Unable to open port");
		ros::shutdown();
	}

	if (ser.isOpen()){
		ROS_INFO_STREAM("Serial Port initialized\"");
	}
	else{
		ROS_INFO_STREAM("Serial Port is not open");
		ros::shutdown();
	}

	cmd_setup();

	run();
}


void RoboteqDriver::cmd_setup(){
	// stop motors
	ser.write("!G 1 0\r");
	ser.write("!G 2 0\r");
	ser.write("!S 1 0\r");
	ser.write("!S 2 0\r");
	ser.flush();

	// // disable echo
	// ser.write("^ECHOF 1\r");
	// ser.flush();

	// enable watchdog timer (1000 ms) to stop if no connection
	ser.write("^RWD 1000\r");

	// set motor operating mode (1 for closed-loop speed)
	if (!closed_loop_){
		// open-loop speed mode
		ser.write("^MMOD 1 0\r");
		ser.write("^MMOD 2 0\r");
		ser.flush();
	}
	else{
		// closed-loop speed mode
		ser.write("^MMOD 1 1\r");
		ser.write("^MMOD 2 1\r");
		ser.flush();
	}

	// set encoder counts (ppr)
	std::stringstream right_enccmd;
	std::stringstream left_enccmd;
	right_enccmd << "^EPPR 1 " << 1024 << "\r";
	left_enccmd << "^EPPR 2 " << 1024 << "\r";
	ser.write(right_enccmd.str());
	ser.write(left_enccmd.str());
	ser.flush();
}


void RoboteqDriver::power_cmd_callback(const geometry_msgs::Twist &msg){
	std::stringstream cmd_str;
	if (closed_loop_){
		cmd_str << "!S 1"
				<< " " << msg.linear.x << "_"
				<< "!S 2"
				<< " " << msg.angular.z << "_";
	}
	else{
		cmd_str << "!G 1"
				<< " " << msg.linear.x << "_"
				<< "!G 2"
				<< " " << msg.angular.z << "_";
	}
	ser.write(cmd_str.str());
	ser.flush();
	ROS_INFO("[ROBOTEQ] left: %9.3f right: %9.3f", msg.linear.x, msg.angular.z);
	// ROS_INFO_STREAM(cmd_str.str());
}


void RoboteqDriver::cmd_vel_callback(const geometry_msgs::Twist &msg){
	// wheel speed (m/s)
	float right_speed = msg.linear.x + track_width_ * msg.angular.z / 2.0;
	float left_speed  = msg.linear.x - track_width_ * msg.angular.z / 2.0;
	
	// ROS_INFO("[ROBOTEQ] left: %.3f right: %.3f", left_speed, right_speed);
	std::stringstream cmd_str;
	if (!closed_loop_){
		// motor power (scale 0-1000)
		float right_power = right_speed *1000.0 *60.0/ (wheel_circumference_ * max_rpm_);
		float left_power  = left_speed  *1000.0 *60.0/ (wheel_circumference_ * max_rpm_);
	
		ROS_INFO("[ROBOTEQ] left: %9d right: %9d", (int)left_power, (int)right_power);
		
		cmd_str << "!G 1"
				<< " " << (int)left_power << "_"
				<< "!G 2"
				<< " " << (int)right_power << "_";
	}
	else{
		// motor speed (rpm)
		int32_t right_rpm = right_speed *60.0 / wheel_circumference_;
		int32_t left_rpm  = left_speed  *60.0 / wheel_circumference_;

		ROS_INFO("[ROBOTEQ] left: %9d right: %9d", left_rpm, right_rpm);
		cmd_str << "!S 1"
				<< " " << left_rpm << "_"
				<< "!S 2"
				<< " " << right_rpm << "_";
	}

	ser.write(cmd_str.str());
	ser.flush();
	// ROS_INFO_STREAM(cmd_str.str());
}


bool RoboteqDriver::configservice(roboteq_controller::config_srv::Request &request, 
									roboteq_controller::config_srv::Response &response){
	std::stringstream str;
	str << "^" << request.userInput << " " << request.channel << " " << request.value << "_ "
		<< "%\clsav321654987";
	ser.write(str.str());
	ser.flush();
	response.result = str.str();

	ROS_INFO_STREAM(response.result);
	return true;
}


bool RoboteqDriver::commandservice(roboteq_controller::command_srv::Request &request, roboteq_controller::command_srv::Response &response)
{
	std::stringstream str;
	str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
	ser.write(str.str());
	ser.flush();
	response.result = str.str();

	ROS_INFO_STREAM(response.result);
	return true;
}


bool RoboteqDriver::maintenanceservice(roboteq_controller::maintenance_srv::Request &request, roboteq_controller::maintenance_srv::Response &response)
{
	std::stringstream str;
	str << "%" << request.userInput << " "
		<< "_";
	ser.write(str.str());
	ser.flush();
	response.result = ser.read(ser.available());

	ROS_INFO_STREAM(response.result);
	return true;
}


void RoboteqDriver::initialize_services(){
	configsrv_ = nh_.advertiseService("config_service", &RoboteqDriver::configservice, this);
	commandsrv_ = nh_.advertiseService("command_service", &RoboteqDriver::commandservice, this);
	maintenancesrv_ = nh_.advertiseService("maintenance_service", &RoboteqDriver::maintenanceservice, this);
}

void RoboteqDriver::run(){
	initialize_services();
	nh_priv_.getParam("frequencyH", frequencyH_);
	nh_priv_.getParam("frequencyL", frequencyL_);
	nh_priv_.getParam("frequencyG", frequencyG_);

	assert(frequencyH_ > 0);

	typedef std::string Key;
	typedef std::string Val;
	std::map<Key, Val> map_sH;
	nh_priv_.getParam("queryH", map_sH);

	std::stringstream ss0;
	std::stringstream ss1;
	std::stringstream ss2;
	std::stringstream ss3;
	std::vector<std::string> KH_vector;

	ss0 << "^echof 1_";
	ss1 << "# c_/\"DH?\",\"?\"";
	for (std::map<Key, Val>::iterator iter = map_sH.begin(); iter != map_sH.end(); ++iter){
		Key KH = iter->first;

		KH_vector.push_back(KH);

		Val VH = iter->second;

		ss1 << VH << "_";
	}
	ss1 << "# " << frequencyH_ << "_";

	std::vector<ros::Publisher> publisherVecH;
	for (int i = 0; i < KH_vector.size(); i++)
	{
		publisherVecH.push_back(nh_.advertise<roboteq_controller::channel_values>(KH_vector[i], 100));
	}

	ser.write(ss0.str());
	ser.write(ss1.str());
	ser.write(ss2.str());
	ser.write(ss3.str());

	ser.flush();
	int count = 0;
	read_publisher = nh_.advertise<std_msgs::String>("read", 1000);
	sleep(2);

	ros::Rate loop_rate(frequencyH_);
	while (ros::ok())
	{

		ros::spinOnce();
		if (ser.available())
		{

			std_msgs::String result;
			result.data = ser.read(ser.available());

			read_publisher.publish(result);
			boost::replace_all(result.data, "\r", "");
			boost::replace_all(result.data, "+", "");

			std::vector<std::string> fields;

			std::vector<std::string> Field9;
			boost::split(fields, result.data, boost::algorithm::is_any_of("D"));

			std::vector<std::string> fields_H;
			boost::split(fields_H, fields[1], boost::algorithm::is_any_of("?"));

			if (fields_H[0] == "H")
			{

				for (int i = 0; i < publisherVecH.size(); ++i)
				{

					std::vector<std::string> sub_fields_H;

					boost::split(sub_fields_H, fields_H[i + 1], boost::algorithm::is_any_of(":"));
					roboteq_controller::channel_values Q1;

					for (int j = 0; j < sub_fields_H.size(); j++)
					{

						try
						{
							Q1.value.push_back(boost::lexical_cast<int>(sub_fields_H[j]));
						}
						catch (const std::exception &e)
						{
							count++;
							if (count > 10)
							{
								ROS_INFO_STREAM("Garbage data on Serial");
								//std::cerr << e.what() << '\n';
							}
						}
					}

					publisherVecH[i].publish(Q1);
				}
			}
		}
		loop_rate.sleep();
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "roboteq_controller");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	RoboteqDriver driver(nh, nh_priv);

	ros::waitForShutdown();

	return 0;
}
