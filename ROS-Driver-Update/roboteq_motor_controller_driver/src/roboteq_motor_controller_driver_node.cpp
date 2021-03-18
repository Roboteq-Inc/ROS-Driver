// #include <roboteq_motor_controller_driver/roboteq_motor_controller_driver_node.h>
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
#include <roboteq_motor_controller_driver/querylist.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <roboteq_motor_controller_driver/channel_values.h>
#include <roboteq_motor_controller_driver/config_srv.h>
#include <roboteq_motor_controller_driver/command_srv.h>
#include <roboteq_motor_controller_driver/maintenance_srv.h>

class RoboteqDriver
{
public:
	RoboteqDriver()
	{
		initialize(); //constructor - Initialize
	}

	~RoboteqDriver()
	{
		if (ser.isOpen())
		{
			ser.close();
		}
	}

private:
	serial::Serial ser;
	std::string port;
	int32_t baud;
	ros::Publisher read_publisher;
	ros::Subscriber cmd_vel_sub;

	int channel_number_1;
	int channel_number_2;
	int frequencyH;
	int frequencyL;
	int frequencyG;
	ros::NodeHandle nh;

	void initialize()
	{

		nh.getParam("port", port);
		nh.getParam("baud", baud);
		cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &RoboteqDriver::cmd_vel_callback, this);

		connect();
	}

	void connect()
	{

		try
		{

			ser.setPort(port);
			ser.setBaudrate(baud); //get baud as param
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			ser.setTimeout(to);
			ser.open();
		}
		catch (serial::IOException &e)
		{

			ROS_ERROR_STREAM("Unable to open port ");
			ROS_INFO_STREAM("Unable to open port");
			;
		}
		if (ser.isOpen())
		{

			ROS_INFO_STREAM("Serial Port initialized\"");
		}
		else
		{
			// ROS_INFO_STREAM("HI4");
			ROS_INFO_STREAM("Serial Port is not open");
		}
		run();
	}

	void cmd_vel_callback(const geometry_msgs::Twist &msg)
	{
		std::stringstream cmd_sub;
		cmd_sub << "!G 1"
				<< " " << msg.linear.x << "_"
				<< "!G 2"
				<< " " << msg.angular.z << "_";

		ser.write(cmd_sub.str());
		ser.flush();
		ROS_INFO_STREAM(cmd_sub.str());
	}

	ros::NodeHandle n;
	ros::ServiceServer configsrv;
	ros::ServiceServer commandsrv;
	ros::ServiceServer maintenancesrv;

	bool configservice(roboteq_motor_controller_driver::config_srv::Request &request, roboteq_motor_controller_driver::config_srv::Response &response)
	{
		std::stringstream str;
		str << "^" << request.userInput << " " << request.channel << " " << request.value << "_ "
			<< "%\clsav321654987";
		ser.write(str.str());
		ser.flush();
		response.result = str.str();

		ROS_INFO_STREAM(response.result);
		return true;
	}

	bool commandservice(roboteq_motor_controller_driver::command_srv::Request &request, roboteq_motor_controller_driver::command_srv::Response &response)
	{
		std::stringstream str;
		str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
		ser.write(str.str());
		ser.flush();
		response.result = str.str();

		ROS_INFO_STREAM(response.result);
		return true;
	}

	bool maintenanceservice(roboteq_motor_controller_driver::maintenance_srv::Request &request, roboteq_motor_controller_driver::maintenance_srv::Response &response)
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

	void initialize_services()
	{
		n = ros::NodeHandle();
		configsrv = n.advertiseService("config_service", &RoboteqDriver::configservice, this);
		commandsrv = n.advertiseService("command_service", &RoboteqDriver::commandservice, this);
		maintenancesrv = n.advertiseService("maintenance_service", &RoboteqDriver::maintenanceservice, this);
	}

	void run()
	{
		initialize_services();
		std_msgs::String str1;
		ros::NodeHandle nh;
		nh.getParam("frequencyH", frequencyH);
		nh.getParam("frequencyL", frequencyL);
		nh.getParam("frequencyG", frequencyG);

		typedef std::string Key;
		typedef std::string Val;
		std::map<Key, Val> map_sH;
		nh.getParam("queryH", map_sH);

		std::stringstream ss0;
		std::stringstream ss1;
		std::stringstream ss2;
		std::stringstream ss3;
		std::vector<std::string> KH_vector;

		ss0 << "^echof 1_";
		ss1 << "# c_/\"DH?\",\"?\"";
		for (std::map<Key, Val>::iterator iter = map_sH.begin(); iter != map_sH.end(); ++iter)
		{
			Key KH = iter->first;

			KH_vector.push_back(KH);

			Val VH = iter->second;

			ss1 << VH << "_";
		}
		ss1 << "# " << frequencyH << "_";

		std::vector<ros::Publisher> publisherVecH;
		for (int i = 0; i < KH_vector.size(); i++)
		{
			publisherVecH.push_back(nh.advertise<roboteq_motor_controller_driver::channel_values>(KH_vector[i], 100));
		}

		ser.write(ss0.str());
		ser.write(ss1.str());
		ser.write(ss2.str());
		ser.write(ss3.str());

		ser.flush();
		int count = 0;
		read_publisher = nh.advertise<std_msgs::String>("read", 1000);
		sleep(2);
		ros::Rate loop_rate(5);
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
						roboteq_motor_controller_driver::channel_values Q1;

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
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roboteq_motor_controller_driver");

	RoboteqDriver driver;

	ros::waitForShutdown();

	return 0;
}
