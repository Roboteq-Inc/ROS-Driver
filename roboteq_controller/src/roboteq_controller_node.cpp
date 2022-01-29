#include "roboteq_controller/roboteq_controller_node.h"

static const std::string tag {"[RoboteQ] "};


RoboteqDriver::RoboteqDriver(const rclcpp::NodeOptions &options): Node("roboteq_controller"),
	wheel_circumference_(0.),
	track_width_(0.),
	max_rpm_(0.),
	frequency_(0),
	serial_port_("dev/ttyUSB0"),
	baudrate_(112500),
	closed_loop_(false),
	diff_drive_mode_(false),
	cmd_vel_topic_("/cmd_vel"){
	
	get_parameter("serial_port", serial_port_);
	get_parameter("baudrate", baudrate_);

	get_parameter("closed_loop", closed_loop_);
	get_parameter("diff_drive_mode", diff_drive_mode_);
	if (close){
		RCLCPP_WARN_STREAM(this->get_logger(),tag << "In CLOSED-LOOP mode!!!!");
	}
	else{
		RCLCPP_WARN_STREAM(this->get_logger(),tag << "In OPEN-LOOP mode!!!!");
	}

	get_parameter("wheel_circumference", wheel_circumference_);
	if (wheel_circumference_ <=0.0 ){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Inproper configuration! wheel_circumference need to be greater than zero.");
	}
	get_parameter("track_width", track_width_);
	if (track_width_ <=0.0 ){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Inproper configuration! track_width need to be greater than zero.");
	}
	get_parameter("max_rpm", max_rpm_);
	if ( max_rpm_ <=0.0 ){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Inproper configuration! max_rpm need to be greater than zero.");
	}

	get_parameter("cmd_vel_topic", cmd_vel_topic_);
	if (diff_drive_mode_){
		cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 10, &RoboteqDriver::cmdVelCallback, this);
	}
	else{
		cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 10, &RoboteqDriver::powerCmdCallback, this);
	}

	// Initiate communication to serial port
	try{	
		ser_.setPort(serial_port_);
		ser_.setBaudrate(baudrate_);
		serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
		ser_.setTimeout(timeout);
		ser_.open();
	}
	catch (serial::IOException &e){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Unable to open port " << serial_port_);
		RCLCPP_INFO_STREAM(this->get_logger(),tag << "Unable to open port" << serial_port_);
		ros::shutdown();
	}

	if (ser_.isOpen()){
		RCLCPP_INFO_STREAM(this->get_logger(),tag << "Serial Port " << serial_port_ << " initialized");
	}
	else{
		RCLCPP_INFO_STREAM(this->get_logger(),tag << "Serial Port " << serial_port_ << " is not open");
		ros::shutdown();
	}

	cmdSetup();

	run();
}


void RoboteqDriver::cmdSetup(){
	// stop motors
	ser_.write("!G 1 0\r");
	ser_.write("!G 2 0\r");
	ser_.write("!S 1 0\r");
	ser_.write("!S 2 0\r");
	ser_.flush();

	// // disable echo
	// ser.write("^ECHOF 1\r");
	// ser.flush();

	// enable watchdog timer (1000 ms) to stop if no connection
	ser_.write("^RWD 1000\r");

	// set motor operating mode (1 for closed-loop speed)
	if (!closed_loop_){
		// open-loop speed mode
		ser_.write("^MMOD 1 0\r");
		ser_.write("^MMOD 2 0\r");
		ser_.flush();
	}
	else{
		// closed-loop speed mode
		ser_.write("^MMOD 1 1\r");
		ser_.write("^MMOD 2 1\r");
		ser_.flush();
	}

	// set encoder counts (ppr)
	std::stringstream right_enccmd;
	std::stringstream left_enccmd;
	right_enccmd << "^EPPR 1 " << 1024 << "\r";
	left_enccmd << "^EPPR 2 " << 1024 << "\r";
	ser_.write(right_enccmd.str());
	ser_.write(left_enccmd.str());
	ser_.flush();
}


void RoboteqDriver::run(){
	initializeServices();
	nh_priv_.getParam("frequency", frequency_);

	if (frequency_ > 0){
		std::stringstream ss0, ss1;
		ss0 << "^echof 1_";
		ss1 << "# c_/\"DH?\",\"?\"";
		std::map<std::string, std::string> query_map;
		formQuery("query", query_map, query_pub_, ss1);

		ss1 << "# " << frequency_ << "_";
		
		ser_.write(ss0.str());
		ser_.write(ss1.str());
		ser_.flush();
	}
	
    serial_read_pub_ = this->create_publisher<std_msgs::msg::String>("read", 100);

	
	if (frequency_ > 0){
		timer_pub_ = this->create_wall_timer(
			rclcpp::Duration(frequency_/ 1000.), 
			std::bind(&RoboteqDriver::queryCallback, this));
	}
}


void RoboteqDriver::powerCmdCallback(const geometry_msgs::Twist &msg){
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
	ser_.write(cmd_str.str());
	ser_.flush();
	RCLCPP_INFO(this->get_logger(),"[ROBOTEQ] left: %9.3f right: %9.3f", msg.linear.x, msg.angular.z);
	// RCLCPP_INFO_STREAM(this->get_logger(),cmd_str.str());
}


void RoboteqDriver::cmdVelCallback(const geometry_msgs::Twist &msg){
	// wheel speed (m/s)
	float right_speed = msg.linear.x + track_width_ * msg.angular.z / 2.0;
	float left_speed  = msg.linear.x - track_width_ * msg.angular.z / 2.0;
	
	// RCLCPP_INFO(this->get_logger(),("[ROBOTEQ] left: %.3f right: %.3f", left_speed, right_speed);
	std::stringstream cmd_str;
	if (!closed_loop_){
		// motor power (scale 0-1000)
		float right_power = right_speed *1000.0 *60.0/ (wheel_circumference_ * max_rpm_);
		float left_power  = left_speed  *1000.0 *60.0/ (wheel_circumference_ * max_rpm_);
	
		RCLCPP_INFO(this->get_logger(),"[ROBOTEQ] left: %9d right: %9d", (int)left_power, (int)right_power);
		
		cmd_str << "!G 1"
				<< " " << (int)left_power << "_"
				<< "!G 2"
				<< " " << (int)right_power << "_";
	}
	else{
		// motor speed (rpm)
		int32_t right_rpm = right_speed *60.0 / wheel_circumference_;
		int32_t left_rpm  = left_speed  *60.0 / wheel_circumference_;

		RCLCPP_INFO(this->get_logger(),"[ROBOTEQ] left: %9d right: %9d", left_rpm, right_rpm);
		cmd_str << "!S 1"
				<< " " << left_rpm << "_"
				<< "!S 2"
				<< " " << right_rpm << "_";
	}

	ser_.write(cmd_str.str());
	ser_.flush();
	// RCLCPP_INFO_STREAM(this->get_logger(),cmd_str.str());
}


// bool RoboteqDriver::configService(roboteq_controller::config_srv::Request &request, 
// 									roboteq_controller::config_srv::Response &response){
// 	std::stringstream str;
// 	str << "^" << request.userInput << " " << request.channel << " " << request.value << "_ "
// 		<< "%\clsav321654987";
// 	ser_.write(str.str());
// 	ser_.flush();
// 	response.result = str.str();

// 	RCLCPP_INFO_STREAM(this->get_logger(),tag << response.result);
// 	return true;
// }


// bool RoboteqDriver::commandService(roboteq_controller::command_srv::Request &request, roboteq_controller::command_srv::Response &response)
// {
// 	std::stringstream str;
// 	str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
// 	ser_.write(str.str());
// 	ser_.flush();
// 	response.result = str.str();

// 	RCLCPP_INFO_STREAM(this->get_logger(),tag << response.result);
// 	return true;
// }


// bool RoboteqDriver::maintenanceService(roboteq_controller::maintenance_srv::Request &request, roboteq_controller::maintenance_srv::Response &response)
// {
// 	std::stringstream str;
// 	str << "%" << request.userInput << " "
// 		<< "_";
// 	ser_.write(str.str());
// 	ser_.flush();
// 	response.result = ser_.read(ser_.available());

// 	RCLCPP_INFO_STREAM(this->get_logger(),response.result);
// 	return true;
// }


// void RoboteqDriver::initializeServices(){
// 	configsrv_ 			= nh_.advertiseService("config_service", &RoboteqDriver::configService, this);
// 	commandsrv_ 		= nh_.advertiseService("command_service", &RoboteqDriver::commandService, this);
// 	maintenancesrv_ 	= nh_.advertiseService("maintenance_service", &RoboteqDriver::maintenanceService, this);
// }

void RoboteqDriver::formQuery(std::string param, 
							std::map<std::string,std::string> &queries, 
							std::vector<ros::Publisher> &pubs,
							std::stringstream &ser_str){
	nh_priv_.getParam(param, queries);
	for (std::map<std::string, std::string>::iterator iter = queries.begin(); iter != queries.end(); iter++){
		RCLCPP_INFO_STREAM(this->get_logger(),tag << "Publish topic: " << iter->first);
		pubs.push_back(nh_.advertise<roboteq_interfaces::msg::ChannelValues>(iter->first, 100));

		std::string cmd = iter->second;
		ser_str << cmd << "_";
	}
}


void RoboteqDriver::queryCallback(const ros::TimerEvent &){
	int count = 0;
	ros::Time current_time = ros::Time::now();
	if (ser_.available()){


		std_msgs::String result;

		std::lock_guard<std::mutex> lock(locker);

		result.data = ser_.read(ser_.available());

		// std::lock_guard<std::mutex> unlock(locker);


		serial_read_pub_.publish(result);
		
		boost::replace_all(result.data, "\r", "");
		boost::replace_all(result.data, "+", "");

		std::vector<std::string> fields;
		
		boost::split(fields, result.data, boost::algorithm::is_any_of("D"));
		if (fields.size() < 2){

			RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Empty data:{" << result.data << "}");
		}
		else if (fields.size() >= 2){
			std::vector<std::string> fields_H;
			for (int i = fields.size() - 1; i >= 0; i--){
				if (fields[i][0] == 'H'){
					try{
						fields_H.clear();
						boost::split(fields_H, fields[i], boost::algorithm::is_any_of("?"));
						if ( fields_H.size() >= query_pub_.size() + 1){
							break;
						}
					}
					catch (const std::exception &e){
						std::cerr << e.what() << '\n';
						RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Finding query output in :" << fields[i]);
						continue;
					}
				}
			}

			if (fields_H[0] == "H"){
				for (int i = 0; i < query_pub_.size(); ++i){
					std::vector<std::string> sub_fields_H;
					boost::split(sub_fields_H, fields_H[i + 1], boost::algorithm::is_any_of(":"));
					
					roboteq_interfaces::msg::ChannelValues msg;
					msg.header.stamp = current_time;

					for (int j = 0; j < sub_fields_H.size(); j++){
						try{
							msg.value.push_back(boost::lexical_cast<int>(sub_fields_H[j]));
						}
						catch (const std::exception &e){
							RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Garbage data on Serial");
							std::cerr << e.what() << '\n';
						}
					}
					query_pub_[i].publish(msg);
				}
			}
		}
		else{
			RCLCPP_WARN_STREAM(this->get_logger(),tag << "Unknown:{" << result.data << "}");
		}
	}
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoboteqDriver>());
  rclcpp::shutdown();
  return 0;
}

