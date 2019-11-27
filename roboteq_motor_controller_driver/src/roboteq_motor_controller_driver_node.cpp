#include <roboteq_motor_controller_driver/roboteq_motor_controller_driver_node.h>
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


serial::Serial ser;

using namespace roboteq;



void Driver::connect(){    


    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();

    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        ROS_INFO_STREAM("Unable to open port");;
    }
if(ser.isOpen()){

        ROS_INFO_STREAM("Serial Port initialized\"");
        
        
       
        }else{
        ROS_INFO_STREAM("Serial Port is not open");
    }
 }
 
void Driver::cmd_vel_callback(const geometry_msgs::Twist& msg){
	std::stringstream cmd_sub;
	cmd_sub << "!G 1" << " " << msg.linear.x << "_" << "!G 2" << " " << msg.angular.z << "_";
    	
    	ser.write(cmd_sub.str());
	ROS_INFO_STREAM(cmd_sub.str());	
} 
void Driver::roboteq_subscriber(){
	ros::NodeHandle n; 
 	cmd_vel_sub = n.subscribe("/cmd_vel",10,&Driver::cmd_vel_callback,this);
 	
}

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

void Driver::roboteq_services(){
	ros::NodeHandle n;
	configsrv = n.advertiseService("config_service", &Driver::configservice, this);
	commandsrv = n.advertiseService("command_service", &Driver::commandservice, this);
	maintenancesrv = n.advertiseService("maintenance_service", &Driver::maintenanceservice, this);
}



 
void Driver::run(){
 
    
    std_msgs::String str1;
	ros::NodeHandle nh; 
	nh.getParam("frequencyH",frequencyH);
	nh.getParam("frequencyL",frequencyL);
	nh.getParam("frequencyG",frequencyG);
	
	
	typedef std::string Key;
	typedef std::string Val;
	std::map<Key,Val> map_sH;
	nh.getParam("queryH", map_sH);
	
	
	
	
	
	std::stringstream ss1;
	std::stringstream ss2;
	std::stringstream ss3;
	std::vector<std::string> KH_vector;
	
	ss1 << "# c_/\"DH:\",\":\"";
	for(std::map<Key,Val>::iterator iter = map_sH.begin(); iter != map_sH.end(); ++iter)
	{
	Key KH =  iter->first;
		//ROS_INFO_STREAM(KH);
	KH_vector.push_back(KH);
    	
	Val VH = iter->second;	
		ss1 << VH << "_";
    
	}
	ss1 << "# " << frequencyH << "_";
	
	
	std::vector<ros::Publisher> publisherVecH;
    	for (int i = 0; i< KH_vector.size(); i++){
    	 publisherVecH.push_back(nh.advertise<roboteq_motor_controller_driver::channel_values>(KH_vector[i],100));
	}
	
	
	std::map<Key,Val> map_sL;
	nh.getParam("queryL", map_sL);
	std::vector<std::string> KL_vector;
	ss2 << 	"/\"DL:\",\":\"";
	for(std::map<Key,Val>::iterator iter = map_sL.begin(); iter != map_sL.end(); ++iter)
	{
	Key KL = iter->first;
		KL_vector.push_back(KL);
		//ROS_INFO_STREAM(KL);
	Val VL = iter->second;
		ss2 << VL << "_";
		
		
	}
	ss2 << "# " << frequencyL << "_";
	
	
	std::vector<ros::Publisher> publisherVecL;
	for(int i=0; i<KL_vector.size(); ++i){
		publisherVecL.push_back(nh.advertise<roboteq_motor_controller_driver::channel_values>(KL_vector[i],100));
	}
		
	
	std::map<Key,Val> map_sG;
	nh.getParam("queryG", map_sG);
	std::vector<std::string> KG_vector;
	ss3 << 	"/\"DG:\",\":\"";
	for(std::map<Key,Val>::iterator iter = map_sG.begin(); iter != map_sG.end(); ++iter)
	{
	Key KG = iter->first;
		KG_vector.push_back(KG);
		ROS_INFO_STREAM(KG);
	Val VG = iter->second;
		ss3 << VG << "_";
		
		
	}
	ss3 << "# " << frequencyG << "_";
	
	
	std::vector<ros::Publisher> publisherVecG;
	for(int i=0; i<KG_vector.size(); ++i){
		publisherVecG.push_back(nh.advertise<std_msgs::String>(KG_vector[i],100));
	}
	
    	
    	
    	ser.write(ss1.str());
    	ser.write(ss2.str());
    	ser.write(ss3.str());
    	read_publisher = nh.advertise<std_msgs::String>("read", 1000);
    	
    	
    	
    	
    	
   ros::Rate loop_rate(5);
    while(ros::ok()){
        
        
        ros::spinOnce();
        if(ser.available()){

            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
           
            
            read_publisher.publish(result);
            boost::replace_all(result.data, "\r", "");
            std::vector<std::string> fields;
            std::vector<std::string> Field9;
	    boost::split(fields, result.data, boost::algorithm::is_any_of("D"));
	    /*ROS_INFO_STREAM(fields[1]);
	    ROS_INFO_STREAM(fields[2]);
	    ROS_INFO_STREAM(fields[3]);*/
	    
	    std::vector<std::string> fields_H;
	    boost::split(fields_H, fields[1], boost::algorithm::is_any_of(":"));
	    
	    if (fields_H[0] == "H")
	    {
	    	
	    	
	    	for(int i=0; i< publisherVecH.size(); ++i){
	    	
	    	roboteq_motor_controller_driver::channel_values Q1;
	    	
	    	Q1.channel_1 = boost::lexical_cast<int>(fields_H[2*i+1]);
	    	Q1.channel_2 = boost::lexical_cast<int>(fields_H[2*i+2]);
	    	publisherVecH[i].publish(Q1);
	    	
	    	
	    	}
	    
	    
	    }
	    else
	    { 
	    	ROS_INFO_STREAM("Garbage data on Serial");
	    }
	    
	    std::vector<std::string> fields_L;
	    std::vector<std::string> fields_G;
	    boost::split(fields_G, fields[3], boost::algorithm::is_any_of(":"));
	    boost::split(fields_L, fields[2], boost::algorithm::is_any_of(":"));
	    
	    
	    
	    //if (fields_L[0] == "L" || fields_G[0] == "L" )
	    if (fields_L[0] == "L")
	    {
	    	

	    	for(int i=0; i< publisherVecL.size(); ++i){
	    	roboteq_motor_controller_driver::channel_values Q1;
	    	Q1.channel_1 = boost::lexical_cast<int>(fields_L[2*i+1]);
	    	Q1.channel_2 = boost::lexical_cast<int>(fields_L[2*i+2]);
	    	publisherVecL[i].publish(Q1);
	    	
	    	
	    	}	    
            
	    
	    }
	     
            
	    if (fields_G[0] == "G" )
	    {
	    	

	    	for(int i=0; i< publisherVecG.size(); ++i){
	    	std_msgs::String Q1;
	    	Q1.data = fields_G[i+1];
	    	//ROS_INFO_STREAM(Q1);
	    	publisherVecG[i].publish(Q1);
	    	
	    	
	    	}	    
		
	    }
	    
	    
	ROS_INFO_STREAM("success!");
	
	    
	Driver::roboteq_subscriber();

        }
        loop_rate.sleep();
	ROS_INFO_STREAM("Type the command - \"rostopic list\" - in new terminal for publishers");  
	
    }
    }







