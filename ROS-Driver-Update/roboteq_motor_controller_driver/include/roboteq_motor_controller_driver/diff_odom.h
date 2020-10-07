#include "ros/ros.h"

class Odometry_calc{

public:
	Odometry_calc();

	void spin();


private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Subscriber l_wheel_sub;
	ros::Subscriber r_wheel_sub;
	ros::Publisher odom_pub;

	tf::TransformBroadcaster odom_broadcaster;
	//Encoder related variables
	double encoder_min;
	double encoder_max;

	double encoder_low_wrap;
	double encoder_high_wrap;

	double prev_lencoder;
	double prev_rencoder;

	double lmult;
	double rmult;

	double left;
	double right;

	double rate;

	ros::Duration t_delta;

	ros::Time t_next;

	ros::Time then;


	double enc_left ;

	double enc_right;

	double ticks_meter;

	double base_width;

	double dx;

	double dr;

	double x_final,y_final, theta_final;

	ros::Time current_time, last_time;


	void leftencoderCb(const roboteq_motor_controller_driver::channel_values& left_ticks);

	void rightencoderCb(const roboteq_motor_controller_driver::channel_values& right_ticks);
	//void rightencoderCb(std_msgs::Int64::ConstPtr& right_ticks);
	void init_variables();

	void get_node_params();


	void update();
};

