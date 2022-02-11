#include "roboteq_controller/diff_odom_node.h"

static const std::string tag {"[RoboteQ-Odom] "};

RoboteqOdom::RoboteqOdom(ros::NodeHandle nh, ros::NodeHandle nh_priv):
	nh_(nh),
	nh_priv_(nh_priv),
	encoder_right_prev_(0),
	encoder_left_prev_(0),
	encoder_left_(0),
	encoder_right_(0),
	encoder_min_(std::numeric_limits<int64_t>::min()),
	encoder_max_(std::numeric_limits<int64_t>::max()){
	
	nh_priv_.param<std::string>("odom_frame", odom_frame_, "odom");
	nh_priv_.param<std::string>("child_frame", child_frame_, "base_footprint");

	nh_priv_.getParam("wheel_circumference", wheel_circumference_);
	if (wheel_circumference_ <=0.0 ){
		ROS_ERROR_STREAM(tag << "Inproper configuration! wheel_circumference need to be greater than zero.");
	}
	nh_priv.getParam("track_width", track_width_);
	if (track_width_ <=0.0 ){
		ROS_ERROR_STREAM(tag << "Inproper configuration! track_width need to be greater than zero.");
	}
	nh_priv.getParam("encoder_resolution", encoder_resolution_);
	if ( encoder_resolution_ <=0.0 ){
		ROS_ERROR_STREAM(tag << "Inproper configuration! encoder_resolution need to be greater than zero.");
	}
	nh_priv.getParam("max_rpm", max_rpm_);
	if ( max_rpm_ <=0.0 ){
		ROS_ERROR_STREAM(tag << "Inproper configuration! max_rpm need to be greater than zero.");
	}

	max_angular_vel_ = max_rpm_/60. *2 * M_PI;

	encoder_sub_	= nh_.subscribe("encoder_count", 10, &RoboteqOdom::encoderCallback, this);
  	odom_pub_ 		= nh_.advertise<nav_msgs::Odometry>("odom", 50);
	
	odom_.child_frame_id = child_frame_;
	odom_.header.frame_id= odom_frame_;

	current_time_ = ros::Time::now();
  	last_time_ = ros::Time::now();
}


void RoboteqOdom::encoderCallback(const roboteq_controller::channel_values& tick){
	std::lock_guard<std::mutex> lock(locker);
	
	int array_size = sizeof(tick.value)/sizeof(tick.value[0]);
	assert(array_size == 2);
	
	encoder_left_prev_ = encoder_left_;
	encoder_right_prev_= encoder_right_;
	
	encoder_left_		= tick.value[0];
	encoder_right_		= tick.value[1];
	
	ros::Duration duration = tick.header.stamp - odom_.header.stamp;
	double dt		= duration.sec + (double)duration.nsec/1.0e-9;
	
	// Linear velocity of two wheels
	double vel_left = 2* M_PI * (encoder_left_ - encoder_left_prev_)/encoder_resolution_/dt;
	double vel_right= 2* M_PI * (encoder_right_- encoder_right_prev_)/encoder_resolution_/dt;

	assert (vel_left <= max_angular_vel_ && vel_right <= max_angular_vel_);

	// Linear velocity and angular velocity of robot
	double velocity = wheel_circumference_/4 * (vel_left + vel_right);
	double angular  = wheel_circumference_/track_width_/2 *(vel_right - vel_left);
	
	tf::Quaternion q(
        odom_.pose.pose.orientation.x,
        odom_.pose.pose.orientation.y,
        odom_.pose.pose.orientation.z,
        odom_.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

	// Velocity in x and y directions
	double velocity_x		= velocity * cos(yaw);
	double velocity_y 		= velocity * sin(yaw);

	// Update robot's pose
	tf2::Quaternion q_rot;
	q_rot.setRPY(roll, pitch, yaw + angular * dt);


	// geometry_msgs::Quaternion odom_quat ;
	// //first, we'll publish the transform over tf
	// geometry_msgs::TransformStamped odom_trans;
	// odom_trans.header.stamp = now;
	// odom_trans.header.frame_id = "odom";
	// odom_trans.child_frame_id = "base_footprint";

	// odom_trans.transform.translation.x = x_final;
	// odom_trans.transform.translation.y = y_final;
	// odom_trans.transform.translation.z = 0.0;
	// odom_trans.transform.rotation = odom_quat;

	// //send the transform
	// odom_broadcaster.sendTransform(odom_trans);
	
	//Publish the odometry message over ROS
	
	odom_.header.stamp 			= tick.header.stamp;
	//set the position
	odom_.pose.pose.position.x 	+= velocity_x * dt;
	odom_.pose.pose.position.y 	+= velocity_y * dt;
	odom_.pose.pose.orientation.x = q_rot.getX();
	odom_.pose.pose.orientation.y = q_rot.getY();
	odom_.pose.pose.orientation.z = q_rot.getZ();
	odom_.pose.pose.orientation.w = q_rot.getW();

	//set the velocity
	odom_.twist.twist.linear.x = velocity_x;
	odom_.twist.twist.linear.y = velocity_y;
	odom_.twist.twist.angular.z = angular;

	//publish the message
	odom_pub_.publish(odom_);
}


int main(int argc, char **argv)

{
	ros::init(argc, argv,"diff_odom_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	RoboteqOdom obj(nh, nh_priv);
	
	ros::spin();
	return 0;

}
