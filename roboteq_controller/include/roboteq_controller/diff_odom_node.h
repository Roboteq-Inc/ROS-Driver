#pragma once

#include <math.h>
#include <limits>
#include <mutex>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <roboteq_controller/channel_values.h>
#include <roboteq_controller/config_srv.h>
#include <roboteq_controller/command_srv.h>
#include <roboteq_controller/maintenance_srv.h>


class RoboteqOdom{

public:
	RoboteqOdom(ros::NodeHandle, ros::NodeHandle );

	void spin();

private:
	ros::NodeHandle nh_, nh_priv_;
	ros::Subscriber encoder_sub_;
	ros::Publisher odom_pub_;

	tf::TransformBroadcaster odom_broadcaster;
	

    int64_t             encoder_left_prev_,
                        encoder_right_prev_,
                        encoder_left_,
                        encoder_right_;
    
    double              wheel_circumference_,
                        track_width_,
                        encoder_resolution_,
                        max_rpm_,
                        max_angular_vel_;

    const int64_t       encoder_min_,
                        encoder_max_;

	ros::Time current_time_, last_time_;
    
    std::string         odom_frame_, child_frame_;
    nav_msgs::Odometry odom_;

    std::mutex          locker;
	void encoderCallback(const roboteq_controller::channel_values& );
};