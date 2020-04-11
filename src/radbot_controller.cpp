#include <stdio.h>
#include <math.h>
#include <vector>
#include <tuple>
#include <limits>
#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <boost/thread/mutex.hpp>

#include "radbot_controller.hpp"

int main(int argc, char** argv) 
{
	// start the node
	ros::init(argc, argv, "radbot_controller");
	// Create a the planner obj
	ros::NodeHandlePtr nh_ptr_(new ros::NodeHandle("~"));
	// Create a the planner obj
	radbot_controller controller(nh_ptr_);
	// spin ros
	ros::spin();
	return 0;
} 