#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <actionlib/client/simple_action_client.h>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <angles/angles.h>

#include <nav_core/recovery_behavior.h>
//#include "occupancy_grid_subscriber.cpp"
#include <nav_msgs/Odometry.h>
#include <bits/stdc++.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_layer.h>
#include "detection_replan.cpp"

int main(int argc,char ** argv)
{
	ros::init(argc, argv, "way_points");
	// class object to update the ocupancy grid
	detection_plan::DetectionPlan replann;
	 // class object to replan the global plan by subscribing to laser , base_pose_ground,path of plan ,goal,  
	ros::spin();
  	return 0;
	
}
