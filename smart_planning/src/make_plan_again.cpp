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


//make plan method
class my_plan : public nav_core::BaseGlobalPlanner
{
	public:

	my_plan(){};
	  //my_plan(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  
	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){};
	bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan)
        		{
				plan.push_back(start);
   				for (int i=0; i<20; i++){
    				 geometry_msgs::PoseStamped new_goal = goal;
    				 tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);
 
    				  new_goal.pose.position.x = -2.5+(0.05*i);
    				  new_goal.pose.position.y = -3.5+(0.05*i);

    				  new_goal.pose.orientation.x = goal_quat.x();
    				  new_goal.pose.orientation.y = goal_quat.y();
    				  new_goal.pose.orientation.z = goal_quat.z();
    				  new_goal.pose.orientation.w = goal_quat.w();

   						plan.push_back(new_goal);
   					}
   				plan.push_back(goal);
   						//ROS_INFO("makePlan function is Working:");
	  			return true;

		}
};		//END OF MY_PLAN CLASS
