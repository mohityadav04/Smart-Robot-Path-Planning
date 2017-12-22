
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
#include "make_plan_again.cpp"
#include "costmap_recovery.cpp"


namespace detection_plan{


class DetectionPlan{


public:
  ros::NodeHandle nh_;
bool reached;
  //ros::NodeHandle private_nh;
  ros::Subscriber way_sub;
  ros::Subscriber goal_sub;
  ros::Subscriber start_sub;
  ros::Subscriber plan_sub;
  geometry_msgs::PoseStamped mstart;
  geometry_msgs::PoseStamped mgoal;
  ros::Subscriber laser_sub;
  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_update_sub_;
  nav_msgs::OccupancyGrid grid_;
  bool is_grid_initialized;
ros::Time begin,last;
double dist_travelled;
bool first_localised_point;

  double goal_x;
  double goal_y;

  std::vector<geometry_msgs::PoseStamped> mplan;
  double start_x,start_y;

  double prev_start_x,prev_start_y;
  int count;
  double distance;
  bool check;
  sensor_msgs::LaserScan::ConstPtr custom_scan;
  //const sensor_msgs::LaserScan::ConstPtr&  custom_scan=*custom_scan;

  double get_distance(double x1,double y1,double x2,double y2){ // auxillary function to get distance between two points
    return sqrt(pow((x2-x1),2)+ pow((y2-y1),2));
  }


  int getIndex(int x, int y){
    int sx = grid_.info.width;
    return y * sx + x;
  }

 
  void grid_callback(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    // ROS_DEBUG("		OccupancyGrid Subscriber callback Entry");
    if(!is_grid_initialized)
      is_grid_initialized =true;
    grid_ = *msg;
    //ROS_DEBUG("		OccupancyGrid Subscriber callback is Working");
  }

  void update_callback(const map_msgs::OccupancyGridUpdateConstPtr& msg)
  {
    if(!is_grid_initialized)
    {
      ROS_WARN(" Occupancy grid has not been initialized. Rejecting updates till that");
      return;
    }
    int index = 0;
    //  //ROS_INFO("			OccupancyGrid Subscriber update callback entry");
 
    ROS_DEBUG_STREAM(" Size of occupancy grid update is width="<<msg->width<<" heigth="<<msg->height<<",  origin x="<<msg->x<<" y="<<msg->y);
    ROS_DEBUG_STREAM(" Size of known occupacny grid is width="<<grid_.info.width<<"heigth="<<grid_.info.height<<" data size="<<grid_.data.size());
    //grid_.data.reserve(numeric);
    for(int y=msg->y; y< msg->y+msg->height; y++)
    {
      for(int x=msg->x; x< msg->x+msg->width; x++)
      {
        int index_other = getIndex(x,y);
        //ROS_INFO_STREAM_THROTTLE(1,"y "<<y<<" x "<<x<< " index="<<index_other);
        grid_.data[index_other] = msg->data[ index++ ];
      }
    }
    // //ROS_INFO("			OccupancyGrid Subscriber update callback exit");
    		//nav_msgs::OccupancyGrid* getOccupancyGrid(){ return &grid_; }
  }



  void way_callback(const nav_msgs::Path& msg)
  { // this is the callback for the plan
    //ROS_INFO(" way_callback entry");



    tf::TransformListener listener;
    geometry_msgs::PointStamped input_p;

    geometry_msgs::PointStamped* pointer_input_p = &input_p;
    geometry_msgs::PointStamped output_p;
    geometry_msgs::PointStamped* pointer_output_p = &output_p;

    tf::StampedTransform transform;



    //ROS_INFO("way_callback is Working:");


    //for(int i=(msg.poses.size())*count;i<=(msg.poses.size())*(count+1)-1;i +=1)
    for(int i=0;i<msg.poses.size();i++)
    {

      //int i=count;
      input_p.point.x = msg.poses[i].pose.position.x;
      input_p.point.y = msg.poses[i].pose.position.y;
      input_p.header.frame_id = "/odom";
      try
      {
        listener.waitForTransform("/odom", "/map", ros::Time(0),
                                  ros::Duration(10.0));
        listener.lookupTransform("/odom", "/map", ros::Time(0),transform);
        listener.transformPoint("/map",*pointer_input_p,*pointer_output_p );
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR_STREAM(" Unable to receive a transform lookup from frame /odom to /map. Exception:  "<<ex.what()<<" Exiting ");
        exit(-1);
      }

      if( get_distance(start_x,start_y,output_p.point.x,output_p.point.y)<5)
      {

        //ROS_INFO("both start & path %0.2f \t %0.2f \t% 0.2f \t% 0.2f",start_x,start_y,output_p.point.x,output_p.point.y);
        distance = get_distance(start_x,start_y,output_p.point.x,output_p.point.y);
        //ROS_INFO("distance is 	 %lf",distance);
        const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
        const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
        int minIndex = ceil((MIN_SCAN_ANGLE_RAD - custom_scan->angle_min) / custom_scan->angle_increment);
        int maxIndex = floor((MAX_SCAN_ANGLE_RAD - custom_scan->angle_min) / custom_scan->angle_increment);
        //float closestRange = scan->ranges[minIndex];
        for (int currIndex = minIndex + 1,check=false; currIndex <= maxIndex; currIndex++)
        {
          if (custom_scan->ranges[currIndex]<5) //if obstacle is detected
          {


            my_plan my_plan_1; //my_plan object to replan
            bool xint;
            xint = my_plan_1.makePlan(mstart,mgoal,mplan); //Replanning
            //ROS_INFO("Obstacle detected at distance  %f",custom_scan->ranges[currIndex]);
            break;
          }
        }

      }

    }
if(get_distance(start_x,start_y,goal_x,goal_y) <= 0.5){

	/*//ROS_INFO_STREAM("recovery"<<start_x<<start_y<<goal_x<<goal_y);
      tf::TransformListener tf(ros::Duration(10));
      costmap_2d::Costmap2DROS global_costmap("global_costmap", tf);
      costmap_2d::Costmap2DROS local_costmap("local_costmap", tf);

      clear_costmap_recovery::ClearCostmapRecovery costmap_clear;
      costmap_clear.initialize("my_clear_costmap_recovery", &tf, &global_costmap, &local_costmap);
      costmap_clear.runBehavior();*/
	if(!reached)
	{		
		reached =true;	
		last = ros::Time::now();
		ROS_INFO_STREAM("this is the time take "<<last-begin << " distance is: " <<dist_travelled);
		dist_travelled=0;
	}
	


    }
    
  }




  void localizationCallback(const nav_msgs::Odometry& mstart ) // for getting the position of robot
  {

    ROS_DEBUG_THROTTLE(2, "RootLocalization message subscription call back entry ");
    ////ROS_INFO("start is Working");

    start_x = mstart.pose.pose.position.y;
    start_y = -mstart.pose.pose.position.x;
	if(first_localised_point){
		prev_start_x=start_x;
		prev_start_y=start_y;
		first_localised_point=false;
	}
	
	dist_travelled += get_distance(prev_start_x,prev_start_y,start_x,start_y);
	//ROS_INFO_STREAM("distance transition is: "<<dist_travelled);
	//ROS_INFO_STREAM("points are "<<prev_start_x<<"	"<<prev_start_y<<"	"<<start_x<<"	"<<start_y);
	prev_start_x = start_x;
	prev_start_y = start_y;
	
    ROS_DEBUG_THROTTLE(0.5,"Robot localization in map frame is  are %0.2f\t%0.2f",start_x,start_y);
  }

  void plan_callback(const nav_msgs::Path& plan	) // for getting plan
  {
    ROS_DEBUG_STREAM(" Plan subscription call back entry;");
    mplan=plan.poses;
  }
  void goal_callback(const move_base_msgs::MoveBaseActionGoal& goal	) // for getting the goal
  {
    //ROS_INFO(" *******************\tGoal subscription call back entry ");
    begin = ros::Time::now();
	goal_x=goal.goal.target_pose.pose.position.x;
    goal_y=goal.goal.target_pose.pose.position.y;
    mgoal.pose.position.x=goal.goal.target_pose.pose.position.x;
    mgoal.pose.position.y=goal.goal.target_pose.pose.position.y;
	reached=false;
	
	first_localised_point = true;
  }
 void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){


    custom_scan = msg;

  }



  DetectionPlan(){

    is_grid_initialized =false;
    prev_start_x=0,prev_start_y=0;
    count=0;
    distance=0;
    check=false;
reached = false;
dist_travelled = 0.0;


    // subscribing to various topics
    costmap_sub_ = nh_.subscribe("/move_base_node/global_costmap/costmap", 10,&DetectionPlan::grid_callback,this);
    costmap_update_sub_=nh_.subscribe("/move_base_node/global_costmap/costmap_updates",10, &DetectionPlan::update_callback,this);
    laser_sub = nh_.subscribe("base_scan", 10,&DetectionPlan::laser_callback,this);
    goal_sub=nh_.subscribe("/move_base/goal", 10,&DetectionPlan::goal_callback,this);
    way_sub = nh_.subscribe("/move_base_node/TrajectoryPlannerROS/global_plan", 10,&DetectionPlan::way_callback,this);
    start_sub = nh_.subscribe("/base_pose_ground_truth",10,&DetectionPlan::localizationCallback,this);//To get start
    plan_sub = nh_.subscribe("/move_base_node/TrajectoryPlannerROS/global_plan",10,&DetectionPlan::plan_callback,this);



    // mstart and mgoal distance difference

    

  }

};



};
