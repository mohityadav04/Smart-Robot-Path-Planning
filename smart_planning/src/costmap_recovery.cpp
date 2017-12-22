
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



using costmap_2d::NO_INFORMATION;
// class for clearing the costmap once robot reaches the goal as obstacles may disappeares
namespace clear_costmap_recovery{
  /**
   * @class ClearCostmapRecovery
   * @brief A recovery behavior that reverts the navigation stack's costmaps to the static map outside of a user-specified region.
   */
  class ClearCostmapRecovery : public nav_core::RecoveryBehavior {
    public:
    
     
      /**
       * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
       * @param  
       * @return 
       */
     
      ClearCostmapRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
  		tf_(NULL), initialized_(false) {
  			
  	  } 

      /**
       * @brief  Initialization function for the ClearCostmapRecovery recovery behavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack 
       * @param local_costmap A pointer to the local_costmap used by the navigation stack 
       */
      void initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
		  if(!initialized_){
			ROS_INFO("								recovery started");
			ROS_INFO("								recovery started");		
					ROS_INFO("								recovery started");
			ROS_INFO("								recovery started");
			ROS_INFO("								recovery started");
			ROS_INFO("								recovery started");	
			name_ = name;
			tf_ = tf;
			global_costmap_ = global_costmap;
			local_costmap_ = local_costmap;

			//get some parameters from the parameter server
			ros::NodeHandle private_nh("~/" + name_);

			private_nh.param("reset_distance", reset_distance_, 8.0);
		
			std::vector<std::string> clearable_layers_default, clearable_layers;
			clearable_layers_default.push_back( std::string("obstacles") );
			private_nh.param("layer_names", clearable_layers, clearable_layers_default);

			for(unsigned i=0; i < clearable_layers.size(); i++) {
				ROS_INFO("				\tRecovery behavior will clear layer %s", clearable_layers[i].c_str());
				clearable_layers_.insert(clearable_layers[i]);
			}


			initialized_ = true;
		  }
		  else{
			ROS_ERROR("You should not call initialize twice on this object, doing nothing");
		  }
	}

	void runBehavior(){
	  if(!initialized_){
		ROS_ERROR("This object must be initialized before runBehavior is called");
		return;
	  }

	  if(global_costmap_ == NULL || local_costmap_ == NULL){
		ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
		return;
	  }
	  ROS_WARN("Clearing costmap to unstuck robot (%fm).", reset_distance_);
	  clear(global_costmap_);
	  clear(local_costmap_);
	}

	void clear(costmap_2d::Costmap2DROS* costmap){
	  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

	  tf::Stamped<tf::Pose> pose;

	  if(!costmap->getRobotPose(pose)){
		ROS_ERROR("Cannot clear map because pose cannot be retrieved");
		return;
	  }

	  double x = pose.getOrigin().x();
	  double y = pose.getOrigin().y();

	  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
		boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
		std::string name = plugin->getName();
		int slash = name.rfind('/');
		if( slash != std::string::npos ){
		    name = name.substr(slash+1);
		}

		if(clearable_layers_.count(name)!=0){
		  boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
		  costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
		  clearMap(costmap, x, y);
		}
	  }
	}


	void clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, 
		                                    double pose_x, double pose_y){
	  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
	  
	  double start_point_x = pose_x - reset_distance_ / 2;
	  double start_point_y = pose_y - reset_distance_ / 2;
	  double end_point_x = start_point_x + reset_distance_;
	  double end_point_y = start_point_y + reset_distance_;

	  int start_x, start_y, end_x, end_y;
	  costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
	  costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

	  unsigned char* grid = costmap->getCharMap();
	  for(int x=0; x<(int)costmap->getSizeInCellsX(); x++){
		bool xrange = x>start_x && x<end_x;
		               
		for(int y=0; y<(int)costmap->getSizeInCellsY(); y++){
		  if(xrange && y>start_y && y<end_y)
		    continue;
		  int index = costmap->getIndex(x,y);
		  if(grid[index]!=NO_INFORMATION){
		    grid[index] = NO_INFORMATION;
		  }
		}
	  }

	  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
	  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
	  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
	  return;
	}

    private:
      
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      std::string name_;
      tf::TransformListener* tf_;
      bool initialized_;
      double reset_distance_;
      std::set<std::string> clearable_layers_; ///< Layer names which will be cleared.
  };
};
