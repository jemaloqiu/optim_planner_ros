/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Non-A Team, INRIA Lille / EC-LILLE, France.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: QIU
*********************************************************************/


#ifndef Optim_LOCAL_PLANNER_Optim_PLANNER_ROS_H_
#define Optim_LOCAL_PLANNER_Optim_PLANNER_ROS_H_
#include <angles/angles.h>
#include <optim_local_planner/optim_planner.h>
#include <boost/shared_ptr.hpp>
#include <nav_core/base_local_planner.h>
#include <boost/thread/mutex.hpp>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/point_grid.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/voxel_grid_model.h>
#include <base_local_planner/trajectory_planner.h>
#include <base_local_planner/map_grid_visualizer.h>

#include <base_local_planner/planar_laser_scan.h>

#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_listener.h>

#include <boost/thread.hpp>

#include <string>

#include <angles/angles.h>

#include <nav_core/base_local_planner.h>
#include <dynamic_reconfigure/server.h>
#include <base_local_planner/BaseLocalPlannerConfig.h>
#include <base_local_planner/odometry_helper_ros.h>

namespace optim_local_planner {

  class OptimPlannerROS : public nav_core::BaseLocalPlanner {
    public:
    
     /**
       * @brief  Construct an instance of the optim_local_planner class
       */
      OptimPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

     /**
       * @brief  Initialize parameters of the planner wrapper
       */

      void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

    /**
	   * @brief Get the angle difference between current orientation and a desired direction
	   * Objective of this function: make sure that the robot has its goal in front of it 
	   * since it cannot move backward
	   * */
	  double getOrientationDiff(geometry_msgs::Pose2D currentPose, geometry_msgs::Pose2D goalPose);
      
    /**
       * @brief  Verify if the goal has been reached
       */
      bool isGoalReached();

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
       * @brief  Start a new thread for path planning using optimization
       */
      void planThread();

    /**
       * @brief  Compute velocity command based on the parameters obtained from optimization
       * @param cmd_vel The variable for stocking/publishing the velocity command
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
       * @brief  Subscriber callback function for updating obstacle information
       * @param obs_msg The local_costmap/obstacles topic 
       */
	  void obsCallback(const nav_msgs::GridCells obs_msg);
	  
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D costmap_;
      tf::TransformListener* tf_;
      
      double max_vel_th_, min_vel_th_; ///< @brief Angular velocity limits of the local planner, here they are only used for calculating the in-place rotation velocity limits: w_reduce_ratio*max_vel_th_
      double yaw_goal_tolerance_, xy_goal_tolerance_; ///< @brief Criteria for determining if the robot reaches its goal pose 
      geometry_msgs::Twist last_final_cmd_vel; ///< @brief Final velocity command of last execution horizon 
      double exec_ratio;    ///< @brief Ratio of motion execution window regarding the planning window length
      double w_reduce_ratio;   ///< @brief Ratio used for calculating the in-place rotation velocity limits: w_reduce_ratio*max_vel_th_
      
      bool prune_plan_;
      bool initialized_;
      bool local_traj_found;
      bool gotLocalPlan;
      bool runLocalPlanner_;
      bool isPlanning;
      bool optim_failed;
      
      ros::Subscriber obs_sub_;
      ros::Publisher g_plan_pub_, l_plan_pub_;
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      tf::Stamped<tf::Pose> global_pose;
    
	  std::string global_frame_;

      nav_msgs::Odometry base_odom_;
      geometry_msgs::PoseWithCovariance base_pose_;
      nav_msgs::GridCells obstacles_;
      boost::shared_ptr<OptimPlanner> op_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      
      bool rotating_to_goal_;
      bool latch_xy_goal_tolerance_, xy_tolerance_latch_;
      bool goalAchieved;
      double initial_instant;
      int num_obs;
	  bool needReOrient;
	  boost::mutex local_planner_mutex_;
	  boost::condition_variable local_planner_cond_;
	  boost::thread* local_planner_thread_;
	  
  };
};
#endif
