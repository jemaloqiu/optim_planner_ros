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


#ifndef Optim_LOCAL_PLANNER_Optim_PLANNER_H_
#define Optim_LOCAL_PLANNER_Optim_PLANNER_H_
#include <queue>
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/map_grid.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
extern "C"
{
	#include <stdlib.h>
	#include "optim_local_planner/cfsqpusr.h"
}

#include <dynamic_reconfigure/server.h>
#include <optim_local_planner/OptimPlannerConfig.h>
#include <base_local_planner/map_grid_visualizer.h>

namespace optim_local_planner {
  /**
   * @class OptimPlanner
   * @brief A class implementing a local planner using the optimization approach
   */
  class OptimPlanner {
    public:

      OptimPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
       * @brief  Destructs the optim local planner
       */

      ~OptimPlanner() {delete world_model_;}

    /**
       * @brief  Update the global plan that the local planner is following
       * @param new_plan A new global plan obtained from global planner 
       */

      void updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan);

    /**
       * @brief Reconfigures the local planner
       */

      void reconfigureCB(OptimPlannerConfig &config, uint32_t level);

      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D costmap_;
      base_local_planner::CostmapModel* world_model_;
      dynamic_reconfigure::Server<OptimPlannerConfig> dsrv_;
      optim_local_planner::OptimPlannerConfig default_config_;
     
      double sim_period_;
      nav_msgs::GridCells localObs; ///< @brief The obstacles obstained from local costmap
      int nb_obstacle;  ///< @brief Number of obstacles obstained from local costmap
      int Nhor; ///< @brief Number of time steps in a planning horizon
      double mystep;    ///< @brief Time step
      double vmax, vmin, wmax, wmin, amax, amin;    ///< @brief Velocity and acceleration limits for the local planner
      geometry_msgs::Pose2D start, goal;    ///< @brief Starting and goal position for the local planner
      bool setup_;
	  double refAngVel; ///< @brief Reference pure-roation angular velocity of the local planner
      bool success_flag;    ///< @brief Boolean variable indicating the result of optimization
      std::vector<double> param_traj;   ///< @brief Parameters of polynomial trajectory obtained from optimization
	  double planif[9]; ///< @brief A table containing the polynomial to optimize
	  double current_v, current_w;  ///< @brief Initial velocities of robot for path planning
	  double rad_robot; ///< @brief Secure distance for a robot to avoid collisions with obstacles
	  static OptimPlanner* CurrentOptimPlanner; ///< @brief A poiter pointing to the planner itself
      
      std::vector<geometry_msgs::PoseStamped> global_plan_; ///< @brief Globlan plan to follow

      boost::mutex configuration_mutex_;    ///< @brief Mutex for configuration thread

    /**
       * @brief  Set a new start pose for the planner
       * @param newStart A new start pose for next planning horizon 
       */

      void setStart(geometry_msgs::Pose2D newStart);

    /**
       * @brief  Set a new goal pose for the planner
       * @param newGoal A new goal pose for next planning horizon 
       */
      void setGoal(geometry_msgs::Pose2D newGoal);

    /**
       * @brief  Update obstacles for the planner
       * @param newObs New obstacles for next planning horizon 
       */
      void setObs(nav_msgs::GridCells newObs);

    /**
       * @brief  Update initial velocities for the planner
       * @param v_lin Current linear velocity of robot
       * @param v_ang Current angular velocity of robot
       */
      void setCurrentVel(double v_lin, double v_ang);

    /**
       * @brief  objective function for optimizaiton solver
       */
      void objectif(int , int , double *, double *, void *);

    /**
       * @brief  constraint function for optimizaiton solver
       */
	  void cntr1(int, int , double *, double *, double *);

    /**
       * @brief  Calculate robot trajectory
       */
	  void calcTraj();


	  static void com_objectif(int, int, double *, double *, void *);

	  static void com_cntr1(int, int, double *, double *, double *);  

   /**
       * @brief  Solve the optimizaiton problem
       */
	  int optimization(double[4], double[3], double[3], double[8], int, int); 

  };
};
#endif
