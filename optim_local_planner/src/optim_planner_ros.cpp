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


#include <pluginlib/class_list_macros.h>
#include <optim_local_planner/optim_planner_ros.h>
#include <base_local_planner/goal_functions.h>
#include <sys/time.h>
#include <boost/tokenizer.hpp>
#include <Eigen/Core>
#include <cmath>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>

/** TODO:
 * 1.	local_plan for visualization
 * 2. 	actions in case when optimization fails
**/

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(optim_local_planner, OptimPlannerROS, optim_local_planner::OptimPlannerROS, nav_core::BaseLocalPlanner)

namespace optim_local_planner {

  void OptimPlannerROS::initialize(std::string name, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros)
  {
    if(!initialized_){
      tf_ = tf;
      rotating_to_goal_ = false;
      costmap_ros_ = costmap_ros;
      ros::NodeHandle pn("~/" + name);

      g_plan_pub_ = pn.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = pn.advertise<nav_msgs::Path>("local_plan", 1);

      pn.param("prune_plan", prune_plan_, true);
      pn.param("exec_ratio", exec_ratio, 0.4); // time ratio execution span / planning span
      
      pn.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.15);   //angle tolerance for final pose
      pn.param("xy_goal_tolerance", xy_goal_tolerance_, 0.2);

      pn.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);

      ros::NodeHandle gn;
      obs_sub_ = gn.subscribe<nav_msgs::GridCells>("move_base/local_costmap/obstacles", 1, &OptimPlannerROS::obsCallback, this);

      pn.param("max_rot_vel", max_vel_th_, 1.5);
      min_vel_th_ = -1.0 * max_vel_th_;

      pn.param("w_reduce_ratio", w_reduce_ratio, 0.4);      // ratio of in place rotation speet regarding max_vel_th_

      // current planner pointer
      op_ = boost::shared_ptr<OptimPlanner>(new OptimPlanner(name, costmap_ros_));
      
/*********** verify loaded parameters *************/
/*	  std::cout <<"Planner has its upper velocity limit: " << op_->vmax << std::endl;
	  std::cout <<"Planner has its lower velocity limit: " << op_->vmin << std::endl;
	  std::cout <<"Planner has its upper acceleration limit: " << op_->amax << std::endl;
	  std::cout <<"Planner has its lower angular vel limit: " << op_->wmax << std::endl;
	  std::cout <<"Planner has its x-y tolerance:  " << xy_goal_tolerance_ << std::endl;
      std::cout <<"Planner has its rad_robot:  " << op_->rad_robot << std::endl;
      std::cout <<"Planner has its max_rot_vel:  " << max_vel_th_ << std::endl;
      std::cout <<"Planner has its exec_ratio:  " << exec_ratio << std::endl;
      std::cout <<"Planner has its window length:  " << op_->Nhor << std::endl;
      std::cout <<"Planner has its w_reduce_ratio:  " << w_reduce_ratio << std::endl; */
/************************/

      global_frame_ = costmap_ros_->getGlobalFrameID();
      initialized_ = true;
      local_traj_found = false; 
      runLocalPlanner_ = false;
      isPlanning = false;
      needReOrient = false;
      optim_failed = false;
      goalAchieved = false;
      rotating_to_goal_ = false;
      last_final_cmd_vel.linear.x = 0.001;
	  last_final_cmd_vel.linear.y = 0.0;
	  last_final_cmd_vel.angular.z = 0.0;
      local_planner_thread_ = new boost::thread(boost::bind(&OptimPlannerROS::planThread, this));
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }

  void OptimPlannerROS::obsCallback(const nav_msgs::GridCells obs_msg)
  {
      obstacles_ = obs_msg;
      // std::cout << "There are obstacles around, number is" << obstacles_.cells.size() << std::endl;
  }


  bool OptimPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    
    // std::vector<geometry_msgs::PoseStamped> local_plan;
    tf::Stamped<tf::Pose> global_pose;
    
    if(!costmap_ros_->getRobotPose(global_pose))
    return false;

    costmap_2d::Costmap2D costmap;
    costmap_ros_->getCostmapCopy(costmap);
    // std::vector<geometry_msgs::PoseStamped> transformed_plan;

  if (!base_local_planner::transformGlobalPlan(*tf_, global_plan_, global_pose, costmap, global_frame_, transformed_plan)) {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }
    
        //now we'll prune the plan based on the position of the robot

    costmap_ros_->clearRobotFootprint();
    
   if(transformed_plan.empty())
      return false;
    
    double goal_x, goal_y;

    tf::Stamped<tf::Pose> goal_point;

    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    goal_x = goal_point.getOrigin().getX();
    goal_y = goal_point.getOrigin().getY();
 
    double current_theta = tf::getYaw(global_pose.getRotation());
    
    if (!isPlanning)
    {
         base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);
         // OptimPlannerROS::pruneGPlan(global_pose, transformed_plan, global_plan_);
	}
	

if (base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_ || rotating_to_goal_)
    {
		  rotating_to_goal_ = true;
		  double ggoal_theta;// current_theta;
		  tf::Stamped<tf::Pose> gGoalPose;
		  tf::poseStampedMsgToTF(global_plan_.back(), gGoalPose);
		  ggoal_theta = tf::getYaw(gGoalPose.getRotation());
		  double horizon = op_->mystep * op_->Nhor ;

          current_theta = tf::getYaw(global_pose.getRotation());
		  double diff;

		  diff = angles::shortest_angular_distance(current_theta, ggoal_theta);

		 if ( std::fabs(diff) < yaw_goal_tolerance_) 
		 {
			ROS_INFO("================Goal achieved!!!!!!!!!!! ==========");
			goalAchieved = true;
			rotating_to_goal_ = false;
            last_final_cmd_vel.linear.x = 0.001;
		 }

		  boost::unique_lock<boost::mutex> lock(local_planner_mutex_);
		  runLocalPlanner_ = false;
		  cmd_vel.linear.x = 0.0;
		  cmd_vel.linear.y = 0.0;
	      cmd_vel.angular.z = diff/(exec_ratio*horizon);
          if ( cmd_vel.angular.z  > w_reduce_ratio*max_vel_th_)
	      {
              cmd_vel.angular.z  = w_reduce_ratio*max_vel_th_;
	      }
          if ( cmd_vel.angular.z  < -1.0*w_reduce_ratio* max_vel_th_)
	      {
              cmd_vel.angular.z  = -1.0* w_reduce_ratio*max_vel_th_;
	      }

	      //publish information to the visualizer
		  // ROS_INFO("Rotating!!!!!");

		  base_local_planner::publishPlan(transformed_plan, g_plan_pub_); // , 0.0, 1.0, 1.0, 0.0);
		  base_local_planner::publishPlan(local_plan, l_plan_pub_);//, 1.0, 0.0, 1.0, 0.0);
		  lock.unlock();
		  return true;
      }

    // if we need to carry out (re-carry out) the optimization
	if ((!local_traj_found) && (!optim_failed))
    {
		//Todo: carry out optimization here
		boost::unique_lock<boost::mutex> lock(local_planner_mutex_);
		if(!isPlanning)
		{
		  runLocalPlanner_ = true;
		  local_planner_cond_.notify_one();
		  // ROS_INFO("We need to generate the local trajectory first!!");
		}
		lock.unlock();
    }

    geometry_msgs::Twist global_vel;
    
    double current_instant = ros::Time::now().toSec();
    double t = (current_instant - initial_instant);
    double horizon = op_->mystep * op_->Nhor ;
    
    if (t > exec_ratio*horizon)
    {
		// we need to redo the planning for next horizon
		boost::unique_lock<boost::mutex> lock(local_planner_mutex_);
		local_traj_found = false;
		// if (!optim_failed)
            {runLocalPlanner_ = true;}
		// needReOrient = false;
		optim_failed = false;
		lock.unlock();
        if ((t - exec_ratio*horizon) > 0.5)
		{
            cmd_vel.linear.x = 0.;
		    cmd_vel.linear.y = 0.;
		    cmd_vel.angular.z = 0.;
			// ROS_WARN("waiting too long time for optimization to finish.");		
		}
        else
        {
            cmd_vel.linear.x = last_final_cmd_vel.linear.x;
            cmd_vel.linear.y = 0.;
            cmd_vel.angular.z = last_final_cmd_vel.angular.z;
        }
		return true;
	}
	      
	if (needReOrient)
	{
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.angular.z = op_->refAngVel;
		//publish information to the visualizer
        last_final_cmd_vel.angular.z =  op_->refAngVel;
		base_local_planner::publishPlan(transformed_plan, g_plan_pub_);//, 0.0, 1.0, 1.0, 0.0);
		base_local_planner::publishPlan(local_plan, l_plan_pub_);//, 1.0, 0.0, 1.0, 0.0);
		return true;
		
	}
	else
	{
		double coef_x[4] = {op_->param_traj[0], op_->param_traj[1], op_->param_traj[2], op_->param_traj[3]};
		double coef_y[4] = {op_->param_traj[4], op_->param_traj[5], op_->param_traj[6], op_->param_traj[7]};
		double vx, vy, wpub, ax, ay;
		vx = coef_x[1] + 2*coef_x[2]*t + 3*coef_x[3]*t*t;
		vy = coef_y[1] + 2*coef_y[2]*t + 3*coef_y[3]*t*t;
		ax = 2 * coef_x[2] + 6 * coef_x[3] * t; 		// computing ax at this instant
		ay = 2 * coef_y[2] + 6 * coef_y[3] * t; 		// computing ay at this instant

		if ((vx != 0) || (vy != 0) )
		{
			wpub = (vx * ay - vy * ax) / (vx * vx + vy * vy);
 			// wpub = (-vx * sin(current_theta) + vy * cos(current_theta)) / 0.4;
		}
		else
		{
			wpub = 0;
		}

		double v_lin = std::sqrt(vx*vx + vy*vy);

		cmd_vel.linear.x = v_lin;
		cmd_vel.linear.y = 0;
		cmd_vel.angular.z = wpub;
		// std::cout << "Output velocity: " << v_lin << std::endl;
		
		if (optim_failed)
		{
            ROS_WARN("Optimization fails, set null velocity");
            cmd_vel.linear.x = -0.050;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0.0*wpub;
            last_final_cmd_vel.linear.x = -0.050;
            last_final_cmd_vel.angular.z = 0.0;
            runLocalPlanner_ = false;
            return true;
		}

		// stock the final valid velocity command which will be published during the optimization period 
        // std::cout << "time margin is : " << (t - exec_ratio*horizon) << std::endl;
		double vx_end = coef_x[1] + 2*coef_x[2]* exec_ratio*horizon + 3*coef_x[3]*  exec_ratio*horizon * exec_ratio*horizon;
		double vy_end = coef_y[1] + 2*coef_y[2]* exec_ratio*horizon + 3*coef_y[3]* exec_ratio*horizon* exec_ratio*horizon;
		double ax_end = 2 * coef_x[2] + 6 * coef_x[3] *  exec_ratio*horizon; 		// computing ax at this instant
		double ay_end = 2 * coef_y[2] + 6 * coef_y[3] *  exec_ratio*horizon; 		// computing ay at this instant
        double wpub_end;
		if ((vx_end != 0) || (vy_end != 0) )
		{
			wpub_end = (vx_end * ay_end - vy_end * ax_end) / (vx_end * vx_end + vy_end * vy_end);
 			// wpub = (-vx * sin(current_theta) + vy * cos(current_theta)) / 0.4;
		}
		else
		{
			wpub_end = 0;
		}
        
		last_final_cmd_vel.linear.x = std::sqrt(vx_end*vx_end + vy_end*vy_end);;
        // std::cout << "last cmd is : " << last_final_cmd_vel.linear.x << std::endl;
		last_final_cmd_vel.linear.y = 0;
		last_final_cmd_vel.angular.z = wpub_end;         
		
	}
		
		// std::cout << "current relative time: " << t << std::endl;
		base_local_planner::publishPlan(transformed_plan, g_plan_pub_);//, 0.0, 1.0, 1.0, 0.0);
		base_local_planner::publishPlan(transformed_plan, l_plan_pub_);//, 1.0, 0.0, 1.0, 0.0);
		
		if (optim_failed)
		{   
            ROS_INFO("***Should not see this message***");
            return false;
		}
		else
		{
            return true;
		}

  }

  bool OptimPlannerROS::isGoalReached()
  {
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    nav_msgs::Odometry base_odom;
    geometry_msgs::PoseWithCovariance base_pose;

    bool res = goalAchieved;
    goalAchieved = false;
    return res;
  }

  bool OptimPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    global_plan_.clear();
    global_plan_ = orig_global_plan;

    return true;
  }

double OptimPlannerROS::getOrientationDiff(geometry_msgs::Pose2D currentPose, geometry_msgs::Pose2D goalPose)
{
	double goalDirection, currentDirection;

	goalDirection = atan2((goalPose.y - currentPose.y),(goalPose.x - currentPose.x));
	currentDirection = currentPose.theta;
	
	double diff;
	diff = angles::shortest_angular_distance(0, goalDirection - currentDirection);

	return diff;
}

void OptimPlannerROS::planThread(){

    // ROS_INFO("optim_local_plan_thread -- Starting planner thread...");
    ros::NodeHandle n;
    
    while(n.ok()){

      boost::unique_lock<boost::mutex> lock(local_planner_mutex_);

      while(!runLocalPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("optim_local_plan_thread","Planner thread is suspending");
        local_planner_cond_.wait(lock); // here, the wait() function automatically unlock the mutex and suspends the thread; when another thread releases the mutex, it should also notify a signal to local_planner_cond_ so that this thread can get mutex and run once
       }

      lock.unlock();
     // ROS_INFO("optim_local_plan_thread -- Planning...");
      // ROS_INFO("**********************optim_local_plan_thread starts********************");
      gotLocalPlan = false;
      isPlanning = true;
      
      local_traj_found = false;
       tf::Stamped<tf::Pose> global_pose;
    
    if(!costmap_ros_->getRobotPose(global_pose))
        ROS_WARN("Getting current robot position failed!!!");

      //tf::Stamped<tf::Pose> global_pose;
      geometry_msgs::Pose2D myStart, myGoal;
	  // getRobotPose(global_pose);

      myStart.x = global_pose.getOrigin().x();
      myStart.y = global_pose.getOrigin().y();
      myStart.theta=tf::getYaw(global_pose.getRotation());


	  bool getGoal = true;
		  
      if (!transformed_plan.empty())
      {
		  tf::Stamped<tf::Pose> GoalPose;

          std::vector<geometry_msgs::PoseStamped>::iterator it = transformed_plan.end();
          while(1)
          {
              tf::poseStampedMsgToTF(*(it-1), GoalPose);
              //we assume the global goal is the last point in the global plan
              //double goal_x = GoalPose.getOrigin().getX();
              //double goal_y = GoalPose.getOrigin().getY();
              double x_diff = global_pose.getOrigin().x() - GoalPose.getOrigin().getX();
              double y_diff = global_pose.getOrigin().y() - GoalPose.getOrigin().getY();
              double distance_sq = x_diff * x_diff + y_diff * y_diff;
              if (distance_sq > 0.64) // we get local goal that is within 0.8m distance from robot
                  it--;
              else
                  break;
          }

          // tf::poseStampedMsgToTF(transformed_plan.back(), GoalPose);
		  myGoal.x = GoalPose.getOrigin().getX();
		  myGoal.y = GoalPose.getOrigin().getY();
		  myGoal.theta = tf::getYaw(GoalPose.getRotation());
	  }
	  else
	  {
		 
		  myGoal.x = myStart.x;
		  myGoal.y = myStart.y;
		  myGoal.theta = myStart.theta;
		  getGoal = false;
	  }

	  // if we need to change robot orientation firstly
      if (std::fabs(getOrientationDiff(myStart, myGoal)) > 1.5 || (!getGoal))
	  {
		  // ROS_INFO("The robot needs to change its orientation!!! ");
		  double horizon = op_->mystep * op_->Nhor ;
		  double diff = getOrientationDiff(myStart, myGoal);
          if (diff/horizon > w_reduce_ratio*max_vel_th_ ) {op_->refAngVel = w_reduce_ratio*max_vel_th_;}
          else if (diff/horizon < -1.0*w_reduce_ratio*max_vel_th_) {op_->refAngVel = -1.0*w_reduce_ratio*max_vel_th_;}
		  else {op_->refAngVel = (1.0/exec_ratio)*diff/horizon ;}
		  lock.lock();
		  initial_instant = ros::Time::now().toSec();
		  local_traj_found = true;
		  needReOrient = true;
		  runLocalPlanner_ = false;
		  isPlanning = false;
		  lock.unlock();
	  }
	  else    // else: not need to change orientation, so start the optimization
	  {
		  // ROS_INFO("optim_local_plan_thread -- Set goal position!");
		  needReOrient = false;
  
		  // std::cout << "Goal: " << myGoal.x << ", " << myGoal.y << ", " << myGoal.theta << std::endl;
		  op_->setStart(myStart);
		  op_->setGoal(myGoal);
		  op_->setObs(obstacles_);
          num_obs = obstacles_.cells.size();
          op_->setCurrentVel(std::max(last_final_cmd_vel.linear.x, 0.001), last_final_cmd_vel.angular.z);

		  op_->calcTraj();
		  if(n.ok() && op_->success_flag)
		  {
			  lock.lock();
			  local_traj_found = true;
			  optim_failed = false;
			  runLocalPlanner_ = false;
			  initial_instant = ros::Time::now().toSec();
			  // ROS_INFO("A local trajectory is found!!!");
			  isPlanning = false;
			  lock.unlock();
		  }
		  else                       
		  {
			  lock.lock();
			  local_traj_found = true;
			  runLocalPlanner_ = false;
			  initial_instant = ros::Time::now().toSec();
			  // ROS_INFO("A local trajectory is not found!!!");
			  isPlanning = false;
              optim_failed = true;
			  lock.unlock();
			  ROS_ERROR("A local trajectory could not be found!!!");
			  
		  }
		} // else: not need to change orientation, so start the optimization

    } //while (n.ok())
  }

};
