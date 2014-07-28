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


#include <optim_local_planner/optim_planner.h>
#include <angles/angles.h>

namespace optim_local_planner {
	
  void OptimPlanner::reconfigureCB(OptimPlannerConfig &config, uint32_t level)
  {
    if(setup_ && config.restore_defaults) {
      config = default_config_;
      config.restore_defaults = false;
    }

    if(!setup_) {
      default_config_ = config;
      setup_ = true;
    }
    boost::mutex::scoped_lock l(configuration_mutex_);
 
    /* Dynamic reconfigure enabled, but I have not yet set parameters 
        */
  } 

  OptimPlanner::OptimPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : 
  costmap_ros_(NULL), 
  world_model_(NULL), 
  dsrv_(ros::NodeHandle("~/" + name)), 
  setup_(false)
  {
    costmap_ros_ = costmap_ros;
    costmap_ros_->getCostmapCopy(costmap_);

    ros::NodeHandle pn("~/" + name);

    pn.param("vmax", vmax, 0.45);
    pn.param("vmin", vmin, 0.0010);
    pn.param("wmax", wmax, 1.5);
    pn.param("wmin", wmin, -1.5);   // not used for the moment

    pn.param("acc_max", amax, 1.5);
    pn.param("acc_min", amin, -1.5); // not used for the moment

    pn.param("mystep", mystep, 0.25);
    pn.param("Nhor", Nhor, 5);
    pn.param("rad_robot", rad_robot, 0.225);
	
    std::string controller_frequency_param_name;
    
    if(!pn.searchParam("controller_frequency", controller_frequency_param_name))
      sim_period_ = 0.05;
    else
    {
      double controller_frequency = 0;
      pn.param(controller_frequency_param_name, controller_frequency, 20.0);
      if(controller_frequency > 0)
        sim_period_ = 1.0 / controller_frequency;
      else
      {
        ROS_WARN("A controller_frequency is set negative. Reset it as 20Hz");
        sim_period_ = 0.05;
      }
    }
    ROS_INFO("Sim period is set to %.2f", sim_period_);


    dynamic_reconfigure::Server<OptimPlannerConfig>::CallbackType cb = boost::bind(&OptimPlanner::reconfigureCB, this, _1, _2);
    dsrv_.setCallback(cb);

	refAngVel = 0.0;
	success_flag = true;

	nb_obstacle = 0;
	current_v = 0.001;
	current_w = 0.0;
	
  }

OptimPlanner* OptimPlanner::CurrentOptimPlanner;


  void OptimPlanner::updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan)
	{
		global_plan_.resize(new_plan.size());
		for(unsigned int i = 0; i < new_plan.size(); ++i)
		{
		  global_plan_[i] = new_plan[i];
		}
	}
	
	
  void OptimPlanner::setStart(geometry_msgs::Pose2D newStart)
	{
		start = newStart;
	}

  void OptimPlanner::setGoal(geometry_msgs::Pose2D newGoal)
	{
		goal = newGoal;
	}

  void OptimPlanner::setCurrentVel(double v_lin, double v_ang)
	{
		current_v = v_lin;
		current_w = v_ang;
	}
	
  void OptimPlanner::setObs(nav_msgs::GridCells newObs)
  {
	  this->localObs = newObs;
	  nb_obstacle = localObs.cells.size();
      // ROS_INFO("Optim planner has received %d obstacles", nb_obstacle);
	  }
  
  void OptimPlanner::objectif(int nparam, int j, double *x, double *fj, void *cdv) 
	{

        
		double* cd = (double*) cdv;
		
		// receive the user defined data transfered by cd
		double x0 = cd[0];
		double y0 = cd[1];
		//double dx0 = cd[3] * cos(cd[2]); //==init.v * cos(theta)
		//double dy0 = cd[3] * sin(cd[2]); //==init.v * sin(theta)
		double dx0 = cd[3] * cos(cd[2]); // - cd[8] * 0.4*sin(cd[2]); //==init.v * cos(theta)
		double dy0 = cd[3] * sin(cd[2]); //+ cd[8] * 0.4*cos(cd[2]); //==init.v * sin(theta)
				
		double c0, c1, c2, c3, d0, d1, d2, d3;

		// Trajectory in x direction
		c0 = x0; 	// initial position
		c1 = dx0; 	// initiale velocity
		c2 = x[1]; 	// variable to optimize
		c3 = x[0]; 	// variable to optimize

		// Trajectory in y direction
		d0 = y0; 	// initial position
		d1 = dy0; 	// initiale velocity
		d2 = x[3]; 	// variable to optimize
		d3 = x[2]; 	// variable to optimize

		double inter1, inter2;

		double tmax = mystep * Nhor;
		double resultat = 0;

		if (j == 1) {
			// double vx, vy;
			inter1 = c3 * tmax * tmax * tmax + c2 * tmax * tmax + c1 * tmax + c0;
			inter2 = d3 * tmax * tmax * tmax + d2 * tmax * tmax + d1 * tmax + d0;
			
			/* vx = 3 * c3 * tmax * tmax  + 2 * c2 * tmax + c1;
			vy = 3 * d3 * tmax * tmax  + 2 * d2 * tmax + d1; */
			
			resultat += 10000 * ( (cd[4] - inter1) * (cd[4] - inter1) + (cd[5] - inter2) * (cd[5] - inter2));// + vx*vx + vy*vy;// + (vx * ay - vy * ax)*(vx * ay - vy * ax);
			*fj = resultat;
		}
		return;	
	}

	void OptimPlanner::cntr1(int nparam, int j, double *x, double *gj, double *cd) 
	{
        // std::cout << "Add constraints" << std::endl;
		double step = mystep;
		double x0 = cd[0];
		double y0 = cd[1];
		double dx0 = cd[3] * cos(cd[2]); // - cd[8] * 0.4*sin(cd[2]); //==init.v * cos(theta)
		double dy0 = cd[3] * sin(cd[2]); // + cd[8] * 0.4*cos(cd[2]); //==init.v * sin(theta)
		
		double c0, c1, c2, c3, d0, d1, d2, d3;

		int Nfinal = Nhor;   // sampling points number

		// Trajectory in x direction
		c0 = x0; 	// initial position
		c1 = dx0; 	// initiale velocity
		c2 = x[1]; 	// variable to optimize
		c3 = x[0]; 	// variable to optimize

		// Trajectory in y direction
		d0 = y0; 	// initial position
		d1 = dy0; 	// initiale velocity
		d2 = x[3]; 	// variable to optimize
		d3 = x[2]; 	// variable to optimize

		static int dec = 1; // number of non-linear constraints

		// Variables definition
		int indice_temps, indice_obstacle;

		double t, xr, yr, vx, vy, ax, ay;
		double vitesse_ang_max = 0;

		// constraints
		if (j == 1) 
		{
			double tmax = Nhor * mystep;
			vx = 3 * c3 * tmax * tmax + 2 * c2 * tmax + c1;
			vy = 3 * d3 * tmax * tmax + 2 * d2 * tmax + d1;
			*gj = sqrt(vx * vx + vy * vy) - vmax;
		}

		// constraints on linear velocity at intermediate instants (num = Nfinal )
		if (j >= dec + 1 && j <= dec + Nfinal)	
		{
			indice_temps = (j - 1) % Nfinal;
			t = step * indice_temps; 		// time of this instant
			vx = c1 + 2 * c2 * t + 3 * c3 * t * t; 	// computing vx at this instant
			vy = d1 + 2 * d2 * t + 3 * d3 * t * t; 	// computing vx at this instant
			*gj = vy * vy + vx * vx - vmax*vmax; 	// cannot exceed the velocity limit
		}

		// constraints on linear acceleration at intermediate instants (num = Nfinal )
		if (j >= dec + Nfinal + 1 && j <= dec + 2 * Nfinal)
		{
			indice_temps = (j - 1) % Nfinal;
			t = step * indice_temps; 	 	// time of this instant
			ax = 2 * c2 + 6 * c3 * t; 		// computing ax at this instant
			ay = 2 * d2 + 6 * d3 * t; 		//	computing ax at this instant
			*gj = ay * ay + ax * ax - amax*amax; 	// cannot exceed the acceleration limit
		}

		// Inequality constraints on angular velocity at intermediate instants (num = Nfinal )
		if (j >= dec + 2 * Nfinal + 1 && j <= dec + 3 * Nfinal)
		{
			indice_temps = (j - 1) % Nfinal;
			t = step * indice_temps; 		// time of this instant
			vx = c1 + 2 * c2 * t + 3 * c3 * t * t; 	// computing vx at this instant
			vy = d1 + 2 * d2 * t + 3 * d3 * t * t; 	// computing vy at this instant
			ax = 2 * c2 + 6 * c3 * t; 		// computing ax at this instant
			ay = 2 * d2 + 6 * d3 * t; 		// computing ay at this instant
			if (vx != 0 || vy != 0) 		// for avoiding singularities
			{
				vitesse_ang_max = (vx * ay - vy * ax) / (vx * vx + vy * vy); //compute the angular velocity
				// vitesse_ang_max =( vx * sin(cd[2]) - vy * cos(cd[2]) )/0.4;
			} 
			else
			{
				vitesse_ang_max = 0;
			}

			*gj = vitesse_ang_max - wmax; 		// cannot exceed the angular velocity limit
		}

		// Inequality constraints on angular velocity at intermediate instants (num = Nfinal )
		if (j >= dec + 3 * Nfinal + 1 && j <= dec + 4 * Nfinal) 
		{ 
			indice_temps = (j - 1) % Nfinal;
			t = step * indice_temps; 		// time of this instant
			vx = c1 + 2 * c2 * t + 3 * c3 * t * t;  // computing vx at this instant
			vy = d1 + 2 * d2 * t + 3 * d3 * t * t; 	// computing vy at this instant
			ax = 2 * c2 + 6 * c3 * t; 		// computing ax at this instant
			ay = 2 * d2 + 6 * d3 * t; 		// computing ay at this instant
			if (vx != 0 || vy != 0) 		// for avoiding singularities
			{
				vitesse_ang_max = (vy * ax - vx * ay) / (vx * vx + vy * vy);
				// vitesse_ang_max =(-vx * sin(cd[2]) + vy * cos(cd[2]) )/0.4; //compute the angular velocity
			} 
			else 
			{
				vitesse_ang_max = 0;
			}

			*gj = vitesse_ang_max - wmax; // vitesse_ang_max - WMAX < 0 <==> vitesse_ang_max  < WMAX <==> contrainte satisfaite
		}

	  if (nb_obstacle > 0) 
	  { //std::cout << "Add constraint for obstacl avoiding " << std::endl;
		if (j >= dec + (4) * Nfinal + 1	&& j <= dec + (4 + nb_obstacle) * Nfinal)
		{
			indice_temps = (j - dec) % Nfinal;
			t = (double) step * indice_temps; //generation du temps
			xr = c0 + c1 * t + c2 * t * t + c3 * t * t * t; // generation des x
			yr = d0 + d1 * t + d2 * t * t + d3 * t * t * t; //	generation des y
			indice_obstacle = (((j - dec) / Nfinal) - 4) % nb_obstacle;

			// int counter = 0;
			// std::pair<double, double> obs;
			double obs[2];
			obs[0] = localObs.cells[indice_obstacle].x;
			obs[1] = localObs.cells[indice_obstacle].y;

			*gj =  rad_robot*rad_robot - (xr - obs[0])* (xr - obs[0]) -  (yr - obs[1])*(yr - obs[1]);
			// std::cout << "Add constraint  " <<  j << std::endl;

		}
	}
		
		
	}  // cntr1

	void OptimPlanner::com_objectif(int int1, int int2, double *d1, double *d2, void *v) 
	{
		OptimPlanner::CurrentOptimPlanner->objectif(int1, int2, d1, d2, v);
	}


	void OptimPlanner::com_cntr1(int int1, int int2, double *d1, double *d2, double *d3) 
	{
		OptimPlanner::CurrentOptimPlanner->cntr1(int1, int2, d1, d2, d3);
	}
	 
	int OptimPlanner::optimization(double init[5], double fin[3], double fin2[3],
			double previous_polynome[8], int coeff1, int coeff2) 
	{
		int nparam, nf, nineq, neq, mode, iprint, miter, neqn, nineqn, ncsrl, ncsrn,
			nfsr, inform;
		double bigbnd, eps, epsneq, udelta;
		double *x, *bl, *bu, *f, *g, *lambda;
		double *cd;
		// nb_obstacle = 0;
		double c0, c1, c2, c3, d0, d1, d2, d3;

		//=================================================================================
		// Definition of global variables for the optimization problem
		//=================================================================================

		// Mode of the optimization problem (CBA)
		mode = 110; 		// see the commentary in the source file cfsqp.c

		// Option for printing informations
		iprint = 0; 		// No information printed to screen

		// Maximum number of iterations
		miter = 4000;

		// For defining lower and upper bounds for the solution
		bigbnd = 1.e10;

		// Criterion for stopping the computing loops
		eps = 1.e-3;

		// Maximum tolerance for constraints
		epsneq = 1e-3;

		// For getting out from a local minimum value
		udelta = 1.e-7; 	

		// Dimension of the variables to optimize 
		// In this planner, the coefficients of 2nd and 3rd order terms are chosen
		// as variables so there are 4 unknown variables (2 for x, 2 for y)
		nparam = 4; 

		// Lower bound for variables
		bl = (double *) calloc(nparam, sizeof(double));

		// Upper bound for variables
		bu = (double *) calloc(nparam, sizeof(double));

		// (INPUT) Initial conditions and (OUTPUT) Iterate at the end of execution
		x = (double *) calloc(nparam, sizeof(double));

		// User defined data for transferring additional informations to the solver
		cd = (double *) calloc(12, sizeof(double));


		cd[0] = init[0]; 	// initial x
		cd[1] = init[1]; 	// initial y
		cd[2] = init[2]; 	// initial theta
		cd[3] = init[3]; 	// initial v
		cd[4] = fin[0]; 	// final x
		cd[5] = fin[1]; 	// final y
		cd[6] = fin[2]; 	// final theta
		cd[7] = fin[3];     	// final v
		//cd[8] = fin2[0];   // initial w
		cd[8] = init[4];
		cd[9] = fin2[1];
		cd[10] = coeff1;		
		cd[11] = coeff2;
		

		bl[0] = bl[1] = -bigbnd;
		bu[0] = bu[1] = bigbnd;
		bl[2] = bl[3] = -bigbnd;
		bu[2] = bu[3] = bigbnd;

		x[0] = previous_polynome[3];
		x[1] = previous_polynome[2];
		x[2] = previous_polynome[7];
		x[3] = previous_polynome[6];

		// Number of objective function 
		nf = 1;

		// Number of sequential objective function sets
		nfsr = 0;

		// Number of inequality non-linear constraints
		nineqn = (5 + nb_obstacle);

		// Total number of inequality constraints (i.e. ineq NL + ineq L)
		nineq = (5 + nb_obstacle);

		// Number of non-linear constraints
		neqn = 0;

		// Total number of equality constraints (i.e. eq NL + eq L)
		neq = 0;

		// Number of sequential non-linear constraint sets
		ncsrn = 4 + nb_obstacle; 

		// Number of sequential linear constraint sets
		ncsrl = 0;

		
		int size_mesh = std::max(1, nfsr + ncsrn + ncsrl);
		int *mesh_pts = (int*) calloc(size_mesh, sizeof(int));
		int i = 0;
		for (i = 0; i < nfsr; i++) 
		{
			mesh_pts[i] = Nhor;
		}
		for (i = nfsr; i < nfsr + ncsrn; i++) 
		{
			mesh_pts[i] = Nhor;
		}
		for (i = nfsr + ncsrn; i < nfsr + ncsrn + ncsrl; i++) 
		{
			mesh_pts[i] = Nhor;
		}

		
		int dimension = 0;
		for (i = 0; i < nfsr; i++) 
		{
			dimension += mesh_pts[i];
		}
		int dimf = std::max(1, nf - nfsr + dimension);
		f = (double *) calloc(dimf, sizeof(double));

		
		
		int dimension1 = 0;
		for (i = nfsr; i < nfsr + ncsrn; i++) 
		{
			dimension1 += mesh_pts[i];
		}
		int dimension2 = 0;
		for (i = nfsr + ncsrn; i < nfsr + ncsrn + ncsrl; i++) 
		{
			dimension2 += mesh_pts[i];
		}
		
		int dimg = std::max(1, nineq + neq - (ncsrl + ncsrn) + dimension1 + dimension2);
		g = (double *) calloc(dimg, sizeof(double));

		
		lambda = (double *) calloc(nparam + dimf + dimg, sizeof(double));

		OptimPlanner::CurrentOptimPlanner = this;
		typedef void (*obj)(int, int, double *, double *, void *);
		typedef void (*cntr)(int, int, double *, double *, double *);

		obj objj = &OptimPlanner::com_objectif;
		cntr cntrr = &OptimPlanner::com_cntr1;


		//=================================================================================
		// Launch the optimization solver
		//=================================================================================

		cfsqp(nparam, nf, nfsr, nineqn, nineq, neqn, neq, ncsrl, ncsrn, mesh_pts,
				mode, iprint, miter, &inform, bigbnd, eps, epsneq, udelta, bl, bu,
				x, f, g, lambda, objj, cntrr, grobfd, grcnfd, cd);

		//=================================================================================
		// Free the memory
		//=================================================================================

		free(mesh_pts);
		free(f);
		free(g);
		free(lambda);
		free(bl);
		free(bu);

		//=================================================================================
		// Compute the trajectories in x and y using the optimal solution
		//=================================================================================

		// step = deltaT;

		// In x direction
		c0 = init[0]; 					// Initial position
		c1 = init[3] * cos(init[2]); // - init[4] * 0.4*sin(init[2]); 	// Initial velocity
		c2 = x[1];						// Coefficient for 2nd-order term
		c3 = x[0];						// Coefficient for 3rd-order term

		// In y direction

		d0 = init[1];					// Initial position
		d1 = init[3] * sin(init[2]); // + init[4] * 0.4*cos(init[2]);	// Initial velocity
		d2 = x[3];						// Coefficient for 2nd-order term
		d3 = x[2];						// Coefficient for 3rd-order term

		
		planif[0] = c0;
		planif[1] = c1;
		planif[2] = c2;
		planif[3] = c3;
		planif[4] = d0;
		planif[5] = d1;
		planif[6] = d2;
		planif[7] = d3;

		/*double xf, yf;
		double t = Nhor * mystep;
			
		xf = c0 + c1 * t + c2 * t * t + c3 * t * t * t; 
		yf = d0 + d1 * t + d2 * t * t + d3 * t * t * t; */

		free(x);
		free(cd);
		return inform;
    }
    
    void OptimPlanner::calcTraj() 
	{
		
		int optim_res; //, optim_res2;
		double init[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
		// double final[4] = {0.0, 0.0, 0.0, 0.0};
		double objectif_courant[3] = {goal.x, goal.y, goal.theta};
		double previous_polynome[8];
		double fin2[2];
		fin2[0] = 1.0;
		fin2[1] = 1.0;

		double current_position[3] = {start.x, start.y, start.theta};
		// double current_velocity[2] = {this->current_v, this->current_w};
		// the initial coefficients of the polynomial are set to zero
		for (int i = 0; i <= 8; i++) 
		{
			previous_polynome[i] = 0.1;
		}

		//int it_loop = 0;  
		//bool bool_notyet_found = true;

		// double vx_final, vy_final;
		init[0] = current_position[0];
		init[1] = current_position[1];
		init[2] = current_position[2];
		init[3] = this->current_v; // the initial speed of the robot
		init[4] = this->current_w; 

		optim_res = this->optimization(init, objectif_courant, fin2, previous_polynome, 1, 0);
		if (optim_res > 0)
		{
			std::cout << "Optimization failed! Exit flag is: " << optim_res << std::endl;
			success_flag = false;
			param_traj.clear();
			for (int i = 0; i < 8; ++i)
			{				
				param_traj.push_back(planif[i]);
			}
		}
		else
		{   
			success_flag = true;
			param_traj.clear();
			for (int i = 0; i < 8; ++i)
			{				
				param_traj.push_back(planif[i]);
			}
		}

	}
	

};
