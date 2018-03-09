#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "std_srvs/Empty.h"
#include "robot_ddpg_gazebo/EnvLoopSrv.h"
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "interpolation.h"

class EnvironmentManager{
	public:
		explicit EnvironmentManager()
		:n(),
		 pub(n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",10)),
		 service(n.advertiseService("env_loop_service", &EnvironmentManager::env_loop_func, this)),
		 reset_client(n.serviceClient<std_srvs::Empty>("/gazebo/reset_world"))
		{}
	private:
		ros::NodeHandle n;
		ros::Publisher pub;
		ros::ServiceServer service;
		ros::ServiceClient reset_client;
		bool env_loop_func(robot_ddpg_gazebo::EnvLoopSrv::Request &req, robot_ddpg_gazebo::EnvLoopSrv::Response &res);
		float calculateReward();
		void reset();
};
