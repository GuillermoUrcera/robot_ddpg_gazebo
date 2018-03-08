#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
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
		 service(n.advertiseService("env_loop_service", &EnvironmentManager::env_loop_func, this))
		{}
	private:
		ros::NodeHandle n;
		ros::Publisher pub;
		ros::ServiceServer service;
		bool env_loop_func(robot_ddpg_gazebo::EnvLoopSrv::Request &req, robot_ddpg_gazebo::EnvLoopSrv::Response &res);
};
