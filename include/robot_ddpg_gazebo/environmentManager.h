#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "std_srvs/Empty.h"
#include "robot_ddpg_gazebo/EnvLoopSrv.h"
#include "robot_ddpg_gazebo/KauthamLoopSrv.h"
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include "interpolation.h"

class EnvironmentManager{
	public:
		explicit EnvironmentManager()
		:n(),
		 pub(n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",10)),
		 service(n.advertiseService("env_loop_service", &EnvironmentManager::env_loop_func, this)),
		 kautham_service(n.advertiseService("kautham_loop_service", &EnvironmentManager::kautham_loop_func, this)),
		 reset_client(n.serviceClient<std_srvs::Empty>("/gazebo/reset_world")),
		 obstacle_client_setter(n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state")),
		 obstacle_client_getter(n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state")),
		 DISTANCE_MOD(0.5)
		{}
	private:
		ros::NodeHandle n;
		ros::Publisher pub;
		ros::ServiceServer service;
		ros::ServiceServer kautham_service;
		ros::ServiceClient reset_client;
		ros::ServiceClient obstacle_client_setter;
		ros::ServiceClient obstacle_client_getter;
		bool env_loop_func(robot_ddpg_gazebo::EnvLoopSrv::Request &req, robot_ddpg_gazebo::EnvLoopSrv::Response &res);
		bool kautham_loop_func(robot_ddpg_gazebo::KauthamLoopSrv::Request &req, robot_ddpg_gazebo::KauthamLoopSrv::Response &res);
		void reset();
		const float DISTANCE_MOD;
};
