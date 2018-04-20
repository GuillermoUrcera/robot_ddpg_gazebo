#include "environmentManager.h"

bool EnvironmentManager::env_loop_func(robot_ddpg_gazebo::EnvLoopSrv::Request &req, robot_ddpg_gazebo::EnvLoopSrv::Response &res){
  // Create the spline:
  // bound conditions=1 means we are limiting the value of the first derivative
  alglib::real_1d_array x;
  alglib::real_1d_array y;
  double temp_x_array[req.num_viapoints];
  double temp_y_array[req.num_viapoints];
  for(unsigned char i=0;i<req.num_viapoints;i++){
	  temp_x_array[i]=i*(req.max_time/(req.num_viapoints-1));
	  temp_y_array[i]=req.viapoints[i];
  }
  x.setcontent(int(req.num_viapoints), &(temp_x_array[0]));
  y.setcontent(int(req.num_viapoints), &(temp_y_array[0]));
  alglib::spline1dinterpolant s;
  alglib::ae_int_t num_viapoints=req.num_viapoints;
  alglib::ae_int_t left_bound_condition=1;
  double left_bound=0;
  alglib::ae_int_t right_bound_condition=1;
  double right_bound=0;
  alglib::spline1dbuildcubic(x, y, num_viapoints, left_bound_condition, left_bound, right_bound_condition, right_bound, s);
  // Discretize path
  float max_x=req.max_x;
  float max_time=req.max_time;
  float interval_time=req.interval_time;
  int num_points=max_time/interval_time;
  float x_position_array[num_points];
  float x_velocity=max_x/max_time;
  double position_array[num_points];
  double velocity_array[num_points];
  double acc_array[num_points];
  for(unsigned int i=0;i<num_points;i++){
	  alglib::spline1ddiff(s,i*interval_time,position_array[i],velocity_array[i],acc_array[i]);
	  x_position_array[i]=i*interval_time*x_velocity;
  }
  // Calculate distance
  float distance=0.f;
  float obstacle_displacement=0.f;
  for(unsigned int i=1;i<num_points-1;i++)distance+=std::sqrt(std::pow(x_position_array[i+1]-x_position_array[i],2)+std::pow(position_array[i+1]-position_array[i],2));
  // Set initial positions
  gazebo_msgs::SetModelState srv;
  for(unsigned char i=0;i<req.num_obstacles;i++){
	srv.request.model_state.model_name=req.obstacles[i];
	srv.request.model_state.pose.position.x=req.obstacle_positions[i*2];
	srv.request.model_state.pose.position.y=req.obstacle_positions[i*2+1];
	srv.request.model_state.pose.position.z=0;
	if(this->obstacle_client_setter.call(srv)){
	  ROS_INFO("Obstacle set");
	}else{
	  ROS_WARN("Obstacles unable to be set");
	}
  }
  // Run episode
  ros::Time t0 = ros::Time::now();
  int e=0;
  while(e<num_points){
	if(ros::Time::now()-t0>ros::Duration(interval_time)){
		gazebo_msgs::ModelState msg;
		msg.model_name="TCP";
		msg.pose.position.x=x_position_array[e];
		msg.pose.position.y=position_array[e];
		msg.pose.position.z=0.25;
		msg.twist.linear.x=x_velocity;
		msg.twist.linear.y=velocity_array[e];
		this->pub.publish(msg);
		ros::spinOnce();
		t0 = ros::Time::now();
		e+=1;
	}
  }
  // Calculate reward
  float reward=0;
  gazebo_msgs::GetModelState get_srv;
  for(unsigned char i=0;i<req.num_obstacles;i++){
	  get_srv.request.model_name=req.obstacles[i];
	  if(this->obstacle_client_getter.call(get_srv)){
		obstacle_displacement+=std::sqrt(std::pow(float(get_srv.response.pose.position.x)-float(req.obstacle_positions[i*2]),2)+std::pow(float(get_srv.response.pose.position.y)-float(req.obstacle_positions[i*2+1]),2));
	  }else{
		ROS_ERROR("OBSTACLE POSITION NOT READABLE!");
	  }
  }
  reward=-obstacle_displacement-(distance*this->DISTANCE_MOD);
  // Return reward
  res.reward=reward;
  res.distance_covered=distance;
  res.obstacle_displacement=obstacle_displacement;
  return true;
}

void EnvironmentManager::reset(){
	std_srvs::Empty e;
	if(this->reset_client.call(e)){
		ROS_INFO("Gazebo world reset");
	}else{
		ROS_WARN("Gazebo world unable to be reset");
	}
}

bool EnvironmentManager::kautham_loop_func(robot_ddpg_gazebo::KauthamLoopSrv::Request &req, robot_ddpg_gazebo::KauthamLoopSrv::Response &res){
  // Calculate distance
  float distance=0.f;
  float obstacle_displacement=0.f;
  for(unsigned int i=1;i<req.num_points-1;i++)distance+=std::sqrt(std::pow(req.x[i+1]-req.x[i],2)+std::pow(req.y[i+1]-req.y[i],2));
  // Set initial positions
  gazebo_msgs::SetModelState srv;
  for(unsigned char i=0;i<req.num_obstacles;i++){
	srv.request.model_state.model_name=req.obstacles[i];
	srv.request.model_state.pose.position.x=req.obstacle_positions[i*2];
	srv.request.model_state.pose.position.y=req.obstacle_positions[i*2+1];
	srv.request.model_state.pose.position.z=0;
	if(this->obstacle_client_setter.call(srv)){
	  ROS_INFO("Obstacle set");
	}else{
	  ROS_WARN("Obstacles unable to be set");
	}
  }
  // Run episode
  ros::Time t0 = ros::Time::now();
  int e=0;
  while(e<req.num_points){
	if(ros::Time::now()-t0>ros::Duration(float(req.t[e]))){
		gazebo_msgs::ModelState msg;
		msg.model_name="TCP";
		msg.pose.position.x=req.x[e];
		msg.pose.position.y=req.y[e];
		msg.pose.position.z=0.25;
		this->pub.publish(msg);
		ros::spinOnce();
		t0 = ros::Time::now();
		e+=1;
	}
  }
  // Calculate reward
  float reward=0;
  gazebo_msgs::GetModelState get_srv;
  for(unsigned char i=0;i<req.num_obstacles;i++){
	  get_srv.request.model_name=req.obstacles[i];
	  if(this->obstacle_client_getter.call(get_srv)){
		obstacle_displacement+=std::sqrt(std::pow(float(get_srv.response.pose.position.x)-float(req.obstacle_positions[i*2]),2)+std::pow(float(get_srv.response.pose.position.y)-float(req.obstacle_positions[i*2+1]),2));
	  }else{
		ROS_ERROR("OBSTACLE POSITION NOT READABLE!");
	  }
  }
  reward=-obstacle_displacement-(distance*this->DISTANCE_MOD);
  // Return reward
  res.reward=reward;
  res.distance_covered=distance;
  res.obstacle_displacement=obstacle_displacement;
  return true;
}
