#include "environmentManager.h"

bool EnvironmentManager::env_loop_func(robot_ddpg_gazebo::EnvLoopSrv::Request &req, robot_ddpg_gazebo::EnvLoopSrv::Response &res){
  // Create the spline:
  // bound conditions=1 means we are limiting the value of the first derivative
  alglib::real_1d_array x;
  alglib::real_1d_array y;
  double temp_x_array[req.num_viapoints];
  double temp_y_array[req.num_viapoints];
  for(unsigned char i=0;i<req.num_viapoints;i++){
	  temp_x_array[i]=i*(req.max_x/(req.num_viapoints-1));
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
  ros::Time t0 = ros::Time::now();
  int e=0;
  while(e<num_points){
	if(ros::Time::now()-t0>ros::Duration(interval_time)){
		gazebo_msgs::ModelState msg;
		msg.model_name="TCP";
		msg.pose.position.x=x_position_array[e];
		msg.pose.position.y=position_array[e];
		msg.pose.position.z=1;
		msg.twist.linear.x=x_velocity;
		msg.twist.linear.y=velocity_array[e];
		this->pub.publish(msg);
		ros::spinOnce();
		t0 = ros::Time::now();
		e+=1;
	}
  }
  return true;
}

float EnvironmentManager::calculateReward(){
}


