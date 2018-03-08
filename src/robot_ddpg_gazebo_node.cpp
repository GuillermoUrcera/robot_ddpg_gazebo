#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "interpolation.h"
#include <iostream>
int main(int argc, char **argv){
  ros::init(argc, argv, "robot_ddpg_gazebo_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",10);
  // Create the spline:
  // bound conditions=1 means we are limiting the value of the first derivative
  alglib::real_1d_array x = "[0.0,+0.5,+1.0,+1.5,+2.0]";
  alglib::real_1d_array y = "[0.0,0.25,1.0,0.25,0.0]";
  alglib::spline1dinterpolant s;
  alglib::ae_int_t num_viapoints=5;
  alglib::ae_int_t left_bound_condition=1;
  double left_bound=0;
  alglib::ae_int_t right_bound_condition=1;
  double right_bound=0;
  alglib::spline1dbuildcubic(x, y, num_viapoints, left_bound_condition, left_bound, right_bound_condition, right_bound, s);
  // TODO max_time max_x and inerval_time should be set via a service
  float max_x=2;
  float max_time=2;
  float interval_time=0.05;
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
		msg.model_name="box";
		msg.pose.position.x=x_position_array[e];
		msg.pose.position.y=position_array[e];
		msg.twist.linear.x=x_velocity;
		msg.twist.linear.y=velocity_array[e];
		pub.publish(msg);
		ros::spinOnce();
		t0 = ros::Time::now();
		e+=1;
	}
  }
  return 0;
}
