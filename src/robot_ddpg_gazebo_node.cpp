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
  ros::Rate loop_rate(50);
  alglib::real_1d_array x = "[0.0,+0.5,+1.0,+1.5,+2.0]";
  alglib::real_1d_array y = "[0.0,0.25,1.0,0.25,0.0]";
  alglib::spline1dinterpolant s;
  alglib::spline1dbuildcubic(x, y, s);
  float max_time=2;
  float interval_time=0.05;
  int num_points=max_time/interval_time;
  double position_array[num_points];
  double velocity_array[num_points];
  for(unsigned int i=0;i<num_points;i++){
	  position_array[i]=spline1dcalc(s,i*interval_time);
	  std::cout<<"Calculating for"<<i*interval_time<<std::endl;
  }
  ros::Time t0 = ros::Time::now();
  int e=0;
  while(e<num_points){
	if(ros::Time::now()-t0>ros::Duration(interval_time)){
		gazebo_msgs::ModelState msg;
		msg.model_name="box";
		msg.pose.position.x=0;
		msg.pose.position.y=position_array[e];
		std::cout<<position_array[e]<<std::endl;
		pub.publish(msg);
		ros::spinOnce();
		t0 = ros::Time::now();
		e+=1;
		std::cout<<e<<std::endl;
	}
  }
  return 0;
}
