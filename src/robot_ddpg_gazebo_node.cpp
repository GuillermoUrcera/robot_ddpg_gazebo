#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "robot_ddpg_gazebo/EnvLoopSrv.h"
#include "environmentManager.h"
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "interpolation.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "robot_ddpg_gazebo_node");
  EnvironmentManager env;
  ros::spin();
  return 0;
}


