#include "ros/ros.h"
#include "robot_ddpg_gazebo/EnvLoopSrv.h"
#include <stdlib.h>
#include <string>
#include <fstream>

int main(int argc, char **argv){
	// Constants
	const int MAX_VALUE=5;
	const int MAX_X=10;
	const int ITERATIONS_PER_CONFIG=100;
	const int CONFIGS_PER_RUN=100;
	const int NUM_OBSTACLES=3;
	const float MAX_TIME=5;
	const float INTERVAL_TIME=0.01;
	const int NUM_VIAPOINTS=5;
	const std::string OBSTACLE_NAMES[NUM_OBSTACLES]={"obs_1","obs_2","obs_3"};
	const std::string LOG_FILENAME="/tmp/offpolicy_data.csv";
	// File to write to
	std::ofstream csvfile;
	csvfile.open(LOG_FILENAME.c_str());
	// File metainformation
	csvfile<<"max value: "<<MAX_VALUE
		<<";max x: "<<MAX_X
		<<";Iterations per config: "<<ITERATIONS_PER_CONFIG
		<<";Configs per run :"<<CONFIGS_PER_RUN
		<<";Number of obstacles: "<<NUM_OBSTACLES
		<<";Max time: "<<MAX_TIME
		<<";Interval time: "<<INTERVAL_TIME
		<<";Number of viapoints :"<<NUM_VIAPOINTS
		<<"\n";
	// File header
	for(unsigned int i=1;i<NUM_VIAPOINTS-1;i++)csvfile<<"Viapoint "<<i<<";";
	for(unsigned int i=0;i<NUM_OBSTACLES;i++)csvfile<<"Obs_"<<i<<" x;Obs_"<<i<<" y;";
	csvfile<<"Reward\n";
	// ROS
	ros::init(argc, argv, "offpolicy_data_creator");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<robot_ddpg_gazebo::EnvLoopSrv>("env_loop_service");
	robot_ddpg_gazebo::EnvLoopSrv srv;
	// Fill in the static component of the service
	srv.request.num_viapoints=NUM_VIAPOINTS;
	srv.request.num_obstacles=NUM_OBSTACLES;
	for(unsigned char i=0;i<NUM_OBSTACLES;i++)srv.request.obstacles.push_back(OBSTACLE_NAMES[i]);
	srv.request.max_time=MAX_TIME;
	srv.request.max_x=MAX_X;
	srv.request.interval_time=INTERVAL_TIME;
	// Fill in the variable component of the service
	std::srand((unsigned)std::time(0));
	for(unsigned char config=0;config<CONFIGS_PER_RUN;config++){
		// Fill in obstacles
		srv.request.obstacle_positions.clear();
		for(unsigned char obs_p=0;obs_p<NUM_OBSTACLES;obs_p++){
			srv.request.obstacle_positions.push_back(std::rand()%MAX_X); // x
			srv.request.obstacle_positions.push_back(std::rand()%(MAX_VALUE*2)-MAX_VALUE); // y
		}
		for(unsigned char iteration=0;iteration<ITERATIONS_PER_CONFIG;iteration++){
			// Fill in viapoint array
			srv.request.viapoints.clear();
			srv.request.viapoints.push_back(0.f);
			for(unsigned char e=1;e<NUM_VIAPOINTS-1;e++)srv.request.viapoints.push_back(float(float(std::rand())/(RAND_MAX)*MAX_VALUE*2-MAX_VALUE));
			srv.request.viapoints.push_back(0.f);
			// Send srv
			if(client.call(srv)){
				// Write data to csv file
				for(unsigned char j=1;j<NUM_VIAPOINTS-1;j++)csvfile<<srv.request.viapoints[j]<<";";
				for(unsigned char k=0;k<NUM_OBSTACLES;k++)csvfile<<srv.request.obstacle_positions[k*2]<<";"<<srv.request.obstacle_positions[k*2+1]<<";";
				csvfile<<srv.response.reward<<"\n";
			}else{
				ROS_ERROR("ERROR CREATING DATA");
			}
		}
	}
	csvfile.close();
	ROS_INFO("Finished writing data to %s",LOG_FILENAME.c_str());	
	return 0;
}
