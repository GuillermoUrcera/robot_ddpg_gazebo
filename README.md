robot_ddpg_gazebo

example service call:

rosservice call /env_loop_service [0.0,2.0,2.0,10.0,0.0] 5 1 1 0.005 ["obs_1","obs_2","obs_3"] 3 [2,2,5,2,10,2]
