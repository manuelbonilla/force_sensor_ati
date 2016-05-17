# force_sensor_ati

This repo contains  the code to parse c++ some API/functions of the ati force sensor to ROS.

Compile doing 'catkin_make --only_pkg-with-deps force_sensor_ati' in your workspace once you followed the steps described in the readme from ATI http://optoforce.com/support/

The command to launch the robot in simulation is : 
  - roslaunch force_sensor_ati force_sensor_ati.launch 

To set zeros on sensors just

  - rosservice call /left/setZeros std_srvs/Empty "{}"


Steps

  - Conect to kuka-platform network
  - Open a terminal
  - roslaunch force_sensor_ati force_sensor_ati.launch 
  - Init on Matlab
  - ruy simulink script