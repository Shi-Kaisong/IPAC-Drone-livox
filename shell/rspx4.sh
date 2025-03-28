sudo chmod 777 /dev/ttyACM0 & sleep 2;
roslaunch mavros px4.launch & sleep 10;
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 10;
roslaunch fast_lio mapping_mid360.launch & sleep 10;
roslaunch ego_planner single_run_in_exp.launch & sleep 10;
roslaunch ego_planner rviz.launch & sleep 10;
cd .. & rosrun livox_to_mavros livox_to_mavros_node & sleep 10;
rosbag record -a & sleep 10;
roslaunch px4ctrl run_ctrl.launch & sleep 10;
wait;

