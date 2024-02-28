#! /bin/bash

gnome-terminal -- bash -c "cd ~/catkin_ws; source devel/setup.bash; roslaunch od_model gazebo.launch; exec bash"

echo “launch1  successfully started”
# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore,导致其中一个roslaunch失败,报runid错误
sleep 0.7s  

gnome-terminal -- bash -c "cd ~/catkin_ws; source devel/setup.bash; roslaunch od_model robot_setup.launch; exec bash"

echo “launch2  successfully started”
sleep 0.7s

gnome-terminal -- bash -c "sudo chmod 777 /dev/ttyUSB0;cd ~/catkin_ws; source devel/setup.bash; roslaunch wheel_leg_fsm FSM_node_debug.launch; exec bash"

echo “launch3  successfully started”

