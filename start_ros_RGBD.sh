#! /bin/bash

gnome-terminal -- bash -c "roslaunch openni2_launch openni2.launch; exec bash"

echo “launch1  successfully started”
# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore,导致其中一个roslaunch失败,报runid错误
sleep 0.7s  

gnome-terminal -- bash -c "cd ~/catkin_ws/src/ORB_SLAM2; rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM2/Asus.yaml; exec bash"

echo “launch2  successfully started”


