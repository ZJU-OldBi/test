#pragma once

#include <ros/ros.h>
#include "sbus_serial/Sbus.h"
#include <sensor_msgs/Imu.h>
#include "OD_Param.h"
#include <std_msgs/Bool.h>
#include "can_msgs/to_ros.h"
#include "OD_Motor.h"
#include "OD_Controller.h"
#include "can_msgs/Frame.h"
#include "std_msgs/Float64MultiArray.h"
#include "OD_Observer.h"
#include "OD_Ratio.h"
#include "OD_Regulator.h"
#include <mutex>
class FSM
{
public:
    ~FSM();
    FSM(WLParam &param_,Controller &controller_,Observer &observer_,Ratio &ratio_,Regulator &regulator_);
    enum State_t
	{
		OB = 1,// no control msg pub , listen from sensors
		STAND,// leg stiff, wheel slack
		BALANCE,// leg stiff , wheel try to balance and follow a speed
        POSITION, // try to stand at a position
        FALL // when the robot fall down
	};
    Eigen::Matrix<double,8,1> leg_force;
    OD_motor left_motor_1,left_motor_2,left_motor_3,left_motor_4,right_motor_1,right_motor_2,right_motor_3,right_motor_4;
    WLParam &param; // params
    Controller &controller;
    Observer &observer;
    Ratio &ratio;
    Regulator &regulator;

    ros::Publisher left_can_pub, right_can_pub;
    ros::Publisher left_motor1_pub, left_motor2_pub, left_motor3_pub, left_motor4_pub; 
    ros::Publisher right_motor1_pub,right_motor2_pub,right_motor3_pub,right_motor4_pub;
    ros::Publisher joint_state_pub,left_wrench_pub,right_wrench_pub,state_pub;
    ros::Publisher debug_pub;

    void loop();
    void publish_joint_state(double hz);
    void publish_state(double hz);
    void publish_wrench(double hz);
    bool rc_is_received(const ros::Time &now_time);
    bool imu_is_received(const ros::Time &now_time);
    bool motor_is_received(const ros::Time &now_time);
    bool pos_is_received(const ros::Time &now_time);

    void cmd_cvel(geometry_msgs::Twist vel_msg);

    void imu_cb(sensor_msgs::ImuConstPtr msg);
    void rc_cb(sbus_serial::SbusConstPtr msg);
    void left_can_cb(can_msgs::Frame::ConstPtr msg);
    void right_can_cb(can_msgs::Frame::ConstPtr msg);   
    void debug_cb(std_msgs::Float64MultiArray::ConstPtr msg);
    void ground_truth_cb(const nav_msgs::Odometry::ConstPtr msg);
    void vio_cb(const nav_msgs::Odometry::ConstPtr msg);
    void vicon_cb(const geometry_msgs::PoseStampedConstPtr msg);
    void wbc_calc();
private:
    State_t _state,_last_state;
    std_msgs::Float64MultiArray debug_sub_msg;
    ros::Time _rc_rcv_stamp, _imu_rcv_stamp, _left_motor_rcv_stamp,_right_motor_rcv_stamp,_pos_rcv_stamp;
    std::vector<OD_motor *> Motors;
    std::mutex leg_force_mutex,observer_mutex,QR_mutex;
    
    void _change_state(const ros::Time &now_time);
};