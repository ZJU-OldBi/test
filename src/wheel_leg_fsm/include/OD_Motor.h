#pragma once
#include "can_msgs/Frame.h"
#include "can_msgs/to_ros.h"
#include "../../filter/include/filter.h"
#include <ros/ros.h>
class OD_motor
{
const double POS_MIN =-12.5;
const double POS_MAX = 12.5;
const double SPD_MIN =-18.0;
const double SPD_MAX = 18.0;
const double I_MIN = -30.0;
const double I_MAX = 30.0;
const double T_MIN = -30.0;
const double T_MAX = 30.0;
const double KP_MAX = 500.0;
const double KD_MAX = 50.0;

private:
    /* data */
    double raw_velocity;
    double torque_cmd;
    uint8_t error_flag;
    uint8_t ack_mode;
    double max_torque;
public:
    uint32_t can_id;
    double position,velocity,current,temperature;
    double zero_angle,delta_angle;
    double ANGLE_UP,ANGLE_LOW; // angle limit
    std::string side_name; // left or right
    bool turn_flag,no_angle_limit;
    Filter::lpf speed_filter;

    void update(can_msgs::Frame::ConstPtr can_frame);
    can_msgs::Frame set_pos(double pos,double vel,double max_cur);
    can_msgs::Frame set_vel(double vel,double max_cur);
    can_msgs::Frame set_tor(double tor);
    can_msgs::Frame set_tor(double pos,double vel,double forward_tor,double Kp,double Kd,double max_tor);
    can_msgs::Frame set_mix(double pos,double vel,double forward_tor,double Kp,double Kd);
    void init(std::string side_name,uint8_t can_id,double zero_angle,double delta_angle,int turn_flag,uint8_t ack,double angle_up, double angle_low, double max_tor); // 0 stands for change direction
    can_msgs::to_ros get_ros_msg();
    OD_motor();
    ~OD_motor();
};
