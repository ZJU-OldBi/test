#include "OD_Param.h"



WLParam::WLParam(/* args */)
{
}

WLParam::~WLParam()
{
}

void WLParam::config_from_ros_handle(const ros::NodeHandle &nh){
    read_essential_param(nh,"isSim",this->isSim);
    read_essential_param(nh,"gravity",this->g);

    read_essential_param(nh,"Robot/mass",this->robot.mass);
    read_essential_param(nh,"Robot/wheel_distance",this->robot.wheel_distance);
    read_essential_param(nh,"Robot/shoulder_distacne",this->robot.shoulder_distacne);
    read_essential_param(nh,"Robot/wheel_radius",this->robot.wheel_radius);
    read_essential_param(nh,"Robot/L1",this->robot.L1);
    read_essential_param(nh,"Robot/L2",this->robot.L2);    
    read_essential_param(nh,"Robot/L3",this->robot.L3);

    read_essential_param(nh,"msg_timeout/rc",this->msg_timeout.rc);
    read_essential_param(nh,"msg_timeout/imu",this->msg_timeout.imu);
    read_essential_param(nh,"msg_timeout/motor",this->msg_timeout.motor);
    read_essential_param(nh,"msg_timeout/pos",this->msg_timeout.pos);

    read_essential_param(nh,"Control/Pid/velocity/p",this->v_pid_param.p);
    read_essential_param(nh,"Control/Pid/velocity/i",this->v_pid_param.i);
    read_essential_param(nh,"Control/Pid/velocity/d",this->v_pid_param.d);
    read_essential_param(nh,"Control/Pid/velocity/upper_limit",this->v_pid_param.upper_limit);
    read_essential_param(nh,"Control/Pid/velocity/lower_limit",this->v_pid_param.lower_limit);
    read_essential_param(nh,"Control/Pid/velocity/windup_limit",this->v_pid_param.windup_limit);
    read_essential_param(nh,"Control/Pid/velocity/cutoff_frequency",this->v_pid_param.cutoff_frequency);

    read_essential_param(nh,"Control/Pid/angle/p",this->a_pid_param.p);
    read_essential_param(nh,"Control/Pid/angle/i",this->a_pid_param.i);
    read_essential_param(nh,"Control/Pid/angle/d",this->a_pid_param.d);
    read_essential_param(nh,"Control/Pid/angle/upper_limit",this->a_pid_param.upper_limit);
    read_essential_param(nh,"Control/Pid/angle/lower_limit",this->a_pid_param.lower_limit);
    read_essential_param(nh,"Control/Pid/angle/windup_limit",this->a_pid_param.windup_limit);
    read_essential_param(nh,"Control/Pid/angle/cutoff_frequency",this->a_pid_param.cutoff_frequency);

    read_essential_param(nh,"Control/Pid/yaw/p",this->y_pid_param.p);
    read_essential_param(nh,"Control/Pid/yaw/i",this->y_pid_param.i);
    read_essential_param(nh,"Control/Pid/yaw/d",this->y_pid_param.d);
    read_essential_param(nh,"Control/Pid/yaw/upper_limit",this->y_pid_param.upper_limit);
    read_essential_param(nh,"Control/Pid/yaw/lower_limit",this->y_pid_param.lower_limit);
    read_essential_param(nh,"Control/Pid/yaw/windup_limit",this->y_pid_param.windup_limit);
    read_essential_param(nh,"Control/Pid/yaw/cutoff_frequency",this->y_pid_param.cutoff_frequency);
    
    read_essential_param(nh,"Control/Pid/rate/p",this->r_pid_param.p);
    read_essential_param(nh,"Control/Pid/rate/i",this->r_pid_param.i);
    read_essential_param(nh,"Control/Pid/rate/d",this->r_pid_param.d);
    read_essential_param(nh,"Control/Pid/rate/upper_limit",this->r_pid_param.upper_limit);
    read_essential_param(nh,"Control/Pid/rate/lower_limit",this->r_pid_param.lower_limit);
    read_essential_param(nh,"Control/Pid/rate/windup_limit",this->r_pid_param.windup_limit);
    read_essential_param(nh,"Control/Pid/rate/cutoff_frequency",this->r_pid_param.cutoff_frequency);    

    read_essential_param(nh,"RC_switch/low",this->sw_low);
    read_essential_param(nh,"RC_switch/high",this->sw_high);

    read_essential_param(nh,"ctrl_freq_max",this->ctrl_freq_max);
    read_essential_param(nh,"mpc_horizon",this->mpc_horizon);
    read_essential_param(nh,"mpc_freq_max",this->mpc_freq_max);
    read_essential_param(nh,"max_velocity",this->max_velocity);
    read_essential_param(nh,"max_yaw_rate",this->max_yaw_rate);



}