#pragma once
#include "OD_Param.h"
#include "OD_Ratio.h"
#include "OD_Observer.h"
#include <ros/ros.h>
#include "wheel_leg_fsm/RegulatorConfig.h"

class Regulator
{
public:
    enum Traj_t{
        IDLE,
        CIRCLE,
        SPACE_WALK,
        SPIRAL,
        RECT
    };
    ros::NodeHandle node_regulator;
    Eigen::Vector3d start_pos;
    bool isRunning;
    double start_yaw;
    void start(Observer obser,double max_time);
    void reset();
    void setRef(double x,double y,double z,double yaw);
    Eigen::MatrixXd get_des_state(ros::Time given_time) const;
    Regulator(WLParam &param_);
    ~Regulator();
    void regulator_reconfigureCallback(wheel_leg_fsm::RegulatorConfig& config, uint32_t level);
    dynamic_reconfigure::Server<wheel_leg_fsm::RegulatorConfig> *regulator_config_server;
    dynamic_reconfigure::Server<wheel_leg_fsm::RegulatorConfig>::CallbackType regulator_cbt;

private:
    /* data */
    WLParam param;
    ros::Time start_time;
    
    double MAX_TIME;
    Traj_t traj,next_traj;
    ros::Publisher path_pub;
};



