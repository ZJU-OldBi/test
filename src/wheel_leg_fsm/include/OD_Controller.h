#pragma once
#include "OD_Param.h"
#include "kinematics_3dof.h"
#include "OD_Observer.h"
#include "OD_Ratio.h"
#include "OD_Regulator.h"
#include <geometry_msgs/WrenchStamped.h>
#include <dynamic_reconfigure/server.h>
#include "wheel_leg_fsm/LqrConfig.h"
#include "wheel_leg_fsm/ManualConfig.h"
#include "wheel_leg_fsm/MPCConfig.h"
#include "ocp.h"
#include <mutex>
class Controller
{
private:
    /* data */
    pid_ns::PidObject _velocity_pid, _angle_pid, _yaw_pid, _angular_pid;
    WLParam param;
    ros::NodeHandle nh;
    ros::Publisher mpc_path_pub,mpc_desire_pub;
    double Ts;
    
    bool _lqr_first_config,_manual_first_config;
    
    void update_wrench(Eigen::Matrix<double , 8 ,1> u);
public:
    Controller(WLParam &param_);
    
    ~Controller();
    Eigen::Matrix<double,8,14> K_DOUBLE;
    Eigen::Matrix<double,8,3> Kc_DOUBLE;
    std::mutex K_mutex;
    ros::Time last_balance_calc_time,last_position_calc_time,last_lqr_calc_time;
    bool first_calc;
    Eigen::MatrixXd KK;
    ocp _ocp;
    dynamic_reconfigure::Server<wheel_leg_fsm::LqrConfig> *lqr_config_server;
    dynamic_reconfigure::Server<wheel_leg_fsm::MPCConfig> *mpc_config_server;
    dynamic_reconfigure::Server<wheel_leg_fsm::ManualConfig> *manual_config_server;
    dynamic_reconfigure::Server<wheel_leg_fsm::LqrConfig>::CallbackType lqr_cbt;
    dynamic_reconfigure::Server<wheel_leg_fsm::MPCConfig>::CallbackType mpc_cbt;
    dynamic_reconfigure::Server<wheel_leg_fsm::ManualConfig>::CallbackType manual_cbt;
    double height_inte_err,vx_inte_err,yaw_rate_inte_err;
    geometry_msgs::WrenchStamped left_wrench,right_wrench;
    Eigen::Matrix<double,8,1> balance_stand_calc(const Observer & obser,const Ratio & ratio, Regulator & regulator);
    Eigen::Matrix<double,8,1> position_stand_calc(const Observer & obser,const Ratio & ratio, std_msgs::Float64MultiArray & debug_msg);
    Eigen::Matrix<double,8,1> pid_stand_calc(const Observer & obser,const Ratio & ratio,std_msgs::Float64MultiArray & debug_msg);
    Eigen::Matrix<double,8,1> mpc_calc(Observer & obser,const Ratio & ratio,Regulator & regulator,std_msgs::Float64MultiArray & debug_msg);
    Eigen::Matrix<double,8,1> lqr_calc(Observer & obser,const Ratio & ratio,Regulator & regulator,std_msgs::Float64MultiArray & debug_msg);
    double cal_des_roll(double des_vx,double des_wz,double des_height);
    double output_ratio,dx_balance,ratio1,ratio2,ratio3;
    void lqr_reconfigureCallback(wheel_leg_fsm::LqrConfig& config, uint32_t level);
    void mpc_reconfigureCallback(wheel_leg_fsm::MPCConfig& config, uint32_t level);
    void manual_reconfigureCallback(wheel_leg_fsm::ManualConfig& config, uint32_t level);
    void reset_intergrator();
    void publish_traj(Eigen::MatrixXd traj_vector,ros::Publisher publisher);
};
