#pragma once
#include "kinematics_3dof.h"
#include "OD_Param.h"
#include "sensor_msgs/Imu.h"
#include "OD_Motor.h"
#include "nav_msgs/Odometry.h"
#include "wheel_leg_fsm/ObserveConfig.h"
#include "geometry_msgs/PoseStamped.h"

class Observer
{
private:
    /* data */
    WLParam param; // params
    Filter::lpf pitch_rate_filter, roll_rate_filter, yaw_rate_filter;
public:
    Eigen::Vector3d p_w; // raw ody position in world frame
    Eigen::Vector3d last_p_w; // raw ody position in world frame
    Eigen::Vector3d p_I; // body position in inertial frame
    Eigen::Vector3d v_d; // body velocity in direction frame
    Eigen::Vector3d v_I; // body velocity in inertial frame
    Eigen::Vector3d v_d_ground_truth;
    Eigen::Vector3d v_d_estimated_by_leg;
    Eigen::Vector3d a_d; // body acceleration in direction frame
    Eigen::Vector3d a_I; // body acceleration in inertial frame
    Eigen::Vector3d rl; // left leg position relative to horizon body
    Eigen::Vector3d rlb; // left leg position relative to body
    Eigen::Vector3d Drl;// left leg velocity relative to body
    Eigen::Vector3d rr; // right leg position relative to horizon body
    Eigen::Vector3d rrb; // right leg position relative to body
    Eigen::Vector3d Drr;// right leg velocity relative to body
    Eigen::Vector3d left_leg_angle; // left leg angle
    Eigen::Vector3d left_leg_angular;// left leg angular
    Eigen::Vector3d right_leg_angle; // right leg angle
    Eigen::Vector3d right_leg_angular;// right leg angular
    Eigen::Vector3d euler_rpy,last_euler_rpy; // body euler angle
    Eigen::Vector3d angular_xyz; // body rotation rate
    double left_wheel_speed, right_wheel_speed; // wheel  speed
    bool fall_flag;
    double lpf_filter_a;
    int yaw_round;
    bool vicon_not_change;
    // record msg reveive time
    ros::Time pos_recv_time;

    // velocity kalman filter
    Eigen::Matrix3d input_m;
    Eigen::Matrix3d Q,R;
    Eigen::Matrix3d E,K;

    // ljy
    // Matrix A B C
    Eigen::Matrix<double, 12, 12> A;
    Eigen::Matrix<double, 12, 9> B;
    Eigen::Matrix<double, 15, 12> C;

    // states
    Eigen::Matrix<double, 12, 1> xk;
    Eigen::Matrix<double, 12, 1> u;
    Eigen::Matrix<double, 9, 1> input;
    Eigen::Matrix<double, 15, 1> yk;
    Eigen::Matrix<double, 15, 1> yk_estimated_by_leg;

    Eigen::Matrix3d rotation_imu_rigid;
    // states conv
    Eigen::Matrix<double, 12, 12> E_inertial; // have the same meaning with E;
    Eigen::Matrix<double, 12, 15> K_inertial; // have the same meaning with K;
    Eigen::Matrix<double, 12, 12> Q_inertial; // have the same meaning with Q;
    Eigen::Matrix<double, 15, 15> R_inertial; // have the same meaning with R;


    Observer(WLParam &param_);
    ~Observer();

    // ljy
    void init_A();
    void init_B();
    void init_C();

    void update(sensor_msgs::ImuConstPtr imu_ptr);
    void update(const OD_motor & motor);
    void update(nav_msgs::OdometryConstPtr odom_ptr);
    void vio_update(nav_msgs::OdometryConstPtr odom_ptr);
    void vicon_update(geometry_msgs::PoseStampedConstPtr vicon_pose_ptr);
    void step();
    void inertial_step();
    void observe_reconfigureCallback(wheel_leg_fsm::ObserveConfig& config, uint32_t level);
    Eigen::Matrix<double,14,1> get_stance_state() const;
    Eigen::Matrix<double,17,1> get_mpc_state() const;
    Eigen::Matrix3d right_Jp,left_Jp,right_Jr,left_Jr; // jacobian matrix
    Eigen::Matrix3d rotation_rp; // rotation matrix from direction frame to body frame
    Eigen::Matrix3d rotation_y;
    Eigen::Quaterniond quaternion;

    dynamic_reconfigure::Server<wheel_leg_fsm::ObserveConfig> *observe_config_server;
    dynamic_reconfigure::Server<wheel_leg_fsm::ObserveConfig>::CallbackType observe_cbt;
};