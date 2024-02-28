#include  "OD_Controller.h"
#include "Timer.hpp"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
double clamp(double data,double upper,double lower);
double dead_zone_compensate(double raw_value,double dead_zone);
double dead_zone_pwm(double raw_value,double dead_zone);
double yaw_error(double yaw1,double yaw2);

Controller::Controller(WLParam &param_): param(param_)
,_velocity_pid(param_.v_pid_param,"v_pid"),_angle_pid(param_.a_pid_param,"a_pid"),_yaw_pid(param_.y_pid_param,"y_pid"),_angular_pid(param_.r_pid_param,"r_pid")
,_ocp(param_,17,8,3)
{
    this->Ts = 1.0/param.ctrl_freq_max;
    height_inte_err = 0;
    vx_inte_err = 0;
    yaw_rate_inte_err = 0;
    Eigen::DiagonalMatrix<double,17> Q_diag;
    Eigen::DiagonalMatrix<double,8> R_diag;
    Q_diag.diagonal() << 31, 210, 1, 400, 1, 400, 1, 300, 1, 100, 4000, 1, 4000, 1, 30, 250, 200;
    R_diag.diagonal() << 0.055, 0.023, 0.013, 0.33, 0.055, 0.023, 0.013, 0.33;
    _ocp.setWeightLQR(Q_diag,R_diag);

    Q_diag.diagonal() << 330, 330, 58, 300, 1, 400, 1, 300, 1, 450, 1, 80,  1, 4500, 1, 4500, 1;
    R_diag.diagonal() << 0.005, 0.03, 0.008, 0.08, 0.005, 0.03, 0.008, 0.08;
    _ocp.setWeightMPC(Q_diag,R_diag);
    _ocp.setHorizon(param.mpc_horizon);
    // _ocp.setConstrainMatrix();
    /*----------------- LQR -------------------*/
    ROS_INFO_STREAM("********** init lqr weight setup ***************");
    Eigen::MatrixXd K;
    if(_ocp.dlqr_calc(&K,1e-7)){
        K(1,2) *= 0.5;
        K(2,2) *= 0.5;
        K(5,2) *= 0.5;
        K(6,2) *= 0.5;

        K(2,4) *= 0.7;
        K(6,4) *= 0.7;

        K(1,6) *= 0.5;
        K(2,6) *= 0.5;
        K(5,6) *= 0.5;
        K(6,6) *= 0.5;

        K(0,8) *= 0.7;
        K(3,8) *= 0.7;
        K(4,8) *= 0.7;
        K(7,8) *= 0.7;

        K(0,11) *= 1.0;
        K(0,13) *= 1.0;
        K(3,11) *= 1.0;
        K(3,13) *= 1.0;
        K(4,11) *= 1.0;
        K(4,13) *= 1.0;
        K(7,11) *= 1.0;
        K(7,13) *= 1.0;
        K_DOUBLE << K.block<8,14>(0,0);
        Kc_DOUBLE << -K.block<8,3>(0,14);
    }
    else{
        K_DOUBLE.setZero();
        Kc_DOUBLE.setZero();
        ROS_ERROR_STREAM("LQR calc failed !");
    }
    Eigen::IOFormat fmt(5, 0, ",", "\n", "", ",");
    ROS_INFO_STREAM("K:\n"<< K_DOUBLE.format(fmt) << std::endl);
    ROS_INFO_STREAM("Kc:\n"<< Kc_DOUBLE.format(fmt) << std::endl);
    KK.resize(8,15);
    KK <<
     10.611,         0,         0,         0,         0,         0,         0,   -37.261,   -1.7582,    -25.984,    -18.8740,     -31.633,   -3.5273,    -52.09,     -3.0888,
          0,    92.991,    18.899,2.2204e-15,3.3307e-16,    73.279,    2.2098,         0,         0,          0,          0,          0,         0,          0,          0,
          0,   -434.39,   -44.998,    195.94,    28.145,    59.121,    2.6701,         0,         0,          0,          0,          0,         0,          0,          0,
      6.547,         0,         0,         0,         0,         0,         0,    10.266,   0.51193,    3.25205,     2.7651,    -104.17,   -3.9643,     3.1391,    0.62417,
     10.611,         0,         0,         0,         0,         0,         0,   -37.261,   -1.7582,      25.984,     18.8740,     -52.09,   -3.0888,    -31.633,    -3.5273,
          0,    92.991,    18.899,1.3323e-15,2.2204e-16,    73.279,    2.2098,         0,         0,          0,          0,          0,         0,          0,          0,
          0,    434.39,    44.998,    195.94,    28.145,   -59.121,   -2.6701,         0,         0,          0,          0,          0,         0,          0,          0,
       6.547,         0,         0,         0,         0,         0,         0,    10.266,   0.51193,  -3.25205,     -2.7651,    3.1391,   0.62417,    -104.17,    -3.9643,

    _lqr_first_config = true;
    _manual_first_config = true;
    last_balance_calc_time = ros::Time::now();
    last_position_calc_time = ros::Time::now();
    last_lqr_calc_time = ros::Time::now();
    // yaw_round = 0;
    /*----------------- LQR -------------------*/

    output_ratio = 0;
    left_wrench.header.frame_id = "left_contact";
    right_wrench.header.frame_id = "right_contact";
    ros::NodeHandle node_lqr("fsm_lqr");
    lqr_config_server = new dynamic_reconfigure::Server<wheel_leg_fsm::LqrConfig>(node_lqr);
    lqr_cbt = boost::bind(&Controller::lqr_reconfigureCallback, this, _1, _2);
    lqr_config_server->setCallback(lqr_cbt);

    ros::NodeHandle node_manual("fsm_manual");
    manual_config_server = new dynamic_reconfigure::Server<wheel_leg_fsm::ManualConfig>(node_manual);
    manual_cbt = boost::bind(&Controller::manual_reconfigureCallback, this, _1, _2);
    manual_config_server->setCallback(manual_cbt);

    ros::NodeHandle node_mpc("fsm_mpc");
    mpc_config_server = new dynamic_reconfigure::Server<wheel_leg_fsm::MPCConfig>(node_mpc);
    mpc_cbt = boost::bind(&Controller::mpc_reconfigureCallback, this, _1, _2);
    mpc_config_server->setCallback(mpc_cbt);

    mpc_path_pub = nh.advertise<nav_msgs::Path>("/mpc_predict_path",10);
    mpc_desire_pub = nh.advertise<nav_msgs::Path>("/mpc_desire_path",10);
}

Controller::~Controller()
{
    delete lqr_config_server;
    delete manual_config_server;
    _ocp.free_qp();
}
/*
leg motor position and wheel torque
*/
Eigen::Matrix<double,8,1> Controller::pid_stand_calc(const Observer & obser,const Ratio & ratio,std_msgs::Float64MultiArray & debug_msg){
    double des_z = 0.35+0.1*ratio.right_knob;
    Eigen::Vector3d left_angle  = Kinematics::ik(Eigen::Vector3d(dx_balance,param.robot.L3,-des_z),param.robot.L1,param.robot.L2,param.robot.L3);
    Eigen::Vector3d right_angle = Kinematics::ik(Eigen::Vector3d(dx_balance,-param.robot.L3,-des_z),param.robot.L1,param.robot.L2,-param.robot.L3);
    // double des_angle = _velocity_pid.doCalcs(0.5*param.robot.wheel_radius*(obser.left_wheel_speed+obser.right_wheel_speed),0.3*ratio.right_v);
    double des_angle = _velocity_pid.doCalcs(obser.v_d(0),0.3*ratio.right_v);
    double forward_tor = _angle_pid.doCalcs(obser.euler_rpy(1),des_angle) + _angular_pid.doCalcs(obser.angular_xyz(1),0.0);
    double rotate_tor = _yaw_pid.doCalcs(obser.angular_xyz(2),0.3*ratio.left_h);
    double left_wheel_tor = dead_zone_compensate(forward_tor - rotate_tor,0.1);
    double right_wheel_tor = dead_zone_compensate(forward_tor + rotate_tor,0.1);
    Eigen::Matrix<double,8,1> ans;
    ans << left_angle,left_wheel_tor,right_angle,right_wheel_tor;

    // debug_msg.data.push_back(left_angle(0));
    // debug_msg.data.push_back(left_angle(1));
    // debug_msg.data.push_back(left_angle(2));
    // debug_msg.data.push_back(left_wheel_tor);
    // debug_msg.data.push_back(right_angle(0));
    // debug_msg.data.push_back(right_angle(1));
    // debug_msg.data.push_back(right_angle(2));
    // debug_msg.data.push_back(right_wheel_tor);

    // debug_msg.data.push_back(0.3*ratio.right_v); // des_v
    // debug_msg.data.push_back(0.3*ratio.left_h); //des_yaw_rate

    // debug_msg.data.push_back(des_angle);
    // debug_msg.data.push_back(forward_tor);
    // debug_msg.data.push_back(rotate_tor);

    return ans;
}

/*
leg motor torque and wheel torque
*/
// ,std_msgs::Float64MultiArray & debug_msg
Eigen::Matrix<double,8,1> Controller::balance_stand_calc(const Observer & obser,const Ratio & ratio,Regulator & regulator){
    static double des_vx = 0;
    static double des_wz = 0;
    static double des_z = 0;
    static double des_y = 0;
    static double des_pitch = 0;
    static double des_dx = 0;
    static double open_force = 0;
    static double des_yaw = 0;
    Eigen::Matrix<double,8,1> motor_torque;
    Eigen::Matrix<double,14,1>state = obser.get_stance_state();
    Eigen::Matrix<double,14,1>balance_state;
    Eigen::Matrix<double,8,1> u,balance_u;

    des_vx = ratio.dx;
    des_wz = ratio.dw;
    des_z = 0.35;

    // des_vx = 0.8*(0.35*ratio.right_v) +0.2*des_vx;
    // des_wz = 0.8*(-0.5*ratio.left_h)+ 0.2*des_wz;
    // des_z = 0.5*(0.35+0.1*ratio.right_knob)+0.5*des_z;

    // des_y = 0.2*(-0.05*ratio.right_h) + 0.8*des_y;
    des_dx = 0;//0.5*(0.08*ratio.right_knob)+0.5*des_dx;

    
    balance_state << 0,des_y,0,des_z,0,0,0,des_pitch,0,0,dx_balance+des_dx,0,dx_balance-des_dx,0;
    open_force = 0.1*(ratio1*(obser.rl(1)-obser.rr(1) - param.robot.wheel_distance))+0.9*open_force;
    balance_u << 0,open_force-36*param.robot.L3 / 0.42,36,0,0,-open_force+36*param.robot.L3 / 0.42,36,0;
    
    vx_inte_err += Ts*(des_vx - state(0));
    vx_inte_err = clamp(vx_inte_err,1.5,-1.5);
    ros::Time now_time = ros::Time::now();
    if(now_time.toSec() - last_balance_calc_time.toSec() > 0.5){
        des_yaw = obser.euler_rpy(2);
    }
    else{
        des_yaw += Ts*des_wz;
        yaw_rate_inte_err = yaw_error(des_yaw,obser.euler_rpy(2));
        yaw_rate_inte_err = clamp(yaw_rate_inte_err,1.0,-1.0);
    }
    last_balance_calc_time = now_time;
    // ROS_INFO_STREAM_THROTTLE(1,"des_yaw: "<<des_yaw<< " now_yaw: "<<obser.euler_rpy(2) << " error: "<<yaw_rate_inte_err);
    height_inte_err += Ts*(des_z - state(3));
    height_inte_err = clamp(height_inte_err,0.3,-0.3);

    K_mutex.lock();
    u = Kc_DOUBLE*Eigen::Vector3d(vx_inte_err,yaw_rate_inte_err,height_inte_err)
     - K_DOUBLE*(state - balance_state)
     + balance_u;
    K_mutex.unlock();
    // u(3) = dead_zone_pwm(u(3),ratio3);
    // u(7) = dead_zone_pwm(u(7),ratio3);
    //force clamp
    u(0) = clamp(u(0),20,-20);
    u(1) = clamp(u(1),15,-15);
    u(2) = clamp(u(2),80,10);
    u(3) = clamp(u(3),12,-12);
    u(4) = clamp(u(4),20,-20);
    u(5) = clamp(u(5),15,-15);
    u(6) = clamp(u(6),80,10);
    u(7) = clamp(u(7),12,-12);
    update_wrench(u);
    Eigen::Vector3d left_leg_torque = obser.left_Jp.transpose()*obser.rotation_rp.transpose()*Eigen::Vector3d(-u(0),-u(1),-u(2));
    Eigen::Vector3d right_leg_torque = obser.right_Jp.transpose()*obser.rotation_rp.transpose()*Eigen::Vector3d(-u(4),-u(5),-u(6));
    left_leg_torque += obser.left_Jr.transpose()*obser.rotation_rp.transpose()*Eigen::Vector3d(0,-u(3),0);
    right_leg_torque += obser.right_Jr.transpose()*obser.rotation_rp.transpose()*Eigen::Vector3d(0,-u(7),0);    
    motor_torque << left_leg_torque,-u(3),right_leg_torque,-u(7);

    // debug_msg.data.push_back(motor_torque(0));
    // debug_msg.data.push_back(motor_torque(1));
    // debug_msg.data.push_back(motor_torque(2));
    // debug_msg.data.push_back(motor_torque(3));
    // debug_msg.data.push_back(motor_torque(4));
    // debug_msg.data.push_back(motor_torque(5));
    // debug_msg.data.push_back(motor_torque(6));
    // debug_msg.data.push_back(motor_torque(7));

    // debug_msg.data.push_back(des_vx);
    // debug_msg.data.push_back(des_wz);
    // debug_msg.data.push_back(des_y);
    // debug_msg.data.push_back(des_z);
    // debug_msg.data.push_back(des_pitch);

    return motor_torque;
}

Eigen::Matrix<double,8,1> Controller::position_stand_calc(const Observer & obser,const Ratio & ratio,std_msgs::Float64MultiArray & debug_msg){
    Eigen::Matrix<double,8,1> motor_torque;
    Eigen::Matrix<double,14,1>state = obser.get_stance_state();
    Eigen::Matrix<double,14,1>balance_state;
    Eigen::Matrix<double,8,1> u,balance_u;

    double des_wz = -0.1*ratio.left_h;
    double des_z = 0.35+0.1*ratio.right_knob;
    // ROS_INFO_STREAM_THROTTLE(1,"des_z: " << des_z);
    balance_state << 0,0,0,des_z,0,0,0,0,0,0,dx_balance,0,dx_balance,0;
    double open_force = ratio1*(obser.rl(1)-obser.rr(1) - param.robot.wheel_distance);
    balance_u << 0,open_force-36*param.robot.L3 / 0.42,36,0,0,-open_force+36*param.robot.L3 / 0.42,36,0;
  
    vx_inte_err = (0 - obser.p_w(0));
    vx_inte_err = clamp(vx_inte_err,1.5,-1.5);
    yaw_rate_inte_err += Ts*(des_wz - state(9));
    yaw_rate_inte_err = clamp(yaw_rate_inte_err,1.5,-1.5);
    height_inte_err += Ts*(des_z - state(3));
    height_inte_err = clamp(height_inte_err,0.3,-0.3);

    u = Kc_DOUBLE*Eigen::Vector3d(vx_inte_err,yaw_rate_inte_err,height_inte_err)
     - K_DOUBLE*(state - balance_state)
     + balance_u;
    
    // u(3) = dead_zone_compensate(u(3),ratio3);
    // u(7) = dead_zone_compensate(u(7),ratio3);
    //force clamp
    u(0) = clamp(u(0),13,-13);
    u(1) = clamp(u(1),15,-15);
    u(2) = clamp(u(2),80,10);
    u(3) = clamp(u(3),4,-4);
    u(4) = clamp(u(4),13,-13);
    u(5) = clamp(u(5),15,-15);
    u(6) = clamp(u(6),80,10);
    u(7) = clamp(u(7),4,-4);
    update_wrench(u);
    Eigen::Vector3d left_leg_torque = obser.left_Jp.transpose()*obser.rotation_rp.transpose()*Eigen::Vector3d(-u(0),-u(1),-u(2));
    Eigen::Vector3d right_leg_torque = obser.right_Jp.transpose()*obser.rotation_rp.transpose()*Eigen::Vector3d(-u(4),-u(5),-u(6));
    left_leg_torque += obser.left_Jr.transpose()*obser.rotation_rp.transpose()*Eigen::Vector3d(0,-u(3),0);
    right_leg_torque += obser.right_Jr.transpose()*obser.rotation_rp.transpose()*Eigen::Vector3d(0,-u(7),0);    
    motor_torque << left_leg_torque,-u(3),right_leg_torque,-u(7);


    debug_msg.data.push_back(motor_torque(0));
    debug_msg.data.push_back(motor_torque(1));
    debug_msg.data.push_back(motor_torque(2));
    debug_msg.data.push_back(motor_torque(3));
    debug_msg.data.push_back(motor_torque(4));
    debug_msg.data.push_back(motor_torque(5));
    debug_msg.data.push_back(motor_torque(6));
    debug_msg.data.push_back(motor_torque(7));

    debug_msg.data.push_back(0.0);
    debug_msg.data.push_back(des_wz);
    debug_msg.data.push_back(des_z);
    
    return motor_torque;
}

// mpc control
Eigen::Matrix<double,8,1> Controller::mpc_calc(Observer & obser,const Ratio & ratio,Regulator & regulator,std_msgs::Float64MultiArray & debug_msg){

    static double open_force = 0;

    static Eigen::Matrix<double,17,1>state,last_state;
    static Eigen::Matrix<double,17,1> ratio_goal_state;
    static Eigen::Matrix<double,17,1>balance_state;
    static Eigen::Matrix<double,-1,1> u;
    static Eigen::Matrix<double,8,1> balance_u;
    static Eigen::Matrix<double,-1,1> predict_x_horizon;
    static Eigen::MatrixXd des_x_horizon;
    static int  last_ratio_D = 0;

    state = obser.get_mpc_state();
    // balance_state << 0,0,state(2),0,0,clamp(state(5),0.5,0.2),0,0,0,0,0,state(11),state(12),0,0,-0,0;
    balance_state << 0,0,0,0,0,state(5),0,0,0,0,0,state(11),0,0,0,-0,0;
    open_force = 0.1*(ratio1*(obser.rl(1)-obser.rr(1) - param.robot.wheel_distance))+0.9*open_force;
    balance_u << 0,open_force-42*param.robot.L3 / 0.42,42,0,0,-open_force+42*param.robot.L3 / 0.42,42,0;
    

    if(ratio.D > -1){
        if(!regulator.isRunning || (ros::Time::now()-last_position_calc_time).toSec() > 0.5){
            regulator.start(obser, 30);
        }
        des_x_horizon = _ocp.setTrajXdqp(balance_state,regulator);
        
    }
    else if((ros::Time::now()-last_position_calc_time).toSec() > 0.5 || last_ratio_D > -1){
        regulator.reset();
        regulator.setRef(state(0),state(1),state(5),state(11));
        
        ratio_goal_state(0) = state(0);
        ratio_goal_state(1) = state(1);
        ratio_goal_state(5) = 0.42;
        ratio_goal_state(11) = state(11);
        // ROS_INFO_STREAM("des x:"<<state(0)<<"des y:"<<state(1)<<"des yaw"<<state(11));
        des_x_horizon = _ocp.setTrajXdqp(balance_state,ratio,ratio_goal_state);
    }
    else{
        des_x_horizon = _ocp.setTrajXdqp(balance_state,ratio,ratio_goal_state);
    }
    last_ratio_D = ratio.D;
    last_position_calc_time = ros::Time::now();
    _ocp.setStateMatrix(obser,1/param.mpc_freq_max,balance_state,balance_u);
    _ocp.solve_mpc(state,balance_state,regulator,&u,&predict_x_horizon,false);
    debug_msg.data.push_back(u(0));
    debug_msg.data.push_back(u(1));
    debug_msg.data.push_back(u(2));
    debug_msg.data.push_back(u(3));
    debug_msg.data.push_back(u(4));
    debug_msg.data.push_back(u(5));
    debug_msg.data.push_back(u(6));
    debug_msg.data.push_back(u(7));
    u += balance_u;
    // u(3) = dead_zone_pwm(u(3),ratio3);
    // u(7) = dead_zone_pwm(u(7),ratio3);
    //force clamp
    u(0) = clamp(u(0),13,-13);
    u(1) = clamp(u(1),15,-15);
    u(2) = clamp(u(2),80,10);
    u(3) = clamp(u(3),8,-8);
    u(4) = clamp(u(4),13,-13);
    u(5) = clamp(u(5),15,-15);
    u(6) = clamp(u(6),80,10);
    u(7) = clamp(u(7),8,-8);
    update_wrench(u);

    publish_traj(predict_x_horizon,mpc_path_pub);
    publish_traj(des_x_horizon,mpc_desire_pub);
    // debug_msg.data.push_back(des_vx);
    // debug_msg.data.push_back(des_wz);
    // debug_msg.data.push_back(des_y);
    // debug_msg.data.push_back(des_z);
    // debug_msg.data.push_back(des_pitch);

    return u;
}

// lqr control
Eigen::Matrix<double,8,1> Controller::lqr_calc(Observer & obser,const Ratio & ratio,Regulator & regulator,std_msgs::Float64MultiArray& debug_msg){
    static double des_vx = 0;
    static double des_wz = 0;
    static double des_z = 0.42;
    static double des_y = 0;
    static double des_pitch = 0;
    static double open_force = 0;
    static Eigen::Matrix<double,8,1> motor_torque;
    static Eigen::Matrix<double,17,1> state;
    static Eigen::Matrix<double,15,1>balance_state;
    static Eigen::Matrix<double,8,1> u,balance_u;
    static Eigen::Matrix<double,17,1> des_regulate_state;
    static int  last_ratio_D = 0;

    state = obser.get_mpc_state();

    if(ratio.D > -1){
        if(!regulator.isRunning || (ros::Time::now()-last_lqr_calc_time).toSec() > 0.5){
            regulator.start(obser, 30);
        }
        des_regulate_state = regulator.get_des_state(ros::Time::now());

    }
    else if((ros::Time::now()-last_lqr_calc_time).toSec() > 0.5 || last_ratio_D > -1){
        regulator.reset();
        regulator.setRef(state(0),state(1),state(5),state(11));
        des_regulate_state(0) = state(0);
        des_regulate_state(1) = state(1);
        des_regulate_state(5) = 0.4;
        des_regulate_state(11) = state(11);
    }
    else{
        
    }
    last_ratio_D = ratio.D;
    last_lqr_calc_time = ros::Time::now();
    
    
    Eigen::Vector3d pos_error,yaw_dir;
    yaw_dir <<cos(state(11)),sin(state(11)),0;
    pos_error << des_regulate_state(0)-state(0),des_regulate_state(1)-state(1),0;
    Eigen::Vector3d pos_error_in_body_frame = obser.rotation_y.transpose() * pos_error;
    Eigen::MatrixXd cosin = (yaw_dir.transpose() * pos_error);
    int sign = cosin(0)> 0 ? 1 : -1;
    des_vx = 1.0*sign*sqrt(pos_error.squaredNorm());
    des_vx = clamp(des_vx,1,-1);

    balance_state << des_vx,des_y,0,des_z,0,0,0,des_pitch,0,des_regulate_state(11),0,dx_balance,0,dx_balance,0;
    open_force = 0.1*(ratio1*(obser.rl(1)-obser.rr(1) - param.robot.wheel_distance))+0.9*open_force;
    balance_u << 0,open_force-42*param.robot.L3 / 0.42,42,0,0,-open_force+42*param.robot.L3 / 0.42,42,0;
    // balance_u << 0,0.5*param.robot.mass*state(2)*state(12),36,0,0,0.5*param.robot.mass*state(2)*state(12),36,0;



    
    Eigen::MatrixXd error_state = state.block(2,0,15,1) - balance_state;
    error_state(9) = clamp(yaw_error(state(11),des_regulate_state(11)),0.45,-0.45) - clamp(4*pos_error_in_body_frame(1),0.3,-0.3);
    // error_state(10) = 0;
    for(int i=0;i<15;i++){
        debug_msg.data.push_back(error_state(i));
    }
    debug_msg.data.push_back(des_regulate_state(0));
    debug_msg.data.push_back(des_regulate_state(1));
    // Eigen::IOFormat CleanFmt(1, 0, ", ", "\n", "[", "]");
    // ROS_INFO_STREAM_THROTTLE(1,"balance_state: "<<balance_state.transpose().format(CleanFmt));
    // ROS_INFO_STREAM_THROTTLE(1,"error_state: "<<error_state.transpose().format(CleanFmt));
    u = - KK*error_state + balance_u;
    
    // u(3) = dead_zone_pwm(u(3),ratio3);
    // u(7) = dead_zone_pwm(u(7),ratio3);
    //force clamp
    u(0) = clamp(u(0),13,-13);
    u(1) = clamp(u(1),15,-15);
    u(2) = clamp(u(2),80,10);
    u(3) = clamp(u(3),8,-8);
    u(4) = clamp(u(4),13,-13);
    u(5) = clamp(u(5),15,-15);
    u(6) = clamp(u(6),80,10);
    u(7) = clamp(u(7),8,-8);
    update_wrench(u);
    Eigen::Vector3d left_leg_torque = obser.left_Jp.transpose()*obser.rotation_rp.transpose()*Eigen::Vector3d(-u(0),-u(1),-u(2));
    Eigen::Vector3d right_leg_torque = obser.right_Jp.transpose()*obser.rotation_rp.transpose()*Eigen::Vector3d(-u(4),-u(5),-u(6));
    left_leg_torque += obser.left_Jr.transpose()*obser.rotation_rp.transpose()*Eigen::Vector3d(0,-u(3),0);
    right_leg_torque += obser.right_Jr.transpose()*obser.rotation_rp.transpose()*Eigen::Vector3d(0,-u(7),0);    
    motor_torque << left_leg_torque,-u(3),right_leg_torque,-u(7);    
    return motor_torque;
}



void Controller::update_wrench(Eigen::Matrix<double , 8 ,1> u){
    ros::Time now_time = ros::Time::now();
    left_wrench.header.stamp = now_time; 
    left_wrench.header.frame_id = "left_contact";
    left_wrench.wrench.force.x = u(0);
    left_wrench.wrench.force.y = u(1);
    left_wrench.wrench.force.z = u(2);
    left_wrench.wrench.torque.y = u(3);

    right_wrench.header.stamp = now_time;
    right_wrench.header.frame_id = "right_contact";
    right_wrench.wrench.force.x = u(4);
    right_wrench.wrench.force.y = u(5);
    right_wrench.wrench.force.z = u(6);
    right_wrench.wrench.torque.y = u(7);
}
// later move to OD_regulater
double Controller::cal_des_roll(double des_vx,double des_wz,double des_height){
    double des_roll = 0;
    double l_v0 = des_vx / (des_wz + 1e-5);
    if (abs(l_v0)>param.robot.wheel_distance / 2){
        des_roll = clamp(atan(-des_height/l_v0),0.25,-0.25);
    }
    else{
        des_roll =   - 0.25 / (param.robot.wheel_distance / 2) * l_v0;
    }
    return des_roll;
}
void Controller::lqr_reconfigureCallback(wheel_leg_fsm::LqrConfig& config, uint32_t level){
    if(_lqr_first_config){
        _lqr_first_config = false;
        return;
    }
    Eigen::DiagonalMatrix<double,17> Q_diag;
    Eigen::DiagonalMatrix<double,8> R_diag;
    Q_diag.diagonal() << config.Q_xv, 
                        config.Q_y, 1, 
                        config.Q_z, 1, 
                        config.Q_roll, 1, 
                        config.Q_pitch, 1, 
                        config.Q_yaw, 1,
                        config.Q_dx, 1, config.Q_dx, 1,
                        config.Q_init_xv,config.Q_init_yawr,config.Q_init_z; 


    R_diag.diagonal() << config.R_N, config.R_Q, config.R_P, config.R_tao, 
                         config.R_N, config.R_Q, config.R_P, config.R_tao;
    _ocp.setWeightLQR(Q_diag,R_diag);
    /*----------------- LQR -------------------*/
    ROS_INFO_STREAM("********** reconfigure lqr weight setup ***************");
    Eigen::MatrixXd K;
    if(_ocp.dlqr_calc(&K,1e-7)){
        K(1,2) *= config.y_d_ratio;
        K(2,2) *= config.y_d_ratio;
        K(5,2) *= config.y_d_ratio;
        K(6,2) *= config.y_d_ratio;

        K(2,4) *= config.z_d_ratio;
        K(6,4) *= config.z_d_ratio;

        K(1,6) *= config.roll_d_ratio;
        K(2,6) *= config.roll_d_ratio;
        K(5,6) *= config.roll_d_ratio;
        K(6,6) *= config.roll_d_ratio;

        K(0,8) *= config.pitch_d_ratio;
        K(3,8) *= config.pitch_d_ratio;
        K(4,8) *= config.pitch_d_ratio;
        K(7,8) *= config.pitch_d_ratio;

        K(0,11) *= config.dx_d_ratio;
        K(0,13) *= config.dx_d_ratio;
        K(3,11) *= config.dx_d_ratio;
        K(3,13) *= config.dx_d_ratio;
        K(4,11) *= config.dx_d_ratio;
        K(4,13) *= config.dx_d_ratio;
        K(7,11) *= config.dx_d_ratio;
        K(7,13) *= config.dx_d_ratio;

        K_mutex.lock();
        K_DOUBLE << K.block<8,14>(0,0);
        Kc_DOUBLE << -K.block<8,3>(0,14);
        K_mutex.unlock();
    }
    else{
        ROS_ERROR_STREAM("LQR calc failed !");
    }
    Eigen::IOFormat fmt(5, 0, ",", "\n", "", ",");
    ROS_INFO_STREAM("K:\n"<< K_DOUBLE.format(fmt) << std::endl);
    ROS_INFO_STREAM("Kc:\n"<< Kc_DOUBLE.format(fmt) << std::endl);
    // ROS_INFO_STREAM("K:\n"<< K.format(fmt) << std::endl);
    /*----------------- LQR -------------------*/

}
void Controller::mpc_reconfigureCallback(wheel_leg_fsm::MPCConfig& config, uint32_t level){
    if(_lqr_first_config){
        _lqr_first_config = false;
        return;
    }
    Eigen::DiagonalMatrix<double,17> Q_diag;
    Eigen::DiagonalMatrix<double,8> R_diag;
    Q_diag.diagonal() << config.Q_inertial_x,config.Q_inertial_y,
                        config.Q_xv, 
                        config.Q_y, 1, 
                        config.Q_z, 1, 
                        config.Q_roll, 1, 
                        config.Q_pitch, 1, 
                        config.Q_yaw, 1,
                        config.Q_dx, 1, config.Q_dx, 1; 


    R_diag.diagonal() << config.R_N, config.R_Q, config.R_P, config.R_tao, 
                         config.R_N, config.R_Q, config.R_P, config.R_tao;
    _ocp.setWeightMPC(Q_diag,R_diag);
    
    ROS_INFO_STREAM("MPC reconfigure done");


}

void Controller::manual_reconfigureCallback(wheel_leg_fsm::ManualConfig& config, uint32_t level){
    if(_manual_first_config){
        _manual_first_config = false;
        return;
    }
    output_ratio = clamp(config.OUTPUT_ratio,1,0);
    dx_balance = config.dx_balance;
    ratio1 = config.ratio1;
    ratio2 = config.ratio2;
    ratio3 = config.ratio3;
    // ROS_INFO_STREAM("output_ratio: "<<output_ratio);
}

void Controller::reset_intergrator(){
    _velocity_pid.reset();
    _yaw_pid.reset();
    _angle_pid.reset();
    _angular_pid.reset();
    height_inte_err = 0;
    vx_inte_err = 0;
    yaw_rate_inte_err = 0;
}

void Controller::publish_traj(Eigen::MatrixXd traj_vector,ros::Publisher publisher){
    nav_msgs::Path mpc_path;
    for(int i=0;i<param.mpc_horizon;i+=5){
        Eigen::Matrix<double,17,1> predict_state = traj_vector.block(i*17,0,17,1);
        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.header.frame_id ="world";
        pose_stamp.pose.position.x = predict_state(0);
        pose_stamp.pose.position.y = predict_state(1);
        pose_stamp.pose.position.z = predict_state(5);
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(predict_state(7),Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(predict_state(9),Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(predict_state(11),Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond quaternion = yawAngle*pitchAngle*rollAngle;
        pose_stamp.pose.orientation.x = quaternion.x();
        pose_stamp.pose.orientation.y = quaternion.y();
        pose_stamp.pose.orientation.z = quaternion.z();
        pose_stamp.pose.orientation.w = quaternion.w();
        mpc_path.poses.push_back(pose_stamp);
    }
    mpc_path.header.frame_id = "world";
    mpc_path.header.stamp = ros::Time::now();
    publisher.publish(mpc_path);

}

double dead_zone_compensate(double raw_value,double dead_zone){
    if(raw_value > 0.001){
        return raw_value + dead_zone;
    }
    else if(raw_value < -0.001){
        return raw_value - dead_zone;
    }
    else{
        return raw_value;
    }
}

double dead_zone_pwm(double raw_value,double dead_zone){
    if(raw_value > 0 && raw_value < dead_zone){
        return dead_zone;
    }
    else if(raw_value < 0 && raw_value > -dead_zone){
        return -dead_zone;
    }
    else{
        return raw_value;
    }
}
// yaw1 - yaw2 continious
double yaw_error(double yaw1,double yaw2){
    Eigen::Matrix3d m1 , m2;
    m1 <<  cos(yaw1), -sin(yaw1) , 0 ,
            sin(yaw1), cos(yaw1) , 0 ,
            0,        0        , 1 ;
    m2 <<  cos(yaw2), -sin(yaw2) , 0 ,
            sin(yaw2), cos(yaw2) , 0 ,
            0,        0        , 1 ;
    m1 = m2.transpose()*m1;
    return atan2(m1(1,0), m1(0,0));
}
