#include "OD_Observer.h"

Eigen::Vector3d R2rpy(const Eigen::Matrix3d& R) {
    double y = atan2(R(1,0), R(0,0)); //[-pi, pi]
    double p = atan2(-R(2,0),sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0))); //[-pi/2 pi/2]
    double r = atan2(R(2,1), R(2,2)); //[-pi, pi]
    // ROS_INFO_STREAM_THROTTLE(0.5,y<<std::endl<<p<<std::endl<<r);
    return Eigen::Vector3d(r,p,y);
}

Observer::Observer(WLParam &param_) : param(param_)
{   
    ros::NodeHandle node_observe("fsm_observe");
    observe_config_server = new dynamic_reconfigure::Server<wheel_leg_fsm::ObserveConfig>(node_observe);
    observe_cbt = boost::bind(&Observer::observe_reconfigureCallback, this, _1, _2);
    observe_config_server->setCallback(observe_cbt);

    p_w = Eigen::Vector3d::Zero();
    last_p_w = Eigen::Vector3d::Zero();
    p_I = Eigen::Vector3d::Zero();
    v_d = Eigen::Vector3d::Zero();
    v_I = Eigen::Vector3d::Zero();
    v_d_estimated_by_leg = Eigen::Vector3d::Zero();
    v_d_ground_truth = Eigen::Vector3d::Zero();
    a_d = Eigen::Vector3d::Zero();
    a_I = Eigen::Vector3d::Zero();
    rl = Eigen::Vector3d::Zero();
    rlb = Eigen::Vector3d::Zero();
    Drl = Eigen::Vector3d::Zero();
    rr = Eigen::Vector3d::Zero();
    rrb = Eigen::Vector3d::Zero();
    Drr = Eigen::Vector3d::Zero();

    left_leg_angle = Eigen::Vector3d::Zero();
    left_leg_angular = Eigen::Vector3d::Zero();
    right_leg_angle = Eigen::Vector3d::Zero();
    right_leg_angular = Eigen::Vector3d::Zero();

    euler_rpy = Eigen::Vector3d::Zero();
    last_euler_rpy = Eigen::Vector3d::Zero();
    angular_xyz = Eigen::Vector3d::Zero();
    left_wheel_speed = 0;
    right_wheel_speed = 0;

    fall_flag = false;
    vicon_not_change = true;
    lpf_filter_a = 0.4;
    vicon_not_change = true;

    right_Jp = Eigen::Matrix3d::Identity();
    left_Jp = Eigen::Matrix3d::Identity();
    right_Jr = Eigen::Matrix3d::Identity();
    left_Jr = Eigen::Matrix3d::Identity();

    rotation_rp = Eigen::Matrix3d::Identity();
    rotation_y = Eigen::Matrix3d::Identity();

    input_m.setZero().diagonal() << 1.0/param.ctrl_freq_max,1.0/param.ctrl_freq_max,1.0/param.ctrl_freq_max;

    Q.setZero().diagonal() <<1e-3 ,1e-3, 1e-3;
    R.setZero().diagonal() <<0.7, 0.7, 0.7;
    E.setZero().diagonal() <<1e5, 1e5, 1e5;
    K = Eigen::Matrix3d::Identity();

    // std::vector<double> num = {0.0976,    0.1953,    0.0976}; // 2 order Fs 400hz Fc 50Hz
    // std::vector<double> den = {1.0000,   -0.9428,    0.3333};
    std::vector<double> num = {0.57919222,0.57919222}; // 1 order Fs 400hz Fc 40Hz
    std::vector<double> den = {1,0.15838444};
    pitch_rate_filter.init(1,den,num);
    roll_rate_filter.init(1,den,num);
    yaw_rate_filter.init(1,den,num);

    // ljy
    init_A();
    init_B();
    init_C();

    // 过程
    Q_inertial.setZero().diagonal() << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3;

    // 观测
    R_inertial.setZero().diagonal() << 8e-1, 8e-1, 8e-1, 8e-1, 8e-1, 8e-1, 1e1, 1e1, 1e1, 1e1, 1e1, 1e1, 5e2, 5e2, 5e2;
    // 状态
    E_inertial.setZero().diagonal() << 1e4, 1e4, 1e4, 1e4, 1e4, 1e4, 1e4, 1e4, 1e4, 1e4, 1e4, 1e4;
    K_inertial.setZero();
    
    // states init

    u.setZero();
    input.setZero();
    yk.setZero();
    yk_estimated_by_leg.setZero();
    xk.setZero();

    if(!param.isSim){
        rotation_imu_rigid = Eigen::AngleAxisd(-7 * M_PI / 180,Eigen::Vector3d::UnitZ()); // 7 deg from rigid to imu
    }
    else{
        rotation_imu_rigid.setIdentity();
    }
    pos_recv_time = ros::Time::now();
    p_I.setZero();
    v_I.setZero();
}
// ljy
void Observer::init_A()
{
    // I = I_3x3
    // A = [I Ts*I 0 0;
    //      0 I 0 0;
    //      0 0 I 0;
    //      0 0 0 I] 12*12
    A.setIdentity();
    A.block<3, 3>(0, 3) = input_m;
    // test
    // std::cout << "A = \n" << A << std::endl;
}

void Observer::init_B()
{
    // I = I_3x3
    // B = [0.5*Ts^2*I 0 0;
    //      Ts*I 0 0;
    //      0 Ts*I 0;
    //      0 0 Ts*I] 12*9
    B.setZero();
    B.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.5 * (1.0 / param.ctrl_freq_max) * (1.0 / param.ctrl_freq_max);
    B.block<3, 3>(3, 0) = input_m;
    B.block<3, 3>(6, 3) = input_m;
    B.block<3, 3>(9, 6) = input_m;
    // test
    // std::cout << "B = \n" << B << std::endl;
}

void Observer::init_C()
{
    // I = I_3x3
    // C = [-I 0 I 0;
    //      -I 0 0 I;
    //      0 I 0 0;
    //      0 I 0 0;
    //      I 0 0 0;] 15*12
    C.setZero();
    C.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    C.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
    C.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
    C.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    C.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();
    C.block<3, 3>(9, 3) = Eigen::Matrix3d::Identity();
    C.block<3, 3>(12,0) = Eigen::Matrix3d::Identity();

    // test
    // std::cout << "C = \n" << C << std::endl;
}

void Observer::vicon_update(geometry_msgs::PoseStampedConstPtr vicon_pose_ptr){
    static bool first_flag = true;
    static Eigen::Matrix4d vicon_first = Eigen::Matrix4d::Identity();
    if(first_flag){
        Eigen::Quaterniond q_vicon(vicon_pose_ptr->pose.orientation.w, vicon_pose_ptr->pose.orientation.x, vicon_pose_ptr->pose.orientation.y, vicon_pose_ptr->pose.orientation.z);
        q_vicon.normalize();
        vicon_first.block<3, 3>(0, 0) = rotation_imu_rigid *q_vicon.matrix();
        vicon_first.block<3, 1>(0, 3) = Eigen::Vector3d(vicon_pose_ptr->pose.position.x, vicon_pose_ptr->pose.position.y, 0);
        
        ROS_INFO_STREAM("vicon_first:"<<vicon_first);
        first_flag = false;
    }
    p_w =  (vicon_first.inverse() * Eigen::Vector4d(vicon_pose_ptr->pose.position.x,vicon_pose_ptr->pose.position.y,vicon_pose_ptr->pose.position.z,1)).block<3,1>(0,0);
    if(std::sqrt((p_w-last_p_w).squaredNorm())<1e-16){
        vicon_not_change = true;
        // ROS_INFO_STREAM_THROTTLE(1,std::endl<<p_w.transpose()<<std::endl<<last_p_w.transpose());
    }
    else{
        vicon_not_change = false;
    }
    last_p_w = p_w;
    // pos_recv_time = ros::Time::now();
}


Observer::~Observer()
{
}
void Observer::update(nav_msgs::OdometryConstPtr odom_ptr){
    Eigen::Vector3d v_w_ground_truth(odom_ptr->twist.twist.linear.x,odom_ptr->twist.twist.linear.y,odom_ptr->twist.twist.linear.z);
    v_d_ground_truth = rotation_y.transpose() * v_w_ground_truth;
    static bool first_flag = true;
    static Eigen::Matrix4d odom_first = Eigen::Matrix4d::Identity();
    if(first_flag){
        Eigen::Quaterniond q_vicon(odom_ptr->pose.pose.orientation.w, odom_ptr->pose.pose.orientation.x, odom_ptr->pose.pose.orientation.y, odom_ptr->pose.pose.orientation.z);
        q_vicon.normalize();
        odom_first.block<3, 3>(0, 0) = q_vicon.matrix();
        odom_first.block<3, 1>(0, 3) = Eigen::Vector3d(odom_ptr->pose.pose.position.x, odom_ptr->pose.pose.position.y, 0);
        
        ROS_INFO_STREAM("odom_first:"<<odom_first);
        first_flag = false;
    }
    p_w = (odom_first.inverse() * Eigen::Vector4d(odom_ptr->pose.pose.position.x,odom_ptr->pose.pose.position.y,odom_ptr->pose.pose.position.z,1)).block<3,1>(0,0);
    pos_recv_time = ros::Time::now();
    // ROS_INFO_STREAM_THROTTLE(1,"ground_truth received!");
}
void Observer::vio_update(nav_msgs::OdometryConstPtr odom_ptr){
    p_w(0) = odom_ptr->pose.pose.position.x;
    p_w(1) = odom_ptr->pose.pose.position.y;
    p_w(2) = odom_ptr->pose.pose.position.z;
    pos_recv_time = ros::Time::now();
}
// get imu data to update the observer
void Observer::update(sensor_msgs::ImuConstPtr imu_ptr){
    static bool first_flag = true;
    static Eigen::Quaterniond imu_first;

    Eigen::Quaterniond q_raw(imu_ptr->orientation.w,imu_ptr->orientation.x,imu_ptr->orientation.y,imu_ptr->orientation.z);
    if(first_flag && imu_ptr->orientation.w!=1.0){
        imu_first = q_raw;
        imu_first.x() = 0;
        imu_first.y() = 0;
        imu_first.normalize();
        ROS_INFO_STREAM("imu_first:"<< imu_first.w()<<","<< imu_first.x()<<","<< imu_first.y()<<","<< imu_first.z() <<"yaw:"<<R2rpy(imu_first.matrix())[2]);
        first_flag = false;
    }
    quaternion = imu_first.inverse() * q_raw;
    if(param.isSim){
        euler_rpy = R2rpy(quaternion.matrix());
        angular_xyz(0) = imu_ptr->angular_velocity.x;
        angular_xyz(1) = imu_ptr->angular_velocity.y;
        angular_xyz(2) = imu_ptr->angular_velocity.z;
    }
    else{
        euler_rpy = 0.8 * R2rpy(quaternion.matrix()) + 0.2 * euler_rpy;
        angular_xyz(0) = roll_rate_filter.filtrate(imu_ptr->angular_velocity.x);
        angular_xyz(1) = pitch_rate_filter.filtrate(imu_ptr->angular_velocity.y);
        angular_xyz(2) = yaw_rate_filter.filtrate(imu_ptr->angular_velocity.z);
    }

    last_euler_rpy = euler_rpy;
    rotation_rp = Eigen::AngleAxisd(euler_rpy(1),Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(euler_rpy(0),Eigen::Vector3d::UnitX());
    rotation_y = Eigen::AngleAxisd(euler_rpy(2),Eigen::Vector3d::UnitZ());
    a_d = rotation_rp * Eigen::Vector3d(imu_ptr->linear_acceleration.x,imu_ptr->linear_acceleration.y,imu_ptr->linear_acceleration.z)+Eigen::Vector3d(0,0,-param.g);
    a_I = quaternion.matrix() * Eigen::Vector3d(imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y,imu_ptr->linear_acceleration.z) + Eigen::Vector3d(0, 0, -param.g);
    //fall flag update
    if (abs(euler_rpy(0)) >  1.0 || abs(euler_rpy(1)) > 1.0)
    {
        fall_flag = true;
    }
    else{
        fall_flag = false;
    }
    
}
// get motor data to update the observer
void Observer::update(const OD_motor & motor){
    if(motor.side_name=="left"){
        if(motor.can_id==1){
            left_leg_angle(0)=motor.position;
            left_leg_angular(0)=motor.velocity;
        }
        else if(motor.can_id==2){
            left_leg_angle(1)=motor.position;
            left_leg_angular(1)=motor.velocity;
        }
        else if(motor.can_id==3){
            left_leg_angle(2)=motor.position;
            left_leg_angular(2)=motor.velocity;            
        }
        else if(motor.can_id==4){
            left_wheel_speed = motor.velocity;
        }
        else{

        }                
    }
    else if(motor.side_name=="right"){
        if(motor.can_id==1){
            right_leg_angle(0) = motor.position;
            right_leg_angular(0) = motor.velocity;
        }
        else if(motor.can_id==2){
            right_leg_angle(1) = motor.position;
            right_leg_angular(1) = motor.velocity;
        }
        else if(motor.can_id==3){
            right_leg_angle(2) = motor.position;
            right_leg_angular(2) = motor.velocity;           
        }
        else if(motor.can_id==4){
            right_wheel_speed = motor.velocity;
        }
        else{
            
        } 
    }
    else{
        return;
    }
}
// use all updated data to observe the state
void Observer::step(){
    right_Jp = Kinematics::Jp(right_leg_angle,param.robot.L1,param.robot.L2,-param.robot.L3);
    right_Jr = Kinematics::Jr(right_leg_angle);
    left_Jp = Kinematics::Jp(left_leg_angle,param.robot.L1,param.robot.L2,param.robot.L3);
    left_Jr = Kinematics::Jr(left_leg_angle);

    rlb = Kinematics::k(left_leg_angle,param.robot.L1,param.robot.L2,param.robot.L3) + Eigen::Vector3d(0,param.robot.shoulder_distacne / 2,0);
    Drl = rotation_rp * left_Jp * left_leg_angular + rotation_rp * Eigen::Vector3d(angular_xyz(0),angular_xyz(1),0).cross(rlb) ;
    rrb = Kinematics::k(right_leg_angle,param.robot.L1,param.robot.L2,-param.robot.L3) + Eigen::Vector3d(0,-param.robot.shoulder_distacne / 2,0);
    Drr = rotation_rp * right_Jp * right_leg_angular+ rotation_rp * Eigen::Vector3d(angular_xyz(0),angular_xyz(1),0).cross(rrb) ;
    rl = rotation_rp * rlb;
    rr = rotation_rp * rrb;

    // velocity kalman filter
    
    v_d_estimated_by_leg  = -0.5*(Eigen::Vector3d(0,0,angular_xyz(2)).cross(rl) + Drl + Eigen::Vector3d(-left_wheel_speed*param.robot.wheel_radius*0.86,0,0))
    -0.5*(Eigen::Vector3d(0,0,angular_xyz(2)).cross(rr) + Drr + Eigen::Vector3d(-right_wheel_speed*param.robot.wheel_radius*0.86,0,0));
    v_d = v_d + input_m*a_d;
    E = E + Q;
    
    K = E*(E+R).inverse();
    v_d = v_d  + K*(v_d_estimated_by_leg-v_d);
    E = (Eigen::Matrix3d::Identity()-K)*E;
}

// 2023-03-24 kalman filter in inertial frame
void Observer::inertial_step()
{
    right_Jp = Kinematics::Jp(right_leg_angle,param.robot.L1,param.robot.L2,-param.robot.L3);
    right_Jr = Kinematics::Jr(right_leg_angle);
    left_Jp = Kinematics::Jp(left_leg_angle,param.robot.L1,param.robot.L2,param.robot.L3);
    left_Jr = Kinematics::Jr(left_leg_angle);

    rlb = Kinematics::k(left_leg_angle,param.robot.L1,param.robot.L2,param.robot.L3) + Eigen::Vector3d(0,param.robot.shoulder_distacne / 2,0);
    Drl = rotation_rp * left_Jp * left_leg_angular + rotation_rp * Eigen::Vector3d(angular_xyz(0),angular_xyz(1),0).cross(rlb) ;
    rrb = Kinematics::k(right_leg_angle,param.robot.L1,param.robot.L2,-param.robot.L3) + Eigen::Vector3d(0,-param.robot.shoulder_distacne / 2,0);
    Drr = rotation_rp * right_Jp * right_leg_angular+ rotation_rp * Eigen::Vector3d(angular_xyz(0),angular_xyz(1),0).cross(rrb) ;
    rl = rotation_rp * rlb;
    rr = rotation_rp * rrb;
    // right_Jp = Kinematics::Jp(right_leg_angle, param.robot.L1, param.robot.L2, -param.robot.L3);
    // right_Jr = Kinematics::Jr(right_leg_angle);
    // left_Jp = Kinematics::Jp(left_leg_angle, param.robot.L1, param.robot.L2, param.robot.L3);
    // left_Jr = Kinematics::Jr(left_leg_angle);

    // Eigen::Vector3d rlb = Kinematics::k(left_leg_angle, param.robot.L1, param.robot.L2, param.robot.L3) +
    //                       Eigen::Vector3d(0, param.robot.shoulder_distacne / 2, 0);
    Eigen::Vector3d Drl_I = left_Jp * left_leg_angular;
    // Eigen::Vector3d rrb = Kinematics::k(right_leg_angle, param.robot.L1, param.robot.L2, -param.robot.L3) +
    //                       Eigen::Vector3d(0, -param.robot.shoulder_distacne / 2, 0);
    Eigen::Vector3d Drr_I = right_Jp * right_leg_angular;
    Eigen::Vector3d rl_I = quaternion.matrix() * rlb;
    Eigen::Vector3d rr_I = quaternion.matrix() * rrb;

    v_d_estimated_by_leg  = -0.5*(Eigen::Vector3d(0,0,angular_xyz(2)).cross(rl) + Drl + Eigen::Vector3d(-left_wheel_speed*param.robot.wheel_radius*0.86,0,0))
    -0.5*(Eigen::Vector3d(0,0,angular_xyz(2)).cross(rr) + Drr + Eigen::Vector3d(-right_wheel_speed*param.robot.wheel_radius*0.86,0,0));
    v_d = v_d + input_m*a_d;
    E = E + Q;
    
    K = E*(E+R).inverse();
    v_d = v_d  + K*(v_d_estimated_by_leg-v_d);
    E = (Eigen::Matrix3d::Identity()-K)*E;
    // if (flag_first == 0)
    // {
    //     flag_first = 1;
    //     xk.block<3, 1>(0, 0) = Eigen::Vector3d(0, 0, -rr(2) + param.robot.wheel_radius);
    //     xk.block<3, 1>(6, 0) = rl + xk.block<3, 1>(0, 0);
    //     xk.block<3, 1>(9, 0) = rr + xk.block<3, 1>(0, 0);

    //     std::cout << "rr = " << rr.transpose() << std::endl;
    //     std::cout << "rl = " << rl.transpose() << std::endl;
    //     std::cout << "xk = " << xk.transpose() << std::endl;

    //     std::cout << "first init xk, done !!!" << std::endl;

    //     rotation_camera2init = rotation_rp * rotation_y;
    //     std::cout << "rotation_camera2init = \n" << rotation_camera2init << std::endl;
    //     return;
    // }
    // std::cout << flag_first << std::endl;

    // fill the input
    input.block<3, 1>(0, 0) = a_I;

    input.block<3, 1>(3, 0) =
        rotation_y * (-Eigen::Vector3d(0, left_wheel_speed, 0).cross(Eigen::Vector3d(0, 0, -param.robot.wheel_radius)));
    input.block<3, 1>(6, 0) =
        rotation_y *
        (-Eigen::Vector3d(0, right_wheel_speed, 0).cross(Eigen::Vector3d(0, 0, -param.robot.wheel_radius)));
    // update the u
    u = B * input;
    // update the xk and E
    xk = A * xk + u;
    E_inertial = A * E_inertial * A.transpose() + Q_inertial;
    // update the yk
    yk = C * xk;
    // update the yk_estimated_by_leg

    yk_estimated_by_leg.block<3, 1>(0, 0) = rl_I;
    yk_estimated_by_leg.block<3, 1>(3, 0) = rr_I;
    yk_estimated_by_leg.block<3, 1>(6, 0) =
        -quaternion.matrix() *
        (angular_xyz.cross(rlb) + Drl_I +
         Eigen::Vector3d(0, left_wheel_speed, 0).cross(Eigen::Vector3d(0, 0, -param.robot.wheel_radius)));
    yk_estimated_by_leg.block<3, 1>(9, 0) =
        -quaternion.matrix() *
        (angular_xyz.cross(rrb) + Drr_I +
         Eigen::Vector3d(0, right_wheel_speed, 0).cross(Eigen::Vector3d(0, 0, -param.robot.wheel_radius)));

    // change according to the vicon:
    yk_estimated_by_leg.block<3, 1>(12, 0) << p_w;

    // update K

    if(vicon_not_change && (ros::Time::now()-pos_recv_time).toSec()> 0.05){
        K_inertial.block<12,12>(0,0) = E_inertial * C.block<12,12>(0,0).transpose() * (C.block<12,12>(0,0) * E_inertial * C.block<12,12>(0,0).transpose() + R_inertial.block<12,12>(0,0)).inverse();
    
        // update xk and E
        xk = xk + K_inertial.block<12,12>(0,0) * (yk_estimated_by_leg - yk).block<12,1>(0,0);
        E_inertial = (Eigen::Matrix<double, 12, 12>::Identity() - K_inertial.block<12,12>(0,0) * C.block<12,12>(0,0)) * E_inertial;
        ROS_WARN_STREAM_THROTTLE(1,"use imu! ");
    }
    else{
        K_inertial = E_inertial * C.transpose() * (C * E_inertial * C.transpose() + R_inertial).inverse();
    
        // update xk and E
        xk = xk + K_inertial * (yk_estimated_by_leg - yk);
        E_inertial = (Eigen::Matrix<double, 12, 12>::Identity() - K_inertial * C) * E_inertial;
        // ROS_INFO_THROTTLE(0.2,"use vicon!");
    }

    // fill the p_I v_I
    p_I = xk.block<3, 1>(0, 0);
    // v_I = xk.block<3, 1>(3, 0);
    v_I = rotation_y.inverse() * xk.block<3, 1>(3, 0);
    // std::cout << v_I.transpose() << std::endl;
}
Eigen::Matrix<double,14,1> Observer::get_stance_state() const{
    Eigen::Matrix<double,14,1> state;
    double z = ((param.robot.wheel_radius-rr(2)) + (param.robot.wheel_radius-rl(2)))/2;
    // double z = 0.5*(p_I(2)-xk(8)) + 0.5*(p_I(2)-xk(11));
    state << v_I(0),-0.5*(rl(1)+rr(1)),v_I(1),z,v_I(2),euler_rpy(0),angular_xyz(0),euler_rpy(1),angular_xyz(1),angular_xyz(2),rl(0),Drl(0),rr(0),Drr(0);
    return state;
}
Eigen::Matrix<double,17,1> Observer::get_mpc_state() const{
    Eigen::Matrix<double,17,1> state;
    double z = ((param.robot.wheel_radius-rr(2)) + (param.robot.wheel_radius-rl(2)))/2;
    state << p_I(0),p_I(1),v_I(0),-0.5*(rl(1)+rr(1)),v_I(1)-0.5*angular_xyz(2)*(rl(1)+rr(1))
    ,z,v_I(2),euler_rpy(0),angular_xyz(0),euler_rpy(1),angular_xyz(1),euler_rpy(2),angular_xyz(2),rl(0),Drl(0),rr(0),Drr(0);
    return state;
}

void Observer::observe_reconfigureCallback(wheel_leg_fsm::ObserveConfig& config, uint32_t level){
    lpf_filter_a = config.Lpf_param;
    // Q.diagonal() <<config.Kalman_Q, config.Kalman_Q, config.Kalman_Q;
    // R.diagonal() << config.Kalman_R, config.Kalman_R , config.Kalman_R;
    ROS_INFO_STREAM("observer_reconfigure; Lpf: "<< lpf_filter_a << std::endl <<" Q: \n"<<Q <<std::endl <<" R: \n"<<R);
}
