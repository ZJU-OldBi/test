#include "OD_Regulator.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
double clamp(double data,double upper,double lower);
Regulator::Regulator(WLParam &param_): param(param_)
{
    node_regulator = ros::NodeHandle("fsm_regulate");
    path_pub = node_regulator.advertise<nav_msgs::Path>("/des_path",10);
    regulator_config_server = new dynamic_reconfigure::Server<wheel_leg_fsm::RegulatorConfig>(node_regulator);
    regulator_cbt = boost::bind(&Regulator::regulator_reconfigureCallback, this, _1, _2);
    regulator_config_server->setCallback(regulator_cbt);
    isRunning = false;
    traj = IDLE;
    next_traj = RECT;
    start_pos.setZero();
    MAX_TIME = 0;
    start_yaw = 0;
}

Regulator::~Regulator()
{
}

void Regulator::start(Observer obser,double max_time){
    
    traj = next_traj;
    MAX_TIME = max_time;
    isRunning = true;
    start_time = ros::Time::now();
    start_pos = obser.p_I;
    start_yaw = obser.euler_rpy(2);
    nav_msgs::Path path;
    int i = 0;
    for(i = 0;i<4*MAX_TIME;i++){
        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.header.seq = i;
        Eigen::MatrixXd des_state = get_des_state(start_time + ros::Duration((double)i/4));
        pose_stamp.pose.position.x = des_state(0);
        pose_stamp.pose.position.y = des_state(1);
        pose_stamp.pose.position.z = des_state(5);
        double yaw = des_state(11);
        Eigen::Matrix3d des_rotaion;
        des_rotaion << cos(yaw), -sin(yaw) , 0 ,
                        sin(yaw), cos(yaw) , 0 ,
                        0,        0        , 1 ;
        Eigen::Quaterniond quaternion(des_rotaion);     
        pose_stamp.pose.orientation.x = quaternion.x();
        pose_stamp.pose.orientation.y = quaternion.y(); 
        pose_stamp.pose.orientation.z = quaternion.z();
        pose_stamp.pose.orientation.w = quaternion.w();      
        path.poses.push_back(pose_stamp);
    }
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();
    path_pub.publish(path);
    
    
}

void Regulator::reset(){
    traj = IDLE;
    isRunning = false;
}

void Regulator::setRef(double x,double y,double z,double yaw){
    start_pos << x,y,z;
    start_yaw = yaw;
}

Eigen::MatrixXd Regulator::get_des_state(ros::Time given_time) const{
    double dt = (given_time - start_time).toSec();
    Eigen::Matrix<double,17,1> des_state;
    if(isRunning){
        double T = MAX_TIME; //s
        double r = 0.75;
        const double pi = 3.14159;
        double cy = cos(start_yaw);
        double sy = sin(start_yaw);
        Eigen::Vector3d local_pos;
        Eigen::Vector3d local_vel;
        dt = clamp(dt,MAX_TIME,0);
        switch (traj)
        {
        case IDLE:
            des_state.setZero();
            des_state(5) = 0.4;
            break;
        case CIRCLE:
            local_pos << r*sin(2*pi*dt/T)   ,r - r*cos(2*pi*dt/T)   ,start_pos(2);
            local_vel << 2*pi/T*r*cos(2*pi*dt/T)    ,2*pi/T*r*sin(2*pi*dt/T),   0; 
            // des_state(0) = start_pos(0) + cy*local_pos(0)-sy*local_pos(1); // x pos
            // des_state(1) = cy*local_vel(0)-sy*local_vel(1); // x vel
            // des_state(2) = start_pos(1) + sy*local_pos(0)+cy*local_pos(1); // y pos
            // des_state(3) = sy*local_vel(0)+cy*local_vel(1); // y vel
            // des_state(4) = start_pos(2); // z pos
            // des_state(5) = 0; // z vel
            // des_state(6) = start_yaw + 2*pi*dt/T; // yaw angle
            // des_state(7) = 2*pi/T; // yaw rate
            // des_state(8) = 0; // rlx
            // des_state(9) = 0; // Drlx
            // des_state(10) = 0; //rrx
            // des_state(11) = 0; //Drrx

            des_state(0) = start_pos(0) + cy*local_pos(0)-sy*local_pos(1); // x pos
            des_state(1) = start_pos(1) + sy*local_pos(0)+cy*local_pos(1); // y pos
            des_state(2) = r*2*pi/T; // vbx
            des_state(3) = 0; // y
            des_state(4) = 0; // vy
            des_state(5) = 0.4; // bz
            des_state(6) = 0; // vbz
            des_state(7) = 0; // roll
            des_state(8) = 0; // wbx
            des_state(9) = 0; // pitch
            des_state(10) = 0; // wby
            des_state(11) = start_yaw + 2*pi*dt/T; // yaw angle
            des_state(12) = 2*pi/T; // wbz
            des_state(13) = 0; //rlx
            des_state(14) = 0; //Drlx
            des_state(15) = 0; // rrx
            des_state(16) = 0; // Drrx

            break;
        case SPACE_WALK:
            local_pos << -r*2*pi*dt/T   ,0   ,start_pos(2);
            local_vel << -r*2*pi/T      ,0   , 0;
            des_state(0) = start_pos(0) + cy*local_pos(0)-sy*local_pos(1); // x pos
            des_state(1) = start_pos(1) + sy*local_pos(0)+cy*local_pos(1); // y pos
            des_state(2) = -r*2*pi/T; // vbx
            des_state(3) = 0; // y
            des_state(4) = 0; // vy
            des_state(5) = 0.42; // bz
            des_state(6) = 0; // vbz
            des_state(7) = 0; // roll
            des_state(8) = 0; // wbx
            des_state(9) = 0; // pitch
            des_state(10) = 0; // wby
            des_state(11) = start_yaw; // yaw angle
            des_state(12) = 0; // wbz
            des_state(13) = 0.05*cos(2*pi*dt/2); //rlx
            des_state(14) = -0.05*2*pi/5*sin(2*pi*dt/2); //Drlx
            des_state(15) = -0.05*cos(2*pi*dt/2); // rrx
            des_state(16) = 0.05*2*pi/5*sin(2*pi*dt/2); // Drrx
            break;
        case SPIRAL:
            local_pos << r*2*pi*dt/T   ,0.07*sin(pi*dt)   ,start_pos(2);
            local_vel << r*2*pi/T      ,-0.07*pi*cos(pi*dt)   , 0;
            des_state(0) = start_pos(0) + cy*local_pos(0)-sy*local_pos(1); // x pos
            des_state(1) = start_pos(1) + sy*local_pos(0)+cy*local_pos(1); // y pos
            des_state(2) = r*2*pi/T; // vbx
            des_state(3) = 0.07*sin(pi*dt); // y
            des_state(4) = 0.07*pi*cos(pi*dt); // vy
            des_state(5) = 0.4 + 0.1*cos(pi*dt); // bz
            des_state(6) = -0.1*pi*sin(pi*dt); // vbz
            des_state(7) = 0; // roll
            des_state(8) = 0; // wbx
            des_state(9) = 0; // pitch
            des_state(10) = 0; // wby
            des_state(11) = start_yaw; // yaw angle
            des_state(12) = 0; // wbz
            des_state(13) = 0; //rlx
            des_state(14) = 0; //Drlx
            des_state(15) = 0; // rrx
            des_state(16) = 0; // Drrx
            break;
        case RECT:
            r*=2;
            if(dt <= 0.25*T){
                local_pos << r*dt/(0.25*T)   ,0   ,start_pos(2);
                local_vel << r/(0.25*T)      ,0   , 0;
                des_state(11) = start_yaw;
            }
            else if(dt <= 0.5*T){
                local_pos << r   ,r*(dt-0.25*T)/(0.25*T)   ,start_pos(2);
                local_vel << 0   ,r/(0.25*T)   , 0;
                des_state(11) = start_yaw + 0.5*pi;
            }
            else if(dt <= 0.75*T){
                local_pos << r-r*(dt-0.5*T)/(0.25*T)   ,r   ,start_pos(2);
                local_vel << -r/(0.25*T)                ,0   , 0;
                des_state(11) = start_yaw + pi;
            }
            else{
                local_pos << 0   ,r-r*(dt-0.75*T)/(0.25*T)   ,start_pos(2);
                local_vel << 0   ,-r/(0.25*T)   , 0;     
                des_state(11) = start_yaw + 1.5*pi;          
            }
            des_state(0) = start_pos(0) + cy*local_pos(0)-sy*local_pos(1); // x pos
            des_state(1) = start_pos(1) + sy*local_pos(0)+cy*local_pos(1); // y pos
            des_state(2) = 0*(cy*local_vel(0)-sy*local_vel(1)); // vbx
            des_state(3) = 0; // y
            des_state(4) = 0*(sy*local_vel(0)+cy*local_vel(1)); // vy
            des_state(5) = 0.4; // bz
            des_state(6) = 0; // vbz
            des_state(7) = 0; // roll
            des_state(8) = 0; // wbx
            des_state(9) = 0; // pitch
            des_state(10) = 0; // wby
            // des_state(11) = start_yaw; // yaw angle
            des_state(12) = 0; // wbz
            des_state(13) = 0; //rlx
            des_state(14) = 0; //Drlx
            des_state(15) = 0; // rrx
            des_state(16) = 0; // Drrx
            break;
        default:
            break;
        }
        
        
    }
    else{
        des_state.setZero();
        des_state(5) = 0.42;
    }
    return des_state;
}

void Regulator::regulator_reconfigureCallback(wheel_leg_fsm::RegulatorConfig& config, uint32_t level){
    next_traj = Traj_t(config.Traj_enum);
    switch(next_traj){
        case CIRCLE:
            ROS_INFO_STREAM("des trajectory changed to : CIRCLE");
            break;
        case SPIRAL:
            ROS_INFO_STREAM("des trajectory changed to : SPIRAL");
            break;
        case SPACE_WALK:
            ROS_INFO_STREAM("des trajectory changed to : SPACE_WALK");
            break;
        case RECT:
            ROS_INFO_STREAM("des trajectory changed to : RECT");
            break;
    }
}
    