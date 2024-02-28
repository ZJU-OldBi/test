//主程序入口
#include "FSM.h"
#include <ros/ros.h>
#include <thread>
#include <signal.h>
#include "Timer.hpp"
#include <sensor_msgs/JointState.h>

void mySigintHandler(int sig)
{
    ROS_INFO("[FSM] exit...");
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "wheel_leg_fsm");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(0.2).sleep();

    WLParam param;
    param.config_from_ros_handle(nh);

    Controller controller(param);
    Observer observer(param);
    Ratio ratio(param);
    Regulator regulator(param);
    // pid controller
    // pid_ns::PidObject velocity_pid(param.v_pid_param);
    // pid_ns::PidObject angle_pid(param.a_pid_param);
    // pid_ns::PidObject yaw_pid(param.y_pid_param);
 
    FSM fsm(param,controller,observer,ratio,regulator);
    //接受指令
    ros::Subscriber cmd_vel = nh.subscribe("/cmd_vel",10,&FSM::cmd_cvel,&fsm,ros::TransportHints().tcpNoDelay());
    //以上为调试区
    ros::Subscriber imu_sub = nh.subscribe("/imu",10,&FSM::imu_cb,&fsm,ros::TransportHints().tcpNoDelay());
    ros::Subscriber rc_sub = nh.subscribe("/sbus",10,&FSM::rc_cb,&fsm,ros::TransportHints().tcpNoDelay());
    ros::Subscriber left_can_sub = nh.subscribe("/left_can_msg",20,&FSM::left_can_cb,&fsm,ros::TransportHints().tcpNoDelay());
    ros::Subscriber right_can_sub = nh.subscribe("/right_can_msg",20,&FSM::right_can_cb,&fsm,ros::TransportHints().tcpNoDelay());
    ros::Subscriber debug_sub = nh.subscribe("/od/debug_sub",10,&FSM::debug_cb,&fsm);
    ros::Subscriber ground_truth_sub = nh.subscribe("/ground_truth/state",10,&FSM::ground_truth_cb,&fsm,ros::TransportHints().tcpNoDelay());
    ros::Subscriber vio_sub = nh.subscribe("/camera/odom/sample",10,&FSM::vio_cb,&fsm,ros::TransportHints().tcpNoDelay());
    ros::Subscriber vicon_sub = nh.subscribe("/vrpn_client_node/Rigid5/pose",10,&FSM::vicon_cb,&fsm,ros::TransportHints().tcpNoDelay());
    //新建Subscriber，设计该subscriber的回调函数，创建成员变量，在回调函数中保存指令，在控制函数中访问指令。
    //发出指令
    fsm.left_can_pub = nh.advertise<can_msgs::Frame>("/left_can_cmd",10);//给can总线发数据
    fsm.right_can_pub = nh.advertise<can_msgs::Frame>("/right_can_cmd",10);
    fsm.left_motor1_pub = nh.advertise<can_msgs::to_ros>("/od/left_motor1",10);
    fsm.left_motor2_pub = nh.advertise<can_msgs::to_ros>("/od/left_motor2",10);
    fsm.left_motor3_pub = nh.advertise<can_msgs::to_ros>("/od/left_motor3",10);
    fsm.left_motor4_pub = nh.advertise<can_msgs::to_ros>("/od/left_motor4",10);
    fsm.right_motor1_pub = nh.advertise<can_msgs::to_ros>("/od/right_motor1",10);
    fsm.right_motor2_pub = nh.advertise<can_msgs::to_ros>("/od/right_motor2",10);
    fsm.right_motor3_pub = nh.advertise<can_msgs::to_ros>("/od/right_motor3",10);
    fsm.right_motor4_pub = nh.advertise<can_msgs::to_ros>("/od/right_motor4",10);
    fsm.joint_state_pub = nh.advertise<sensor_msgs::JointState>("/od/joint_states",10);//发state的状态，用于可视化
    fsm.left_wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/od/left_wrench",10);//足端接触力
    fsm.right_wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/od/right_wrench",10);
    fsm.state_pub = nh.advertise<std_msgs::Float64MultiArray>("/od/state",10);//机器人的各种状态
    fsm.debug_pub = nh.advertise<std_msgs::Float64MultiArray>("/od/debug_pub",10);
    int cnt = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        if (fsm.rc_is_received(ros::Time::now()) && fsm.imu_is_received(ros::Time::now()))
        {
            ROS_INFO("[FSM] RC and imu received. State: OB");
            break;
        }
        else if(cnt++>20){
            ROS_WARN("no RC or imu received. Check again!");
            cnt = 0;
        }
        ros::Duration(0.1).sleep();
        
    }
    ROS_INFO_STREAM("Enter mpc thread" << std::endl);
    std::thread mpc_thread([&]() {
        ros::Rate r_main(param.mpc_freq_max); //40 hz
        while (ros::ok())
        {
            r_main.sleep();
            // ros::spinOnce();
            fsm.loop();

        }
        ROS_WARN("mpc thread exit!");
        std::terminate();
    });

    ROS_INFO_STREAM("Enter wbc update thread" << std::endl);
    std::thread wbc_thread([&]() {
        ros::Rate r_model(param.ctrl_freq_max); //400 hz
        while (ros::ok())
        {
            r_model.sleep();
            fsm.observer.inertial_step();
            // Timer init_time;
            
            
            fsm.wbc_calc();
            fsm.publish_state(500);
        }
        ROS_WARN("wbc thread exit!");
        std::terminate();
    });
    ros::AsyncSpinner spinner(4);
    spinner.start();


    mpc_thread.join();
    wbc_thread.join();
    return 0;
}