# OD-wheel-leg
用OD motor搭建的8自由度轮足机器人，实现多自由度运动控制与规划功能
## packages 说明
1. fdilink_ahrs 轮趣科技IMU(N100) 提供的ROS**结点**，目前工作频率200hz，话题名:"/imu"
2. filter 自编写的IIR滤波器实现**库**，利用给定的滤波器阶次与离散传递函数系数a，b实现IIR滤波
3. kinematics_3dof 自编写的3自由度腿部运动学**库**，目前实现了正运动学、逆运动学、速度雅可比与角速度雅可比
4. od_model 基于Gazebo的仿真**结点**，与机器人urdf模型描述，另有一些辅助结点模拟OD电机的CAN通讯协议
5. ros_canopen ROS提供的CAN通讯**结点**，利用socketCAN支持实现CAN与topic的双向桥接，对其中CAN套接字的初始化略作修改(socketcan_interfacce/include/socketcan.h:163)以避免在CAN总线上捕获过多错误帧，并只接收0-7id帧
6. sbus_serial ROS提供的SBUS解析**结点**，对原始的100kbps做修改以适配SBUS转USB模块的115200bps(sbus_serial/src/sbus_serial_driver.cpp:201)，结点话题名:"/sbus"
7. usb_can 自编写usb虚拟串口转can的**库**，但由于虚拟串口带宽受限且不能多程序同时访问等，目前已弃用
8. wheel_leg_fsm 腿轮机器人状态机控制规划**结点**，总体功能由多个库实现，主程序入口:FSM_node.cpp，具体各个模块在**FSM_node架构说明**介绍

## FSM_node架构说明
* FSM**类**:实现状态机大循环，类内包含Controller、Motor、Observer、Param、Ratio、Regulator的实例化对象
* Motor**类**:电机类，储存各电机的初始位置，滤波器系数，最大电流，极限位置，转向配置等，并可生成can_frame用于与CAN总线上的实物电机通讯(发送控制命令与解包电机信息)
* Ratio**类**:遥控类，存储当前各摇杆、拨杆值，进行死区、饱和等操作
* Observer**类**:观测器类，获取Motor编码器、IMU值，对机器人状态进行状态估计(速度、姿态、轮相对位置等)
* Regulator**类**:规划器类，根据控制目标(来自遥控Ratio或规划程序)，给出机器人状态的期望值
* Controller**类**:控制器类，根据机器人实际状态与期望状态的偏差，计算控制率，得出控制量，并进行必要的限幅
* Param**类**:参数类，上述所有类中均包含一个参数类的对象，用于公共参数获取，如机器人物理参数、控制频率、遥控器校准等

## 调参手册
### 待整定参数
1. Controller类中的反馈增益矩阵与积分增益矩阵，积分最大限幅，左右虚拟力限幅，平衡控制量balance_u
2. Motor类中的各电机最大电流限幅，速度低通滤波器系数
3. Observer类中的kalman滤波器，模型误差方差与测量误差方差
4. Ratio类中，拨杆偏移量、死区量等

### 整定顺序
1. 各电机零位、旋转方向、速度滤波器，在Rviz中反显
2. IMU 姿态解算、角速度方向
3. 机体速度kalman滤波器
4. rrx Drrx rlx Drlx 反馈系数
5. z vz iz反馈系数,平衡控制量balance_u
6. pitch wy 反馈系数
7. roll wx y vy 反馈系数
8. vx ivx wz iwz 反馈系数
9. wrench限幅与电流限幅

## 控制算法说明
### 动力学模型

### 线性化
系统状态 vx y vy z vz roll wx pitch wy wz rlx Drlx rrx Drrx 
### VMC+LQR

### VMC+MPC
