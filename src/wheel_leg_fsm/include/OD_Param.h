#pragma once
#include <ros/ros.h>
#include "../../balance_control_pkg/include/pid.h"


class WLParam
{
public:
	bool isSim;
	struct Robot
	{
		double mass;
		double wheel_distance;
		double shoulder_distacne;
		double wheel_radius;
		double L1,L2,L3;
	};
	struct MsgTimeout
	{
		double rc;
		double motor;
		double imu;
		double pos;
	};
    MsgTimeout msg_timeout;
	Robot robot;
    pid_ns::PID_param v_pid_param,a_pid_param,y_pid_param,r_pid_param;
	double ctrl_freq_max,mpc_freq_max;
	int mpc_horizon;
	double sw_low,sw_high;
	double max_velocity,max_yaw_rate;
	double g;
    WLParam(/* args */);
    ~WLParam();
    void config_from_ros_handle(const ros::NodeHandle &nh);

private:
	template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};  
};

