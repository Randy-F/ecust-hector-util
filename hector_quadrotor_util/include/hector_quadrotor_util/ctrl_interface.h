#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <hector_uav_msgs/YawrateCommand.h>
#include <hector_uav_msgs/ThrustCommand.h>
#include <hector_uav_msgs/AttitudeCommand.h>
#include <hector_quadrotor_util/my_data_type.hpp>

class CtrlInterface
{
private:
	ros::Publisher attitude_publisher, yawrate_publisher, thrust_publisher;
    ros::Rate loop_rate = ros::Rate(100);
    AttCmd attCmd;
    bool msg_enable = false;

    // att msg
    std::string base_link_frame_ = "base_link", base_stabilized_frame_ = "base_stabilized";
    hector_uav_msgs::AttitudeCommand attitude;
    hector_uav_msgs::ThrustCommand thrust;
    hector_uav_msgs::YawrateCommand yawrate;

    inline void sendCtrlMsg(const AttCmd _att_cmd); //shloud be private!

public:
    CtrlInterface(ros::NodeHandle nh)
    {
        attitude_publisher = nh.advertise<hector_uav_msgs::AttitudeCommand>("/command/attitude", 10);
        yawrate_publisher = nh.advertise<hector_uav_msgs::YawrateCommand>("/command/yawrate", 10);
        thrust_publisher = nh.advertise<hector_uav_msgs::ThrustCommand>("/command/thrust",10); 

        attitude.header.frame_id = yawrate.header.frame_id = base_stabilized_frame_;
        thrust.header.frame_id = base_link_frame_; 
    }

    void testCtrlMsg(const AttCmd _att_cmd);
    void velCtrlStart();
    void velCtrl(double vx,double vy,double vz);

    void attCtrlStart();
    void setAtt(const AttCmd &_att_cmd);

    void msgLoop();
    void msgEnable(const bool _msg_enable);
    
};
