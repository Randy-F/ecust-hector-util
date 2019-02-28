#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <hector_quadrotor_util/my_data_type.hpp>

class Joysimer
{
private:
	ros::Publisher ctrlVelPub;
    ros::Rate loop_rate = ros::Rate(100);
    AttCmd attCmd;
    Vec3d velCmd;
    bool msg_enable = true;

    inline void sendJoyMsg(const double _val[5], const int &startButton);
    

public:
    Joysimer(ros::NodeHandle nh)
    {
        ctrlVelPub = nh.advertise<sensor_msgs::Joy>("/joy", 1000);  
    }

    void velCtrlStart();
    void velCtrl(double vx,double vy,double vz);
    void setVel(Vec3d &_vel_cmd);

    void attCtrlStart();
    void setAtt(AttCmd &_att_cmd);

    void msgLoop();
    void msgEnable(const bool _msg_enable);
    
};
