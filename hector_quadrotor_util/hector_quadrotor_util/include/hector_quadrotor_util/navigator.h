#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <vector>
#include <deque>
#include <iostream>
#include <fstream>
#include <thread>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>

#include <hector_quadrotor_util/joy_sim.h>
#include <hector_quadrotor_util/data_type.h>
#include <hector_quadrotor_util/PIDcontroller.h>
#include <hector_quadrotor_util/ctrl_interface.h>
#include <hector_quadrotor_util/my_data_type.hpp>

template <int M, int N>
using Matf = Eigen::Matrix<double, M, N>; //simplify Matrix type

class Navigator
{
private:
	Joysimer* joysimer;
    CtrlInterface* ctrlInterface;
    ros::Subscriber leeCmdSub, attSub, poseSub; //Sub att command R, current att, current pose
    std::vector<lee_cmd> att_cmd_list;
    PIDcontroller x_con, y_con, z_con;//(0, 0, 0);
    PIDcontroller roll_con, pitch_con, yaw_con, vz2z_con, thrust2vz_con;//(0, 0, 0);
    static double cur_vz;
    float con_roll = 0, con_pitch = 0, con_yawrate = 0, con_thrust = 0;

    enum CtrlState {
        VEL_CON = 0, //速度控制
        ATT_CON = 1,  //姿态控制
        HOVER = 2,  //结束后悬停
    };
    int ctrlState = CtrlState::VEL_CON;
    // int ctrlState = CtrlState::ATT_CON;

    static bool new_att, new_pose;

    std::vector<Eigen::Vector3d> RotationMatrix2euler(const std::vector<Mat3f> &_R); 
    void attCtrl(AttCmd &_att_cmd, std::ofstream &_outFile);

    PhysicsParm physicsParm;

    std::string traj_file_name = "/home/randy/test.bag";
    std::string traj_topic_name = "/test_primitive/lee_att_geo";
    static const uint traj_size;
    static bool is_reciving_traj;
    static bool position_callback_init_;

    static std::deque<TrajPoint> traj;
    TrajPoint cur_traj_point;
    static Eular cur_euler;
    static Pos cur_pose;

    static ros::Time cur_time;
    
    static void read_lee_cmd(const geometry_msgs::PoseWithCovariance::ConstPtr& msg);    //read lee traj topic from traj planning node
    void print_pose();

    static void attitude_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);  //get current att from ground truth
    static void position_callback(const nav_msgs::Odometry::ConstPtr& msg);

    void pid_init(ros::NodeHandle &nh);
    void set_next_trajpoint();

    bool check_arrival();
    bool check_arrival(const Pos &_cur_pos, const Pos &_sp_pos, double _threshold);
    bool check_arrival(const double &sp_z, const double &cur_z, double _threshold);

public:
    Navigator(ros::NodeHandle &nh)     {
        joysimer = new Joysimer(nh); 
        leeCmdSub = nh.subscribe("/test_primitive/lee_att_geo", 10, &Navigator::read_lee_cmd);
        attSub = nh.subscribe("/ground_truth_to_tf/euler", 10, &Navigator::attitude_callback);
        poseSub = nh.subscribe("/ground_truth/state", 10, &Navigator::position_callback);
        pid_init(nh);
        
    };

    Navigator(ros::NodeHandle &nh, Joysimer* _joysimer, CtrlInterface* _ctrlInterface)    {
        joysimer = _joysimer;
        ctrlInterface = _ctrlInterface;
        leeCmdSub = nh.subscribe("/test_primitive/lee_att_geo", 10, &Navigator::read_lee_cmd);
        attSub = nh.subscribe("/ground_truth_to_tf/euler", 10, &Navigator::attitude_callback);
        poseSub = nh.subscribe("/ground_truth/state", 10, &Navigator::position_callback);
        pid_init(nh);
    };



    void ctrlLoop();        //main conrtrol loop, execute cmd

};
