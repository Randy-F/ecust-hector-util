#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <vector>
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
    static bool new_att, new_pose;

    std::vector<Eigen::Vector3d> RotationMatrix2euler(const std::vector<Mat3f> &_R); 
    void attCtrl(AttCmd &_att_cmd, std::ofstream &_outFile);

    std::vector<double> sp_x = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
    std::vector<double> sp_y = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}; 
    std::vector<double> sp_z = {1.44, 2.04, 2.50, 2.89, 3.23, 3.54, 3.82, 4.08, 4.33, 4.56, 4.79, 5.00};
    // std::vector<double> sp_x = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
    // std::vector<double> sp_y = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4, 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 4.7, 4.8, 4.9, 5, 5.1, 5.2, 5.3, 5.4, 5.5, 5.6, 5.7, 5.8, 5.9, 6, 6.1, 6.2, 6.3, 6.4, 6.5, 6.6, 6.7, 6.8, 6.9, 7, 7.1, 7.2, 7.3, 7.4, 7.5, 7.6, 7.7, 7.8, 7.9, 8, 8.1, 8.2, 8.3, 8.4, 8.5, 8.6, 8.7, 8.8, 8.9, 9, 9.1, 9.2, 9.3, 9.4, 9.5, 9.6, 9.7, 9.8, 9.9, 10, 10.1, 10.2, 10.3, 10.4, 10.5, 10.6, 10.7, 10.8, 10.9, 11, 11.1, 11.2, 11.3, 11.4, 11.5, 11.6, 11.7, 11.8, 11.9, 12}; 
    // std::vector<double> sp_z = {0.00,  0.46,  0.65,  0.79,  0.91,  1.02,  1.12,  1.21,  1.29,  1.37,  1.44,  1.51,  1.58,  1.65,  1.71,  1.77,  1.83,  1.88,  1.94,  1.99,  2.04,  2.09,  2.14,  2.19,  2.24,  2.28,  2.33,  2.37,  2.42,  2.46,  2.50,  2.54,  2.58,  2.62,  2.66,  2.70,  2.74,  2.78,  2.81,  2.85,  2.89,  2.92,  2.96,  2.99,  3.03,  3.06,  3.10,  3.13,  3.16,  3.20,  3.23,  3.26,  3.29,  3.32,  3.35,  3.39,  3.42,  3.45,  3.48,  3.51,  3.54,  3.56,  3.59,  3.62,  3.65,  3.68,  3.71,  3.74,  3.76,  3.79,  3.82,  3.85,  3.87,  3.90,  3.93,  3.95,  3.98,  4.01,  4.03,  4.06,  4.08,  4.11,  4.13,  4.16,  4.18,  4.21,  4.23,  4.26,  4.28,  4.31,  4.33,  4.35,  4.38,  4.40,  4.43,  4.45,  4.47,  4.50,  4.52,  4.54,  4.56,  4.59,  4.61,  4.63,  4.65,  4.68,  4.70,  4.72,  4.74,  4.77,  4.79,  4.81,  4.83,  4.85,  4.87,  4.89,  4.92,  4.94,  4.96,  4.98,  5.00 };
    std::vector<double>::iterator it_x, it_y, it_z;
    uint index = 0;

    PhysicsParm physicsParm;

    std::string traj_file_name = "/home/randy/test.bag";
    std::string traj_topic_name = "/test_primitive/lee_att_geo";
    static const uint pose_array_size;
    static bool is_reciving_pose;
    
    static void read_lee_cmd(const geometry_msgs::PoseWithCovariance::ConstPtr& msg);    //read lee traj topic from traj planning node
    void print_pose();

    static void attitude_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);  //get current att from ground truth
    static void position_callback(const nav_msgs::Odometry::ConstPtr& msg);

    void pid_init(ros::NodeHandle &nh);
    void traj_init();

    bool check_arrival();

public:
    Navigator(ros::NodeHandle &nh)     {
        joysimer = new Joysimer(nh); 
        leeCmdSub = nh.subscribe("/test_primitive/lee_att_geo", 10, &Navigator::read_lee_cmd);
        attSub = nh.subscribe("/ground_truth_to_tf/euler", 10, &Navigator::attitude_callback);
        poseSub = nh.subscribe("/ground_truth/state", 10, &Navigator::position_callback);
        traj_init();
        pid_init(nh);
        
    };

    Navigator(ros::NodeHandle &nh, Joysimer* _joysimer, CtrlInterface* _ctrlInterface)    {
        joysimer = _joysimer;
        ctrlInterface = _ctrlInterface;
        leeCmdSub = nh.subscribe("/test_primitive/lee_att_geo", 10, &Navigator::read_lee_cmd);
        attSub = nh.subscribe("/ground_truth_to_tf/euler", 10, &Navigator::attitude_callback);
        poseSub = nh.subscribe("/ground_truth/state", 10, &Navigator::position_callback);
        traj_init();
        pid_init(nh);
    };

    static std::vector<Pose> pose_array;
    static Eular cur_euler;
    static Pos cur_pose;

    void ctrlLoop();        //main conrtrol loop, execute cmd

};
