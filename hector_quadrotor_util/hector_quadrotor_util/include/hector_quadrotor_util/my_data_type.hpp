/**
 * @file data_type.h
 * @brief Defines all data types used in this lib

 * Mostly aliasing from Eigen Library.
 */
#pragma once

#include <stdio.h>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <hector_quadrotor_util/data_type.h>

#define PI 3.1415926535

class AttCmd    {     
public:
    double roll = 0;
    double pitch = 0;
    double yawrate = 0;
    double thrust = 0;

    AttCmd() {};
    ~AttCmd() {};
    AttCmd(const AttCmd &_att_cmd)    {
        roll = _att_cmd.roll;
        pitch = _att_cmd.pitch;
        yawrate = _att_cmd.yawrate;
        thrust = _att_cmd.thrust;
    }
    AttCmd(double _roll, double _pitch, double _yawrate, double _thrust): roll(_roll), pitch(_pitch), yawrate(_yawrate), thrust(_thrust) {};

    inline AttCmd operator/(double num)    {
        roll /= num;
        pitch /= num;
        yawrate /= num;

        return *this;
    }
};

class lee_cmd   {
public:
    Mat3f R;
    double thrust;

    lee_cmd(Mat3f _R, double _thrust): R(_R), thrust(_thrust) {};
};

class Eular {     
public:
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    Eular() {};
	~Eular() {};
	Eular(double _roll, double _pitch, double _yaw): roll(_roll), pitch(_pitch), yaw(_yaw) {};
};

class Vec3d {     
public:
    double x = 0;
    double y = 0;
    double z = 0;

    Vec3d() {};
	~Vec3d() {};
	Vec3d(double _x, double _y, double _z): x(_x), y(_y), z(_z) {};
};

class PhysicsParm   {
public:
    const double gravity = 9.81;
    const double mass = 1.75;
    const double hoverThrust = mass * gravity;
};

typedef Vec3d Pos;
typedef Mat3f Rotation;

class Pose  {
public:
    Pos position;
    Rotation rotation;
    Eular eular;
    
    void rotation2eularRPY()   {
        Eigen::Vector3d vec_eular = rotation.eulerAngles(0, 1, 2);
        //std::cout << vec_eular[0] << "," << vec_eular[1] << "," << vec_eular[2] << "    ";
        if ( fabs( fabs(vec_eular[2])-PI ) < fabs(vec_eular[2]) )  { //yaw更靠近PI
            //std::cout << "    change     " << fabs(eular[2]) << "   ";
            vec_eular[0] = fabs(vec_eular[0]) - PI;
            vec_eular[1] = PI - fabs(vec_eular[1]);
            vec_eular[2] = PI - fabs(vec_eular[2]);
        }
    }

    void rotation2eularRP()   {
        double a[3] = {rotation(2, 0), rotation(2, 1), rotation(2, 2)}; 
        double factor = sqrt( 1/(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]) );
        for (int i=0; i<=2; i++)    {
            a[i] *= factor;
            //std::cout << a[i] << ", " ;
        }
        //std::cout << std::endl;

        eular.roll = -asin(a[1]);
        double cos_roll = sqrt( 1 - a[1]*a[1] );
        eular.pitch = -asin( a[0] / cos_roll );
        eular.yaw = 0;                          //yaw=0 时 旋转矩阵最后一列为(cxsy. -sx, cxcy), 
        //ROS_INFO("Eular: [%lf, %lf, %lf]! \n",  eular.roll, eular.pitch, eular.yaw);
        
        //_pose.rotation(x, y) 


        // //std::cout << vec_eular[0] << "," << vec_eular[1] << "," << vec_eular[2] << "    ";
        // if ( fabs( fabs(vec_eular[2])-PI ) < fabs(vec_eular[2]) )  { //yaw更靠近PI
        //     //std::cout << "    change     " << fabs(eular[2]) << "   ";
        //     vec_eular[0] = fabs(vec_eular[0]) - PI;
        //     vec_eular[1] = PI - fabs(vec_eular[1]);
        //     vec_eular[2] = PI - fabs(vec_eular[2]);
        // }

        // eular.roll = vec_eular[0]; eular.pitch = vec_eular[1]; eular.yaw = vec_eular[2];

        // std::cout << vec_eular[0] << "," << vec_eular[1] << "," << vec_eular[2] << std::endl;
    }

};
typedef Pose TrajPoint;


class PIDParam   {
public:
    double kp = 1;
    double ki = 0;
    double kd = 0;

    PIDParam() {};
	~PIDParam() {};
	PIDParam(double _kp, double _ki, double _kd): kp(_kp), ki(_ki), kd(_kd) {};
};