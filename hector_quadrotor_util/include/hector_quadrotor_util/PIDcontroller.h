#ifndef PIDcontroller_hpp
#define PIDcontroller_hpp

//#pragma once
#include <chrono>
#include <iostream>
#include <ros/ros.h>

class PIDcontroller
{
private:
	float setpoint_;
	float kp_;
	float ki_;
	float kd_;
	float pre_error_;
	float I_limit_;
	float sum_;
	bool init_;
	const std::string con_name;
	std::chrono::time_point<std::chrono::system_clock> prev_time_;
public:
	PIDcontroller();
	~PIDcontroller();

	PIDcontroller(float kp, float ki, float kd);
	PIDcontroller(float kp, float ki, float kd, float I_limit);
	void setP(float kp);
	void setI(float ki);
	void setD(float kd);
	void setKpid(float kp, float ki, float kd);
	void setPIDParam(ros::NodeHandle &nh, const std::string &con_name);
	void printParam();
	void setIlimit(float I_limit);
	void setTarget(float target);
	float getOutput(float processVariable);
	float getSum();
	void clear();

};


#endif // !PIDcontroller_hpp
