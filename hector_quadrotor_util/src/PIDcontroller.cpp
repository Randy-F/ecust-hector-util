#include <hector_quadrotor_util/PIDcontroller.h>

PIDcontroller::PIDcontroller(): I_limit_(100)
{
}

PIDcontroller::~PIDcontroller()
{
}

PIDcontroller::PIDcontroller(float kp, float ki, float kd) {
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
};
PIDcontroller::PIDcontroller(float kp, float ki, float kd, float I_limit) {
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
	I_limit_ = I_limit;
};

void PIDcontroller::setP(float kp) {
	kp_ = kp;
}
void PIDcontroller::setI(float ki) {
	ki_ = ki;
}
void PIDcontroller::setD(float kd) {
	kd_ = kd;
}
void PIDcontroller::setKpid(float kp, float ki, float kd) {
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
}
void PIDcontroller::setPIDParam(ros::NodeHandle &nh, const std::string &con_name) {
	double _kp, _ki, _kd;
	nh.param(con_name + "_" + "kp", _kp, 1.0);
	nh.param(con_name + "_" + "ki", _ki, 1.0);
	nh.param(con_name + "_" + "kd", _kd, 1.0);
	setKpid(_kp, _ki, _kd);
}
void PIDcontroller::setIlimit(float I_limit) {
	I_limit_ = I_limit;
}
void PIDcontroller::setTarget(float target) {
	//std::cout<<"target set"<<target<<std::endl;
	setpoint_ = target;
}
float PIDcontroller::getOutput(float processVariable) {
	auto t = std::chrono::system_clock::now();
	auto diff = std::chrono::system_clock::now() - prev_time_;
	prev_time_ = t;
	
	float error = setpoint_ - processVariable;
	if (!init_)
	{
		init_ = true;
		pre_error_ = error;
		return 0;
	}

	float dt = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(diff).count()) * 0.000001f;
	dt = dt> 0.01f?dt:0.01f;
	float proportionalGain = 0;
	float derivativeGain = 0;
	float integralGain = 0;

	if (kp_ != 0) {
		proportionalGain = error * kp_;
	}
	if (kd_ != 0) {
		float derivative = (error - pre_error_) / dt;
		derivativeGain = derivative * kd_;
	}
	if (ki_ != 0) {
		float temp = sum_ + error * dt;
		//ROS_INFO("Temp I: %lf, dt: %lf", temp, dt);
		if (temp > I_limit_) {
			sum_ = I_limit_;
		}
		else if (temp < -I_limit_) {
			sum_ = -I_limit_;
		}
		else {
			sum_ = temp;
		}
		integralGain = sum_ * ki_;
	}
    pre_error_ = error;

    //ROS_INFO("P: %lf, I: %lf, D: %lf,", proportionalGain, integralGain, derivativeGain);

	return proportionalGain + derivativeGain + integralGain;
}
float PIDcontroller::getSum() {
	return sum_;
}
void PIDcontroller::clear() {
	sum_ = 0;
	init_ = false;
}
void PIDcontroller::printParam() {
	std::cout << "kp: " << kp_ << " ki: " << ki_ << " kd: " << kd_ << std::endl;
}
