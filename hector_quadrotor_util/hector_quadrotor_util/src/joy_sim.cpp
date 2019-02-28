#include <hector_quadrotor_util/joy_sim.h>

//Simulate sending joy msg 
inline void Joysimer::sendJoyMsg(const double _val[5], const int &startButton)
{
    sensor_msgs::Joy controlVel;
    controlVel.axes.push_back(_val[0]);
    controlVel.axes.push_back(_val[1]);
    controlVel.axes.push_back(_val[2]);
    controlVel.axes.push_back(_val[3]);
    controlVel.axes.push_back(_val[4]);
    controlVel.buttons.push_back(0);
    controlVel.buttons.push_back(0);
    controlVel.buttons.push_back(0);
    controlVel.buttons.push_back(0);
    controlVel.buttons.push_back(0);
    controlVel.buttons.push_back(startButton);
    ctrlVelPub.publish(controlVel);  
}

void Joysimer::velCtrlStart()
{    
    ROS_INFO("Start control!");
    for ( int i=0; i<=5; i++ )
    {  
        double val[5] = {0,0,0,0,0};
        int button = 1;
        sendJoyMsg(val, button); //press start button
        ROS_INFO("Start control!");
        loop_rate.sleep();
    }      
}

void Joysimer::velCtrl(double vx,double vy,double vz)
{
    //ROS_INFO("velCtrl vx:%lf, vy:%lf, vz:%lf", vx, vy, vz);
    double val[5] = {0,vz,0,vy,vx};
    int button = 0;
    sendJoyMsg(val, button); 
    loop_rate.sleep(); 
}

void Joysimer::attCtrlStart()
{    
    ROS_INFO("Start control!");
    for ( int i=0; i<=5; i++ )
    {  
        double val[5] = {0,0,0,0,0};
        int button = 1;
        sendJoyMsg(val, button); //press start button
        ROS_INFO("Start control!");
        loop_rate.sleep();
    }      

}

void Joysimer::setAtt(AttCmd &_att_cmd){
    _att_cmd = _att_cmd / M_PI; //joy 1 ----- angle PI
    attCmd = _att_cmd;
    //ROS_INFO("Sending msg: pitch:%lf, roll:%lf, thrust:%lf", attCmd.pitch, attCmd.roll, attCmd.thrust);
}

void Joysimer::setVel(Vec3d &_vel_cmd){
     //_att_cmd = _att_cmd / M_PI; //joy 1 ----- angle PI
     velCmd = _vel_cmd;
    //ROS_INFO("Sending msg: pitch:%lf, roll:%lf, thrust:%lf", attCmd.pitch, attCmd.roll, attCmd.thrust);
}

void Joysimer::msgEnable(const bool _msg_enable){
    msg_enable = _msg_enable;
}

void Joysimer::msgLoop()
{
    while(1)
    {
        // double val[5] = {attCmd.yawrate, 0, -attCmd.thrust, attCmd.roll, attCmd.pitch};
        // int button = 0;
        // sendJoyMsg(val, button); 
        // //ROS_INFO("Sending msg: pitch:%lf, roll:%lf, thrust:%lf", attCmd.roll, attCmd.pitch, attCmd.thrust);
        // loop_rate.sleep();   
        if(msg_enable)
        {
            double val[5] = {0,velCmd.z,0,velCmd.y,velCmd.x};
            int button = 1;
            sendJoyMsg(val, button); 
            //ROS_INFO("Joy");
            //ROS_INFO("Sending msg: pitch:%lf, roll:%lf, thrust:%lf", attCmd.roll, attCmd.pitch, attCmd.thrust);
        }
        loop_rate.sleep();   
    }
}


