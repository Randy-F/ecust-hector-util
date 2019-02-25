#include <hector_quadrotor_util/ctrl_interface.h>

//Simulate sending joy msg 
inline void CtrlInterface::sendCtrlMsg(const AttCmd _att_cmd)
{
    attitude.header.stamp = thrust.header.stamp = yawrate.header.stamp = ros::Time::now();
    attitude.roll = _att_cmd.roll;
    attitude.pitch = _att_cmd.pitch;
    thrust.thrust = _att_cmd.thrust;
    yawrate.turnrate = _att_cmd.yawrate;

    attitude_publisher.publish(attitude);
    thrust_publisher.publish(thrust);
    yawrate_publisher.publish(yawrate);
    //ROS_INFO("roll:%lf, pitch:%lf, yawrate:%lf", attitude.roll, attitude.pitch, yawrate.turnrate);
 
}

void CtrlInterface::testCtrlMsg(const AttCmd _att_cmd)
{
    sendCtrlMsg(_att_cmd);
}

void CtrlInterface::velCtrlStart()
{    
    ROS_INFO("Start control!");
    for ( int i=0; i<=5; i++ )
    {  
        double val[5] = {0,0,0,0,0};
        int button = 1;
        //sendMsg(val, button); //press start button
        ROS_INFO("Start control!");
        loop_rate.sleep();
    }      
}

void CtrlInterface::velCtrl(double vx,double vy,double vz)
{
    //ROS_INFO("velCtrl vx:%lf, vy:%lf, vz:%lf", vx, vy, vz);
    double val[5] = {0,vz,0,vy,vx};
    int button = 0;
    //sendMsg(val, button); 
    loop_rate.sleep(); 
}

void CtrlInterface::attCtrlStart()
{    
    ROS_INFO("Start control!");
    for ( int i=0; i<=5; i++ )
    {  
        double val[5] = {0,0,0,0,0};
        int button = 1;
        //sendMsg(val, button); //press start button
        ROS_INFO("Start control!");
        loop_rate.sleep();
    }      
}
     
void CtrlInterface::setAtt(const AttCmd &_att_cmd){
    attCmd = _att_cmd;
}

void CtrlInterface::msgEnable(const bool _msg_enable){
    msg_enable = _msg_enable;

}

void CtrlInterface::msgLoop()
{
    while(1)
    {
        if(msg_enable)  
        {
            sendCtrlMsg(attCmd);
            //ROS_INFO("CtrlInterface");
        }
        //ROS_INFO("Sending msg: roll:%lf, pitch:%lf, thrust:%lf", attCmd.roll, attCmd.pitch, attCmd.thrust);
        loop_rate.sleep();   
    }
}


