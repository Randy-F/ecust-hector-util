#include <hector_quadrotor_util/navigator.h>

using namespace std;

std::vector<Pose> Navigator::pose_array; 
const uint Navigator::pose_array_size = 200;
bool Navigator::is_reciving_pose = true;

Eular Navigator::cur_euler;   //Definnation of static var
Pos Navigator::cur_pose;
bool Navigator::new_att = false;
bool Navigator::new_pose = false;

void Navigator::ctrlLoop()
{    
    std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
    // while( is_reciving_pose );  //wait for reciving pose finished
    
    // for ( auto iter = pose_array.begin(); iter != pose_array.end(); iter++ ) {
    //     iter->rotation2eular();
    // }

    // for ( auto iter = pose_array.begin(); iter != pose_array.end(); iter++ ) {
    //     std::cout << iter->position.x << "," << iter->position.y << "," << iter->position.z << std::endl;
    //     std::cout << iter->eular.roll << "," << iter->eular.pitch << "," << iter->eular.yaw << std::endl << std::endl;
    // }

    joysimer->attCtrlStart();
    ROS_INFO("attCtrl");

    ofstream outFile; 
    outFile.open("/home/randy/ros_ws/hectorsim_ws/src/cmd_data.csv" , ios::out); 

    // Vec3d velCmd(0, 0, 2);
    // joysimer->setVel(velCmd);

    // const double gravity = physicsParm.gravity;
    // const double mass = physicsParm.mass;
    
    // cout << "hoverThrust" << hoverThrust << endl;

    // joysimer->msgEnable(false);
    // ctrlInterface->msgEnable(true);
    // AttCmd att_cmd(0, 0, -10, hoverThrust + 2);
    // ctrlInterface->setAtt(att_cmd);
    // std::this_thread::sleep_for(std::chrono::duration<double>(5));
    // ROS_INFO("Switch!!!");

    // att_cmd = AttCmd(0.4, 0, -10, hoverThrust);
    // ctrlInterface->setAtt(att_cmd);
    // while(1);

    // joysimer->msgEnable(false);
    // ctrlInterface->msgEnable(true);

    const double hoverThrust = physicsParm.hoverThrust;
    index = 0;
    while(1)    {
        //pid
        if(new_att)    {
            float con_roll = roll_con.getOutput(cur_euler.roll);
            float con_pitch = pitch_con.getOutput(cur_euler.pitch);
            float con_yawrate = yaw_con.getOutput(cur_euler.yaw);
            
            float con_vz = vz2z_con.getOutput(cur_pose.z);
            float con_thrust = thrust2vz_con.getOutput(con_vz);

            cout << << endl;

            // if (check_arrival())    {
            //     ROS_INFO("Arrive!!");
            //     index++;
            //     if (index==sp_x.size()) {
            //         ROS_INFO("Switch!!!");
            //         joysimer->msgEnable(false);
            //         ctrlInterface->msgEnable(true);
            //         AttCmd att_cmd(-0.5, 0.5, -10, 17.8);
            //         ctrlInterface->setAtt(att_cmd);
            //         std::this_thread::sleep_for(std::chrono::duration<double>(0.4));
            //         att_cmd = AttCmd(-0.5, -0.5, -10, 17.8);
            //         ctrlInterface->setAtt(att_cmd);
            //         while(1);
            //     }
            //     x_con.setTarget(sp_x[index]);
            //     y_con.setTarget(sp_y[index]);
            //     z_con.setTarget(sp_z[index]);
            // }
            // AttCmd att_cmd = AttCmd(con_roll, con_pitch, -10, 17.8);
            //ROS_INFO("con_thrust: %lf! \n", hoverThrust + con_thrust);
            AttCmd att_cmd = AttCmd(0, 0, -10, hoverThrust + con_thrust);
            ctrlInterface->setAtt(att_cmd);

            //ROS_INFO_STREAM_THROTTLE(10, "con_v: [%lf, %lf, %lf]! \n", con_vz, con_vz, con_vz);
            
            new_pose = false;
        }
    }

    // index = 0;
    // while(1)    {
    //     //pid
    //     if(new_pose)    {
    //         float con_vx = x_con.getOutput(cur_pose.x);
    //         float con_vy = y_con.getOutput(cur_pose.y);
    //         float con_vz = z_con.getOutput(cur_pose.z);

    //         if (check_arrival())    {
    //             ROS_INFO("Arrive!!");
    //             index++;
    //             if (index==sp_x.size()) {
    //                 ROS_INFO("Switch!!!");
    //                 joysimer->msgEnable(false);
    //                 ctrlInterface->msgEnable(true);
    //                 AttCmd att_cmd(-0.5, 0.5, -10, 17.8);
    //                 ctrlInterface->setAtt(att_cmd);
    //                 std::this_thread::sleep_for(std::chrono::duration<double>(0.4));
    //                 att_cmd = AttCmd(-0.5, -0.5, -10, 17.8);
    //                 ctrlInterface->setAtt(att_cmd);
    //                 while(1);
    //             }
    //             x_con.setTarget(sp_x[index]);
    //             y_con.setTarget(sp_y[index]);
    //             z_con.setTarget(sp_z[index]);
    //         }
    //         Vec3d velCmd(con_vx, con_vy, con_vz);
    //         joysimer->setVel(velCmd);

    //         //ROS_INFO_STREAM_THROTTLE(10, "con_v: [%lf, %lf, %lf]! \n", con_vz, con_vz, con_vz);
    //         //ROS_INFO("con_v: [%lf, %lf, %lf]! \n", con_vz, con_vz, con_vz);
    //         new_pose = false;
    //     }
    // }

    // if(!att_cmd.empty())
    // {
    //     for( auto iter = att_cmd.begin(); iter != att_cmd.end(); iter++)    
    //     {
    //         ROS_INFO("NEXT att_cmd!");
    //         attCtrl(*iter, outFile);
    //         std::this_thread::sleep_for(std::chrono::duration<double>(3));
    //     }
    // }

}
     
//Simulate sending joy msg 
void Navigator::read_lee_cmd(const geometry_msgs::PoseWithCovariance::ConstPtr& msg)    {
    // static int i = 0;
    // ROS_INFO("time: %d",i++);
    Pose _pose;
    _pose.position.x = msg->pose.position.x;
    _pose.position.y = msg->pose.position.y;
    _pose.position.z = msg->pose.position.z;
    for (int x = 0; x < 3; x++) 
      for (int y = 0; y < 3; y++) 
          _pose.rotation(x, y) = msg->covariance[3 * x + y];
    pose_array.push_back(_pose);

    if( pose_array.size() == pose_array_size )
        is_reciving_pose = false;
}

void Navigator::print_pose()    {
    for(uint64_t i = 0; i < pose_array.size(); i++ )    {
        ROS_INFO("Pos:%lf,%lf,%lf", pose_array[i].position.x, pose_array[i].position.y, pose_array[i].position.z);
        cout << "Rotation:"<< pose_array[i].rotation << endl;
        cout << i << endl << endl;;
    }
}

std::vector<Eigen::Vector3d> Navigator::RotationMatrix2euler(const std::vector<Mat3f> &_R)  { 
    std::vector<Eigen::Vector3d> _euler;
    for( auto iter = _R.begin(); iter != _R.end(); iter++)
        _euler.push_back(iter->eulerAngles(0, 1, 2));

    return _euler; 
}

void Navigator::attCtrl(AttCmd &_att_cmd, ofstream &_outFile)   { //with .csv output
    //ROS_INFO("Eular: [%lf, %lf, %lf]! \n",  _att_cmd.roll, _att_cmd.pitch, _att_cmd.yawrate);
    //joysimer->setAtt(_att_cmd);
    ctrlInterface->setAtt(_att_cmd);
    _outFile << _att_cmd.pitch << ',' << _att_cmd.roll << endl;     
}

void Navigator::attitude_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)   {
    cur_euler.roll = msg->vector.x;
    cur_euler.pitch = msg->vector.y;
    cur_euler.yaw = msg->vector.z;
    new_att = true;

    // static int i=0;
    // if(i++ == 10)
    // {
    //     //outFile << _pitch << ',' << _roll << ',' << _yaw << endl; 
    //     ROS_INFO(" Twist: (%f, %f, %f) ", cur_euler.pitch, cur_euler.roll, cur_euler.yaw);
    //     i=0;
    // }    

    static int i=0;
    if(i++ == 10)   {
        //ROS_INFO(" att: (%f, %f, %f) ", cur_euler.roll, cur_euler.pitch, cur_euler.yaw);
        //ROS_INFO(" Vel: (%f, %f, %f) ", vx, vy, vz);
        i=0;
    }    
}

void Navigator::position_callback(const nav_msgs::Odometry::ConstPtr& msg)  {
    double pre_x = cur_pose.x;
    double pre_y = cur_pose.y;
    double pre_z = cur_pose.z;
    cur_pose.x = msg->pose.pose.position.x;
    cur_pose.y = msg->pose.pose.position.y;
    cur_pose.z = msg->pose.pose.position.z;

    double vx = cur_pose.x - pre_x;
    double vy = cur_pose.y - pre_y;
    double vz = cur_pose.z - pre_z;

    new_pose = true;

    static int i=0;
    if(i++ == 10)   {
        //outFile << _pitch << ',' << _roll << ',' << _yaw << endl; 
        ROS_INFO(" Pos: (%f, %f, %f) ", cur_pose.x, cur_pose.y, cur_pose.z);
        //ROS_INFO(" Vel: (%f, %f, %f) ", vx, vy, vz);
        i=0;
    }    
}

void Navigator::pid_init(ros::NodeHandle &nh)  {

    x_con.setPIDParam(nh, "x_con");
    x_con.setTarget(*it_x);
    x_con.printParam();

    y_con.setPIDParam(nh, "y_con");
    y_con.setTarget(*it_y);
    y_con.printParam();

    z_con.setPIDParam(nh, "z_con");
    z_con.setTarget(*it_z);
    z_con.printParam();

    roll_con.setPIDParam(nh, "roll_con");
    roll_con.setTarget(0.3);
    roll_con.printParam();

    pitch_con.setPIDParam(nh, "pitch_con");
    pitch_con.setTarget(0);
    pitch_con.printParam();

    yaw_con.setPIDParam(nh, "yaw_con");
    yaw_con.setTarget(0);
    yaw_con.printParam();

    vz2z_con.setPIDParam(nh, "vz2z_con");
    vz2z_con.setTarget(3);
    vz2z_con.printParam();

    thrust2vz_con.setPIDParam(nh, "thrust2vz_con");
    thrust2vz_con.setTarget(0);
    thrust2vz_con.printParam();


    // double test_param;
    // nh.param("test_param", test_param, 1.0);
    // std::cout << "test_param" << test_param << std::endl;
    // std::cout << "test_param" << test_param << std::endl;
}

void Navigator::traj_init() { 
    it_x = sp_x.begin();
    it_y = sp_y.begin();
    it_z = sp_z.begin();
}

bool Navigator::check_arrival() {
    double sq_dist = (sp_x[index]-cur_pose.x)*(sp_x[index]-cur_pose.x) + (sp_y[index]-cur_pose.y)*(sp_y[index]-cur_pose.y) + (sp_z[index]-cur_pose.z)*(sp_z[index]-cur_pose.z);
    //double sq_dist = (*it_x-cur_pose.x)*(*it_x-cur_pose.x) + (*it_y-cur_pose.y)*(*it_y-cur_pose.y) + (*it_z-cur_pose.z)*(*it_z-cur_pose.z);
    double arrival_dist = 0.2;
    //ROS_INFO("%lf, %lf", *it_y, cur_pose.y);
    return(sq_dist < arrival_dist*arrival_dist);
}

