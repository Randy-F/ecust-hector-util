#include <hector_quadrotor_util/navigator.h>

using namespace std;

std::deque<TrajPoint> Navigator::traj; 
const uint Navigator::traj_size = 200;
bool Navigator::is_reciving_traj = true;

Eular Navigator::cur_euler;   //Definnation of static var
Pos Navigator::cur_pose;
double Navigator::cur_vz = 0;
bool Navigator::new_att = false;
bool Navigator::new_pose = false;
bool Navigator::position_callback_init_ = false;
ros::Time Navigator::cur_time = ros::Time(0);


void Navigator::ctrlLoop()
{    
    std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
    while( is_reciving_traj );  //等待接收轨迹结束

    // for ( auto iter = traj.begin(); iter != traj.end(); iter++ ) {
    //     std::cout << iter->position.x << "," << iter->position.y << "," << iter->position.z << std::endl;
    //     std::cout << iter->eular.roll << "," << iter->eular.pitch << "," << iter->eular.yaw << std::endl << std::endl;
    // }
    // print_pose();
    joysimer->attCtrlStart();
    ROS_INFO("attCtrl");

    ofstream outFile; 
    outFile.open("/home/randy/ros_ws/hectorsim_ws/src/cmd_data.csv" , ios::out); 


    const double hoverThrust = physicsParm.hoverThrust;
    // joysimer->msgEnable(false);
    // ctrlInterface->msgEnable(true);
    // AttCmd att_cmd(0, 0, -10, hoverThrust + 1);
    // ctrlInterface->setAtt(att_cmd);
    // std::this_thread::sleep_for(std::chrono::duration<double>(3));
    // ROS_INFO("Switch!!!");

    // att_cmd = AttCmd(0.4, 0, -10, hoverThrust);
    // ctrlInterface->setAtt(att_cmd);
    

    // 手柄调试
        // joysimer->msgEnable(false);
        // ctrlInterface->msgEnable(false);
        // while(1);
    
    // 开环姿态调试
    // joysimer->msgEnable(false);
    // ctrlInterface->msgEnable(true);
    // AttCmd att_cmd = AttCmd(-0.7, 0, -10, 20);
    // ctrlInterface->setAtt(att_cmd);
    // std::this_thread::sleep_for(std::chrono::duration<double>(4));
    // ROS_INFO("Switch!!!");
    // att_cmd = AttCmd(0.7, 0, -10, 20);
    // ctrlInterface->setAtt(att_cmd);
    // while(1);


    while(1)    {
		switch(ctrlState)  //两种控制方式  速度控制和姿态控制
		{
		case CtrlState::VEL_CON:
            if(new_pose)    {
                
                float con_vx = x_con.getOutput(cur_pose.x);
                float con_vy = y_con.getOutput(cur_pose.y);
                float con_vz = z_con.getOutput(cur_pose.z);

                if (check_arrival(cur_pose, Pos(0, 0, 5), 0.03) &&  fabs(cur_vz) <= 0.005 )  {  //到达h=5,切换至姿态模式
                    //ROS_INFO("VEL_CON Arrive!!");
                    joysimer->msgEnable(false);
                    ctrlInterface->msgEnable(true);
                    ctrlState = CtrlState::ATT_CON;
                    //while(1);
                }


                Vec3d velCmd(con_vx, con_vy, con_vz);
                joysimer->setVel(velCmd);

                //ROS_INFO_STREAM_THROTTLE(10, "con_v: [%lf, %lf, %lf]! \n", con_vz, con_vz, con_vz);
                //ROS_INFO("con_v: [%lf, %lf, %lf]! \n", con_vz, con_vz, con_vz);
                new_pose = false;
            }
			break; 

		case CtrlState::ATT_CON:
            if(new_pose)    {

                while (check_arrival(traj.front().position.y ,cur_pose.y, 0.01) )    { //结束后的悬停状态，定点悬停
                    //ROS_INFO("ATT_CON Arrive!!");
                    set_next_trajpoint();
                    if (ctrlState == CtrlState::HOVER)  break;
                }
                if (ctrlState == CtrlState::HOVER)  break;

                con_roll = roll_con.getOutput(cur_euler.roll);
                con_pitch = pitch_con.getOutput(cur_euler.pitch);
                con_yawrate = yaw_con.getOutput(cur_euler.yaw);
                float con_vz = vz2z_con.getOutput(cur_pose.z);
                thrust2vz_con.setTarget(con_vz);
                con_thrust = thrust2vz_con.getOutput(cur_vz);
                //ROS_INFO(" com_ (%f, %f, %f) ", con_roll, con_pitch, con_yawrate);

                //ROS_INFO(" cur_roll: %f, sp_roll: %f, con_roll: %f) ", cur_euler.roll, -cur_traj_point.eular.roll, con_roll);
                //ROS_INFO(" cur_pitch: %f, sp_pitch: %f, con_pitch: %f) ", cur_euler.pitch, -cur_traj_point.eular.pitch, con_pitch);
                //ROS_INFO(" com_ (%f, %f, %f) ", con_roll, con_pitch, con_yawrate);
                //cout << "cur_vz: " << cur_vz << " con_thrust: " << con_thrust << endl;
                //ROS_INFO("con_thrust: %lf! \n", hoverThrust + con_thrust);
                AttCmd att_cmd = AttCmd(con_roll, con_pitch, -10, hoverThrust + con_thrust);
                ctrlInterface->setAtt(att_cmd);
                
                new_pose = false;
            }
            if(new_att)    {
                con_roll = roll_con.getOutput(cur_euler.roll);
                con_pitch = pitch_con.getOutput(cur_euler.pitch);
                con_yawrate = yaw_con.getOutput(cur_euler.yaw);

                //ROS_INFO(" cur_roll: %f, sp_roll: %f, con_roll: %f) ", cur_euler.roll, -cur_traj_point.eular.roll, con_roll);
                //ROS_INFO(" cur_pitch: %f, sp_pitch: %f, con_pitch: %f) ", cur_euler.pitch, -cur_traj_point.eular.pitch, con_pitch);
                //ROS_INFO(" com_ (%f, %f, %f) ", con_roll, con_pitch, con_yawrate);
                //cout << "cur_vz: " << cur_vz << " con_thrust: " << con_thrust << endl;
                //ROS_INFO("con_thrust: %lf! \n", hoverThrust + con_thrust);
                AttCmd att_cmd = AttCmd(con_roll, con_pitch, -10, hoverThrust + con_thrust);
                ctrlInterface->setAtt(att_cmd);
                
                new_att = false;
            }
            break;

        case CtrlState::HOVER:
            if(new_pose)    {   
                ROS_INFO("HOVER!");             
                AttCmd att_cmd = AttCmd(0, 0, -10, hoverThrust);
                ctrlInterface->setAtt(att_cmd);
                new_pose = false;
            }
			break; 

		default:
			break;
		}

    }


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
    static int i = 0;
    //ROS_INFO("time: %d",i++);
    Pose _pose;
    _pose.position.x = msg->pose.position.x;
    _pose.position.y = msg->pose.position.y + 5;
    _pose.position.z = msg->pose.position.z;
    for (int x = 0; x < 3; x++) 
      for (int y = 0; y < 3; y++) 
          _pose.rotation(x, y) = msg->covariance[3 * x + y];
    _pose.rotation2eularRP();
    traj.push_back(_pose);

    if( traj.size() == traj_size )
        is_reciving_traj = false;
}

void Navigator::print_pose()    {
    for(uint64_t i = 0; i < traj.size(); i++ )    {
        ROS_INFO("Pos:%lf,%lf,%lf", traj[i].position.x, traj[i].position.y, traj[i].position.z);
        cout << "Rotation:"<< traj[i].rotation << endl;
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
    // cur_euler.roll = msg->vector.x;
    // cur_euler.pitch = msg->vector.y;
    // cur_euler.yaw = msg->vector.z;
    // new_att = true;
    //ROS_INFO("new_att");

    // static int i=0;
    // if(i++ == 10)
    // {
    //     //outFile << _pitch << ',' << _roll << ',' << _yaw << endl; 
    //     ROS_INFO(" Twist: (%f, %f, %f) ", cur_euler.pitch, cur_euler.roll, cur_euler.yaw);
    //     i=0;
    // }    

    // ROS_INFO("  (%f, %f, %f) ", cur_euler.roll, cur_euler.pitch, cur_euler.yaw);
    static int i=0;
    if(++i == 10)   {
        //ROS_INFO(" att: (%f, %f, %f) ", cur_euler.roll, cur_euler.pitch, cur_euler.yaw);
        //ROS_INFO(" cur_roll: %f, sp_roll: %f) ", cur_euler.roll, -cur_traj_point.eular.roll);
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

    auto prev_time_ = cur_time;
    cur_time = msg->header.stamp;
	auto dt = cur_time - prev_time_;
    cur_vz = (cur_pose.z - pre_z)/dt.toSec();
    if (!position_callback_init_)	{
		position_callback_init_ = true;
		cur_vz = 0;
        ROS_INFO("position_callback_init_");
	}

    Eigen::Quaterniond quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2,1,0);

    Eigen::Vector3d vec_eular = Eigen::Vector3d(eulerAngle[2], eulerAngle[1], eulerAngle[0]); //摆正rpy顺序
    //std::cout << vec_eular[0] << "," << vec_eular[1] << "," << vec_eular[2] << "    ";

    if ( fabs( fabs(vec_eular[2])-PI ) < fabs(vec_eular[2]) )  { //yaw更靠近PI
        //std::cout << "    change     " << fabs(eular[2]) << "   ";
        vec_eular[0] = fabs(vec_eular[0]) - PI;
        vec_eular[1] = PI - fabs(vec_eular[1]);
        vec_eular[2] = PI - fabs(vec_eular[2]);
    }

    cur_euler.roll = vec_eular[0];
    cur_euler.pitch = vec_eular[1];
    cur_euler.yaw = vec_eular[2];

    new_pose = true;

    static int i=0;
    if(++i == 10)   {
        //outFile << _pitch << ',' << _roll << ',' << _yaw << endl; 
        //ROS_INFO(" Pos: (%f, %f, %f) Vel: %f ", cur_pose.x, cur_pose.y, cur_pose.z, cur_vz);
        //ROS_INFO(" Vel: %f", cur_vz);
        ROS_INFO(" cur_euler: (%lf, %lf, %lf) ", cur_euler.roll, cur_euler.pitch, cur_euler.yaw);
        i=0;
    }    
}

void Navigator::pid_init(ros::NodeHandle &nh)  {

    x_con.setPIDParam(nh, "x_con");
    x_con.setTarget(0);
    x_con.printParam();

    y_con.setPIDParam(nh, "y_con");
    y_con.setTarget(0);
    y_con.printParam();

    z_con.setPIDParam(nh, "z_con");
    z_con.setTarget(5);
    z_con.printParam();

    roll_con.setPIDParam(nh, "roll_con");
    roll_con.setTarget(1);
    roll_con.printParam();
    roll_con.setLimit(-PI/2, PI/2);	

    pitch_con.setPIDParam(nh, "pitch_con");
    pitch_con.setTarget(0);
    pitch_con.printParam();
    pitch_con.setLimit(-PI/2, PI/2);	

    yaw_con.setPIDParam(nh, "yaw_con");
    yaw_con.setTarget(0);
    yaw_con.printParam();

    vz2z_con.setPIDParam(nh, "vz2z_con");
    vz2z_con.setTarget(5);
    vz2z_con.printParam();

    thrust2vz_con.setPIDParam(nh, "thrust2vz_con");
    thrust2vz_con.setTarget(0);
    thrust2vz_con.printParam();


    // double test_param;
    // nh.param("test_param", test_param, 1.0);
    // std::cout << "test_param" << test_param << std::endl;
    // std::cout << "test_param" << test_param << std::endl;
}

void Navigator::set_next_trajpoint() { 
    //if ( traj.empty() ){ //到达最后路径点，开始悬停
    if ( cur_pose.y > 8 ){ //到达最后路径点，开始悬停
        // roll_con.setTarget(0);
        // pitch_con.setTarget(0);
        // vz2z_con.setTarget(5);
        //ROS_INFO("traj.empty()!!!");
        ctrlState = CtrlState::HOVER;
        // joysimer->msgEnable(true);
        // ctrlInterface->msgEnable(false);
        return ;
    }
    traj.pop_front();  //pop掉上一个路径点，第一个路径点是原点，不需要，所以第一次也可以直接popfront
    cur_traj_point = traj.front();
    roll_con.setTarget(-cur_traj_point.eular.roll);
    pitch_con.setTarget(cur_traj_point.eular.pitch);
    vz2z_con.setTarget(cur_traj_point.position.z);
}

bool Navigator::check_arrival(const double &_sp_z, const double &_cur_z, double _threshold) {
    double dist = (_sp_z-_cur_z);  //
    //double sq_dist = (*it_x-cur_pose.x)*(*it_x-cur_pose.x) + (*it_y-cur_pose.y)*(*it_y-cur_pose.y) + (*it_z-cur_pose.z)*(*it_z-cur_pose.z);
    
    //ROS_INFO("%lf", dist);
    return(dist < _threshold);
}

bool Navigator::check_arrival(const Pos &_cur_pos, const Pos &_sp_pos, double _threshold) {
    double sq_dist = (_sp_pos.x-cur_pose.x)*(_sp_pos.x-cur_pose.x) + (_sp_pos.y-cur_pose.y)*(_sp_pos.y-cur_pose.y) + (_sp_pos.z-cur_pose.z)*(_sp_pos.z-cur_pose.z);
    //double sq_dist = (*it_x-cur_pose.x)*(*it_x-cur_pose.x) + (*it_y-cur_pose.y)*(*it_y-cur_pose.y) + (*it_z-cur_pose.z)*(*it_z-cur_pose.z);
    
    //ROS_INFO("%lf", sq_dist);
    return(sq_dist < _threshold*_threshold);
}



