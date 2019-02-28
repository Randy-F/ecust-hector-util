void Navigator::attCtrl(Eular &_euler, ofstream &_outFile) //with .csv output
{ 
    //ROS_INFO("Eular: [%lf, %lf, %lf]! \n",  _euler[0], _euler[1], _euler[2]);
    _euler.roll = -_euler.roll / 3.1415927;
    _euler.pitch = -_euler.pitch / 3.1415927;
    _euler.yaw = -_euler.yaw / 3.1415927;

    joysimer->setAtt( _euler.pitch, _euler.roll, 0, 2 );
    
    _outFile << _euler.pitch << ',' << _euler.roll << endl;     
}