#include <orocos_opensot_ik.h>
#include <RobotInterfaceOROCOS/RobotInterfaceOROCOS.h>

bool orocos_opensot_ik::attachToRobot(const std::string &robot_name, const std::string &urdf_path, const std::string &srdf_path, const std::string &config_path  )
{
    //_ports_loaded =  XBot::RobotInterfaceOROCOS::attachToRobot(robot_name, config_path,
    //    _robot, std::shared_ptr<RTT::TaskContext>(this));
    _ports_loaded = XBot::RobotInterfaceOROCOS::attachToRobot(robot_name, urdf_path, srdf_path, true, "RBDL","",
            						      _robot, std::shared_ptr<RTT::TaskContext>(this));
    if(!_ports_loaded){
        RTT::log(RTT::Error)<<"ERROR!!! attachToRobot returned false"<<RTT::endlog();
        return false;}
    if(!_robot){
        RTT::log(RTT::Error)<<"ERROR!!! _robot is invalid pointer"<<RTT::endlog();
        return false;}

    //_model = XBot::ModelInterface::getModel(config_path);
    //if(!_model){
    //    RTT::log(RTT::Error)<<"ERROR!!! _model is invalid pointer"<<RTT::endlog();
    //    return false;
    // }

    _model_loaded = true;
    return true;
}

void orocos_opensot_ik::move(const Eigen::VectorXd& q)
{
    _robot->setPositionReference(q);
    _robot->move();
}

void orocos_opensot_ik::sense(Eigen::VectorXd &q, Eigen::VectorXd &tau)
{
    _robot->sense();
    _robot->getJointPosition(q);
    _robot->getJointEffort(tau);
}

bool orocos_opensot_ik::configureHook()
{
    if(!_model_loaded)
    {
        RTT::log(RTT::Info)<<"Model was not loaded! Call loadConfig(const std::string &config_path)"<<RTT::endlog();
        return false;
    }

    if(!_ports_loaded)
    {
        RTT::log(RTT::Info)<<"Ports was not loaded! Call attachToRobot(const std::string &robot_name)"<<RTT::endlog();
        return false;
    }

    this->addPort(_joystik_port).doc("Joystik from ROS");

//    Eigen::Affine3d T;
//    _model->getPose("r_foot_upper_right_link", T);
//    std::cout<<"r_foot_upper_right_link: "<<T.translation()<<std::endl;
//    _model->getPose("r_foot_lower_right_link", T);
//    std::cout<<"r_foot_lower_right_link: "<<T.translation()<<std::endl;
//    _model->getPose("l_foot_upper_left_link", T);
//    std::cout<<"l_foot_upper_left_link: "<<T.translation()<<std::endl;
//    _model->getPose("l_foot_lower_left_link", T);
//    std::cout<<"l_foot_lower_left_link: "<<T.translation()<<std::endl;

    return true;
}

