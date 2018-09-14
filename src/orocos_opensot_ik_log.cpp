#include <orocos_opensot_ik.h>

void orocos_opensot_ik::logRobot(const XBot::ModelInterface::Ptr robot)
{
    Eigen::Vector3d tmp;
    robot->getCOMVelocity(tmp);
    _logger->add("com_vel", tmp);

    robot->getCOM(tmp);
    _logger->add("com_pos", tmp);

    Eigen::Affine3d tmp2;
    robot->getPose("l_sole", tmp2);
    _logger->add("lsole_pos", tmp2.matrix());

    Eigen::Vector6d tmp3;
    robot->getVelocityTwist("l_sole", tmp3);
    _logger->add("lsole_vel", tmp3);

    robot->getPose("r_sole", tmp2);
    _logger->add("rsole_pos", tmp2.matrix());

    robot->getVelocityTwist("r_sole", tmp3);
    _logger->add("rsole_vel", tmp3);

    robot->getPose("Waist", tmp2);
    _logger->add("waist_pos", tmp2.matrix());

    std::map< std::string, XBot::ForceTorqueSensor::ConstPtr > frames_wrenches_map = _robot->getForceTorque();
    Eigen::Vector6d wrench;
    frames_wrenches_map.at("l_leg_ft")->getWrench(wrench);
    _logger->add("l_leg_ft", wrench);
    frames_wrenches_map.at("r_leg_ft")->getWrench(wrench);
    _logger->add("r_leg_ft", wrench);
}
