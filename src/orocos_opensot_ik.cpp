/* Author: Enrico Mingo Hoffman
 *
 * Description: Based on the simple orocos/rtt component template by Pouya Mohammadi
 */

#include "orocos_opensot_ik.h"
#include <rtt/Component.hpp>
#include <rtt/Operation.hpp>
#include <rtt/OperationCaller.hpp>


orocos_opensot_ik::orocos_opensot_ik(std::string const & name):
    RTT::TaskContext(name),
    _config_path(""),
    _robot_name(""),
    _q(),
    _dq(),
    _model_loaded(false),
    _ports_loaded(false)
{
    this->setActivity(new RTT::Activity(1, 0.001));

    this->addOperation("loadConfig", &orocos_opensot_ik::loadConfig,
                this, RTT::ClientThread);
    this->addOperation("attachToRobot", &orocos_opensot_ik::attachToRobot,
                this, RTT::ClientThread);
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

    return true;
}

bool orocos_opensot_ik::startHook()
{
    _q.setZero(_model->getJointNum());
    _dq.setZero(_model->getJointNum());

    sense(_q);

    _model->setJointPosition(_q);
    _model->update();

    ik.reset(new opensot_ik(_q, _model, this->getPeriod()));

    return true;
}

void orocos_opensot_ik::updateHook()
{
    _model->setJointPosition(_q);
    _model->update();

    ik->stack->update(_q);

    if(!ik->iHQP->solve(_dq)){
        _dq.setZero(_dq.size());
        std::cout<<"iHQP can not solve"<<std::endl;}

    _q+=_dq;

    move(_q);
}

void orocos_opensot_ik::stopHook()
{

}

void orocos_opensot_ik::cleanupHook()
{

}

bool orocos_opensot_ik::attachToRobot(const std::string &robot_name)
{
    _robot_name = robot_name;
    RTT::log(RTT::Info)<<"Robot name: "<<_robot_name<<RTT::endlog();

    RTT::TaskContext* task_ptr = this->getPeer(robot_name);
    if(!task_ptr){
        RTT::log(RTT::Error)<<"Can not getPeer("<<robot_name<<")"<<RTT::endlog();
        return false;}

    RTT::log(RTT::Info)<<"Found Peer "<<robot_name<<RTT::endlog();

    RTT::OperationCaller<std::map<std::string, std::vector<std::string> >(void) > getKinematicChainsAndJoints
        = task_ptr->getOperation("getKinematicChainsAndJoints");

    _map_kin_chains_joints = getKinematicChainsAndJoints();

    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        std::string kin_chain_name = it->first;
        std::vector<std::string> joint_names = it->second;

        _kinematic_chains_feedback_ports[kin_chain_name] =
            boost::shared_ptr<RTT::InputPort<rstrt::robot::JointState> >(
                        new RTT::InputPort<rstrt::robot::JointState>(
                            kin_chain_name+"_"+"JointFeedback"));
        this->addPort(*(_kinematic_chains_feedback_ports.at(kin_chain_name))).
                doc(kin_chain_name+"_"+"JointFeedback port");

        _kinematic_chains_feedback_ports.at(kin_chain_name)->connectTo(
                    task_ptr->ports()->getPort(kin_chain_name+"_"+"JointFeedback"));

        rstrt::robot::JointState tmp(joint_names.size());
        _kinematic_chains_joint_state_map[kin_chain_name] = tmp;
        RTT::log(RTT::Info)<<"Added "<<kin_chain_name<<" port and data"<<RTT::endlog();

        _kinematic_chains_output_ports[kin_chain_name] =
                boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> >(
                            new RTT::OutputPort<rstrt::kinematics::JointAngles>(
                                kin_chain_name+"_"+"JointPositionCtrl"));
        this->addPort(*(_kinematic_chains_output_ports.at(kin_chain_name))).
                doc(kin_chain_name+"_"+"JointPositionCtrl port");
        _kinematic_chains_output_ports.at(kin_chain_name)->connectTo(
                    task_ptr->ports()->getPort(kin_chain_name+"_"+"JointPositionCtrl"));


        rstrt::kinematics::JointAngles tmp2(joint_names.size());
        _kinematic_chains_desired_joint_state_map[kin_chain_name] = tmp2;
    }


    RTT::OperationCaller<std::vector<std::string> (void) > getForceTorqueSensorsFrames
        = task_ptr->getOperation("getForceTorqueSensorsFrames");
    std::vector<std::string> ft_sensors_frames = getForceTorqueSensorsFrames();
    for(unsigned int i = 0; i < ft_sensors_frames.size(); ++i)
    {
        _frames_ports_map[ft_sensors_frames[i]] =
                boost::shared_ptr<RTT::InputPort<rstrt::dynamics::Wrench> >(
                    new RTT::InputPort<rstrt::dynamics::Wrench>(
                        ft_sensors_frames[i]+"_SensorFeedback"));
        this->addPort(*(_frames_ports_map.at(ft_sensors_frames[i]))).
                doc(ft_sensors_frames[i]+"_SensorFeedback port");

        _frames_ports_map.at(ft_sensors_frames[i])->connectTo(
                    task_ptr->ports()->getPort(ft_sensors_frames[i]+"_SensorFeedback"));

        rstrt::dynamics::Wrench tmp;
        _frames_wrenches_map[ft_sensors_frames[i]] = tmp;

        RTT::log(RTT::Info)<<"Added "<<ft_sensors_frames[i]<<" port and data"<<RTT::endlog();
    }
    _ports_loaded = true;
    return true;
}

void orocos_opensot_ik::sense(Eigen::VectorXd &q)
{
    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        RTT::FlowStatus fs = _kinematic_chains_feedback_ports.at(it->first)->read(
                    _kinematic_chains_joint_state_map.at(it->first));

        for(unsigned int i = 0; i < it->second.size(); ++i)
            q[_model->getDofIndex(it->second.at(i))] =
                    _kinematic_chains_joint_state_map.at(it->first).angles[i];
    }
}

void orocos_opensot_ik::move(const Eigen::VectorXd& q)
{
    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        for(unsigned int i = 0; i < it->second.size(); ++i)
            _kinematic_chains_desired_joint_state_map.at(it->first).angles[i] =
                    _q[_model->getDofIndex(it->second.at(i))];

        _kinematic_chains_output_ports.at(it->first)->
                write(_kinematic_chains_desired_joint_state_map.at(it->first));
    }
}

bool orocos_opensot_ik::loadConfig(const std::string &config_path)
{
    _config_path = config_path;
    _model = XBot::ModelInterface::getModel(_config_path);
    _model_loaded = true;
    return true;
}

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(orocos_opensot_ik)


