#include <orocos_opensot_ik.h>

bool orocos_opensot_ik::loadConfig(const std::string &config_path)
{
    _config_path = config_path;
    _model = XBot::ModelInterface::getModel(_config_path);
    _model_loaded = true;
    return true;
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

