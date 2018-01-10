/* Author: Enrico Mingo Hoffman
 *
 * Description: Based on the simple orocos/rtt component template by Pouya Mohammadi
 */

#include "orocos_opensot_ik.h"
#include <rtt/Component.hpp>
#include <rtt/Operation.hpp>
#include <rtt/OperationCaller.hpp>
#include <ros/ros.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rtt_rostopic.h>


orocos_opensot_ik::orocos_opensot_ik(std::string const & name):
    RTT::TaskContext(name),
    _config_path(""),
    _robot_name(""),
    _q(),
    _dq(),
    _model_loaded(false),
    _ports_loaded(false),
    _v_max(0.1*700.),
    Zero(4,4),
    _jump_time(0.04),
    _jump_counter(-1)
{
    this->setActivity(new RTT::Activity(1, 0.002));

    this->addOperation("loadConfig", &orocos_opensot_ik::loadConfig,
                this, RTT::ClientThread);
    this->addOperation("attachToRobot", &orocos_opensot_ik::attachToRobot,
                this, RTT::ClientThread);
    this->addOperation("setVMax", &orocos_opensot_ik::setVMax,
                this, RTT::ClientThread);

    zero3.setZero();
    Zero.setZero(4,4);
    desired_twist.setZero();
    centroidal_momentum.setZero(6);

    _logger = XBot::MatLogger::getLogger("orocos_opensot_ik");
}

void orocos_opensot_ik::setVMax(const double vmax)
{
    _v_max = vmax;
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
//    Eigen::Vector3d com;
//    _model->getCOM(com);
//    std::cout<<"com: "<<com<<std::endl;

    return true;
}

void orocos_opensot_ik::setWorld(const KDL::Frame& l_sole_T_Waist, Eigen::VectorXd& q)
{
    _model->setFloatingBasePose(l_sole_T_Waist);
    _model->update();
    _model->getJointPosition(q);
}

Eigen::Vector3d orocos_opensot_ik::getGains(const double x, const double y, const double z)
{
    Eigen::Vector3d tmp;
    tmp<<x,y,z;
    return tmp;
}

bool orocos_opensot_ik::startHook()
{
    _joystik_port.createStream(rtt_roscomm::topic("joy"));

    _q.setZero(_model->getJointNum());
    _dq.setZero(_model->getJointNum());

    Eigen::VectorXd tmp,tmp2;
    tmp = tmp2 = _q_m = _dq_m = _tau_m = Eigen::VectorXd::Zero(_model->getJointNum());
    sense(_q, tmp, tmp2);
    sense(_q_m, _dq_m, _tau_m);
    _logger->add("q_m", _q_m);
    _logger->add("dq_m", _dq_m);
    _logger->add("tau_m", _tau_m);

    _model->setJointPosition(_q);
    _model->setJointVelocity(_dq);
    _model->update();

    //Update world according this new configuration:
    KDL::Frame l_sole_T_Waist;
    _model->getPose("Waist", "l_sole", l_sole_T_Waist);

    l_sole_T_Waist.p.x(0.0);
    l_sole_T_Waist.p.y(0.0);

    this->setWorld(l_sole_T_Waist, _q);

    ik.reset(new opensot_ik(_q, _model, this->getPeriod()));

    _com_initial = ik->com->getActualPosition();

    Eigen::Vector2d foot_size;
    foot_size<<0.2,0.1;
    Eigen::Affine3d tmpAffine;
    _model->getPose("l_ankle", "l_sole", tmpAffine);
    _stabilizer.reset(new CompliantStabilizer(this->getPeriod(), _model->getMass(),
                                    fabs(tmpAffine(2,3)), foot_size, 10.,
                                    getGains(0.15,0.13,0.), getGains(-0.007,-0.007,0.),
                                    getGains(DEFAULT_MaxLimsx, DEFAULT_MaxLimsy, DEFAULT_MaxLimsz),
                                    getGains(DEFAULT_MinLimsx, DEFAULT_MinLimsy, DEFAULT_MinLimsz)));

    return true;
}

void orocos_opensot_ik::stabilizer()
{
    Eigen::Affine3d l_sole, r_sole;
    _model->getPose("l_sole", l_sole);
    _model->getPose("r_sole", r_sole);

    Eigen::Vector2d CopPos_L, CopPos_R;
    CopPos_L(0) = _com_initial[0] - l_sole.translation()[0];
    CopPos_L(1) = _com_initial[1] - l_sole.translation()[1];

    CopPos_R(0) = _com_initial[0] - r_sole.translation()[0];
    CopPos_R(1) = _com_initial[1] - r_sole.translation()[1];

    Eigen::Vector6d left_wrench;
    left_wrench<<_frames_wrenches_map.at("l_leg_ft").forces.cast<double>(),_frames_wrenches_map.at("l_leg_ft").torques.cast<double>();
    //std::cout<<"left_wrench: "<<left_wrench.transpose()<<std::endl;
    Eigen::Vector6d right_wrench;
    right_wrench<<_frames_wrenches_map.at("r_leg_ft").forces.cast<double>(),_frames_wrenches_map.at("r_leg_ft").torques.cast<double>();
    //std::cout<<"right_wrench: "<<right_wrench.transpose()<<std::endl;
    Eigen::Vector3d delta_com = _stabilizer->update(left_wrench, right_wrench,
                                                  CopPos_L, CopPos_R,
                                                  l_sole.translation(), r_sole.translation());

    Eigen::Vector3d com_ref;
    com_ref = _com_initial + delta_com;
    ik->com->setReference(com_ref);

}

void orocos_opensot_ik::setReferences(const sensor_msgs::Joy &msg)
{
    //desired_twist[1] = _v_max*msg.axes[0];
    //desired_twist[0] = _v_max*msg.axes[1];

    //desired_twist[1] = _v_max*msg.axes[0];


//    if(msg.axes[0] > 0.4){
//        desired_twist[5] = 10.0*fabs(msg.axes[0])*_v_max*msg.axes[4];}
//    if(msg.axes[0] < -0.4){
//        desired_twist[5] = -10.0*fabs(msg.axes[0])*_v_max*msg.axes[4];}

    if(msg.buttons[0] && _jump_counter<0)
        _jump_counter=0;

    Eigen::Vector3d twist_com_ref; twist_com_ref.setZero();

if(msg.buttons[1]){
    if(_jump_counter*this->getPeriod() >= 0 && _jump_counter*this->getPeriod() <= _jump_time)
    {
        if(msg.axes[0] > 0.4){
            desired_twist[5] = 10.0*fabs(msg.axes[0])*_v_max;}//*msg.buttons[0];}
        else if(msg.axes[0] < -0.4){
            desired_twist[5] = -10.0*fabs(msg.axes[0])*_v_max;}//*msg.buttons[0];}
        else{
            twist_com_ref[0] = -0.1*_v_max;//*msg.buttons[0];
            desired_twist[4] = 0.1*_v_max;}//*msg.buttons[0];}



    //    if(_v_max*msg.axes[4] < 0.0)
    //        desired_twist[2] = _v_max*msg.axes[4];
    //    else
    //        desired_twist[2] = 10.0*_v_max*msg.axes[4];


        desired_twist[2] = 10.0*_v_max;//*msg.buttons[0];


        //desired_twist[4] = -0.0*_v_max*msg.axes[4];

        //desired_twist[5] = 2.0*_v_max*msg.axes[0];
        _jump_counter += 1;
    }
    else
    {
        desired_twist.setZero();
        _jump_counter = -1;
    }
}

     _logger->add("desired_twist_waist", desired_twist*this->getPeriod());
     _logger->add("desired_twist_com", twist_com_ref*this->getPeriod());


    ik->waist->setReference(ik->waist->getReference(), desired_twist*this->getPeriod());
    ik->com->setReference(ik->com->getReference(),twist_com_ref*this->getPeriod());
}

void orocos_opensot_ik::updateHook()
{
    sense(_q_m, _dq_m, _tau_m);
    _logger->add("q_m", _q_m);
    _logger->add("qdot_m", _dq_m);
    _logger->add("tau_m", _tau_m);



    _model->setJointPosition(_q);
    _model->setJointVelocity(_dq/this->getPeriod());
    _model->update();

//    Eigen::Vector6d L;
//    _model->getCentroidalMomentum(L);
//    ik->mom->setReference(this->getPeriod()*L.segment(3,3));

//    Eigen::Vector3d com;
//    _model->getCOM(com);
//    std::cout<<"com: ["<<com<<"]"<<std::endl;

    stabilizer();

    RTT::FlowStatus fs = _joystik_port.read(joystik_msg);
    if(fs != 0)
        setReferences(joystik_msg);


    ik->stack->update(_q);
//    ik->com_z->update(_q);
//    ik->capture_point->update(_q);

    if(!ik->iHQP->solve(_dq)){
        _dq.setZero(_dq.size());
        std::cout<<"iHQP can not solve"<<std::endl;}

    _q+=_dq;

    _logger->add("q", _q);
    _logger->add("qdot", _dq/this->getPeriod());

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
    _force_torque_sensors_frames = ft_sensors_frames;
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

void orocos_opensot_ik::sense(Eigen::VectorXd &q, Eigen::VectorXd &dq, Eigen::VectorXd &tau)
{
    map<string, vector<string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        RTT::FlowStatus fs = _kinematic_chains_feedback_ports.at(it->first)->read(
                    _kinematic_chains_joint_state_map.at(it->first));

        for(unsigned int i = 0; i < it->second.size(); ++i){
            q[_model->getDofIndex(it->second.at(i))] =
                    _kinematic_chains_joint_state_map.at(it->first).angles[i];
            dq[_model->getDofIndex(it->second.at(i))] =
                    _kinematic_chains_joint_state_map.at(it->first).velocities[i];
            tau[_model->getDofIndex(it->second.at(i))] =
                    _kinematic_chains_joint_state_map.at(it->first).torques[i];
        }

    }

    for(unsigned int i = 0; i < _force_torque_sensors_frames.size(); ++i)
    {
        RTT::FlowStatus fs = _frames_ports_map.at(_force_torque_sensors_frames[i])->read(
                    _frames_wrenches_map.at(_force_torque_sensors_frames[i]));
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

orocos_opensot_ik::~orocos_opensot_ik()
{
    _logger->flush();
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


