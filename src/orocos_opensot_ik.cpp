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
#include <chrono>


orocos_opensot_ik::orocos_opensot_ik(std::string const & name):
    RTT::TaskContext(name),
    _config_path(""),
    _robot_name(""),
    _tau(),
    _model_loaded(false),
    _ports_loaded(false),
    Zero(4,4),
    _step_height(0.08),
    _compute_fb(false),
    _change_control_mode(true)
{
    _logger = XBot::MatLogger::getLogger("/tmp/orocos_opensot_ik");

    this->setActivity(new RTT::Activity(1, 0.002));

    this->addOperation("attachToRobot", &orocos_opensot_ik::attachToRobot,
                this, RTT::ClientThread);

    zero3.setZero();
    Zero.setZero(4,4);
    desired_twist.setZero();
}

void orocos_opensot_ik::setWorld(const KDL::Frame& l_sole_T_Waist, Eigen::VectorXd& q)
{
    _model->setFloatingBasePose(l_sole_T_Waist);
    _model->update();
    _model->getJointPosition(q);
}

bool orocos_opensot_ik::startHook()
{

_task_peer_ptr.reset(this->getPeer(_robot_name));

if(!_task_peer_ptr){
    RTT::log(RTT::Error)<<"Can not getPeer("<<_robot_name<<")"<<RTT::endlog();
    return false;}

#if GROUND_TRUTH_GAZEBO
    getLinkPoseVelocityGazebo = _task_peer_ptr->getOperation("getLinkPoseVelocityGazebo");
#endif


    _joystik_port.createStream(rtt_roscomm::topic("joy"));

    _tau.setZero(_model->getJointNum());

    _qm.setZero(_robot->getJointNum());
    _dqm.setZero(_robot->getJointNum());
    _taum.setZero(_robot->getJointNum());


    //Update world according this new configuration:
//    KDL::Frame l_sole_T_Waist;
//    _model->getPose("Waist", "l_sole", l_sole_T_Waist);

//    l_sole_T_Waist.p.x(0.0);
//    l_sole_T_Waist.p.y(0.0);

    //this->setWorld(l_sole_T_Waist, _q);

    sense(_qm ,_dqm,_taum);

    //#if GROUND_TRUTH_GAZEBO
        Eigen::Affine3d fb_pose;
        Eigen::Vector6d fb_twist;
        FloatingBaseFromGazebo(fb_pose, fb_twist);
        _logger->add("Q_gazebo", fb_pose.matrix());
        _logger->add("Qdot_gazebo", fb_twist);

        _model->setFloatingBaseState(fb_pose, fb_twist );
        _model->update();
    //#endif




    foot_size<<0.2,0.1;//0.2,0.1;
    std::cout<<"foot_size: "<<foot_size<<std::endl;

    Eigen::Affine3d tmp;
    _model->getPose("l_ankle", "l_sole", tmp);

    ik.reset(new opensot_ik(_model, this->getPeriod(), fabs(tmp(2,3)), foot_size));



    relative_activity = 50;
    std::cout<<"relative_activity: "<<relative_activity<<std::endl;
    double __dT = this->getPeriod()*relative_activity;
    std::cout<<"__dT: "<<__dT<<std::endl;
    update_counter = 1;
    _wpg.reset(new legged_robot::Walker(*_model, __dT, 1.5, 0.6, //1.5, 0.6//1., 0.3
                                        foot_size,
                                        "l_sole", "r_sole", "Waist",
                                        3,
                                        2e2,2e3,1e3));
    _wpg->setStepHeight(_step_height);
    _wpg->setFootSpan(_wpg->getFootSpan());//0.8
    next_state = _wpg->getCurrentState();
    integrator.set(_wpg->getCurrentState(), next_state, _wpg->getDuration(), this->getPeriod());


    return true;
}

//#if GROUND_TRUTH_GAZEBO
void orocos_opensot_ik::FloatingBaseFromGazebo(Eigen::Affine3d& fb_pose,
                                               Eigen::Vector6d& fb_twist)
{
    rstrt::kinematics::Twist twist;
    rstrt::geometry::Pose pose;
    if(!getLinkPoseVelocityGazebo("base_link", pose, twist))
        RTT::log(RTT::Error)<<"Can not get floating base from gazebo"<<RTT::endlog();

    Eigen::Quaterniond quat(pose.rotation.rotation[0],
                           pose.rotation.rotation[1],
                           pose.rotation.rotation[2],
                           pose.rotation.rotation[3]);
    fb_pose.linear() = quat.toRotationMatrix();
    fb_pose.translation() = pose.translation.translation.cast<double>();

    fb_twist[0] = twist.linear[0];
    fb_twist[1] = twist.linear[1];
    fb_twist[2] = twist.linear[2];


    fb_twist[3] = twist.angular[0];
    fb_twist[4] = twist.angular[1];
    fb_twist[5] = twist.angular[2];
}
//#endif



void orocos_opensot_ik::updateHook()
{

//#if GROUND_TRUTH_GAZEBO
    Eigen::Affine3d fb_pose;
    Eigen::Vector6d fb_twist;
    FloatingBaseFromGazebo(fb_pose, fb_twist);
    _logger->add("Q_gazebo", fb_pose.matrix());
    _logger->add("Qdot_gazebo", fb_twist);
//#endif

     sense(_qm,_dqm, _taum);

    _model->setFloatingBaseState(fb_pose, fb_twist);
    _model->update();


    _logger->add("qm", _qm);
    _logger->add("dqm", _dqm);
    _logger->add("taum", _taum);




    RTT::FlowStatus fs = _joystik_port.read(joystik_msg);
    if(fs != 0)
    {
        desired_twist.setZero();
        joystick.getWalkingReferences(joystik_msg, desired_twist, _step_height);
        _wpg->setStepHeight(_step_height);
        _wpg->setReference(desired_twist.segment(0,2));
    }


    logRobot(_model);


    if(update_counter == relative_activity)
    {
        _wpg->setCurrentState(next_state);

        update_counter = 1;

        _wpg->solve(next_state);
        _wpg->log(_logger, "wpg");
        next_state.log(_logger, "next_state");

        integrator.set(_wpg->getCurrentState(), next_state, _wpg->getDuration(), this->getPeriod());
    }
    else{
        update_counter++;
    }
    integrator.Tick();
    _out = integrator.Output();

    //ik->setWalkingReferences(_out, _robot->getForceTorque());
    integrator.Output().log(_logger, "integrator");
    _out.log(_logger, "integrator_stabilized");


    Eigen::VectorXd useless(1);
    ik->stack->update(useless);
    ik->stack->log(_logger);

    Eigen::VectorXd x(_model->getJointNum() + 12);
    if(!ik->iHQP->solve(x)){
        RTT::log(RTT::Error)<<"iHQP can not solve"<<RTT::endlog();}

    Eigen::VectorXd qddot_value(_model->getJointNum());
    ik->qddot.getValue(x, qddot_value);
    _logger->add("qddot_value", qddot_value);
    Eigen::Vector6d left_wrench_value, right_wrench_value;
    ik->legs_wrench[0].getValue(x, left_wrench_value);
    ik->legs_wrench[1].getValue(x, right_wrench_value);
    _logger->add("left_wrench_value", left_wrench_value);
    _logger->add("right_wrench_value", right_wrench_value);
    _model->setJointAcceleration(qddot_value);
    _model->update();
    Eigen::MatrixXd Jc1, Jc2;
    _model->getJacobian("l_sole", Jc1);
    _model->getJacobian("r_sole", Jc2);
    _model->computeInverseDynamics(_tau);
    _tau -= (Jc1.transpose()*left_wrench_value + Jc2.transpose()*right_wrench_value);
    _logger->add("tau", _tau);

    for(unsigned int i = 0; i < 6; ++i)
    {
        if(fabs(_tau[i]) > 10e-3){
            RTT::log(RTT::Error)<<"Floating Base Wrench is not 0!"<<RTT::endlog();
            RTT::log(RTT::Error)<<"_tau #"<<i<<": "<<_tau[i]<<RTT::endlog();}
    }

    if(_change_control_mode)
    {
        RTT::OperationCaller<bool(const std::string&, const std::string&)> setControlMode =
                _task_peer_ptr->getOperation("setControlMode");

        setControlMode("left_leg", "JointTorqueCtrl");
        setControlMode("right_leg", "JointTorqueCtrl");
        setControlMode("torso", "JointTorqueCtrl");
        _change_control_mode = false;
    }

    move(_tau.tail(_model->getActuatedJointNum()));

}

void orocos_opensot_ik::stopHook()
{

}

void orocos_opensot_ik::cleanupHook()
{
    _logger->flush();
}


ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(orocos_opensot_ik)


