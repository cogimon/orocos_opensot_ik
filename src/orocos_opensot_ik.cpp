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
    _q(),
    _dq(),
    _model_loaded(false),
    _ports_loaded(false),
    Zero(4,4),
    _step_height(0.05)
{
    _logger = XBot::MatLogger::getLogger("/tmp/orocos_opensot_ik");

    this->setActivity(new RTT::Activity(1, 0.002));

    this->addOperation("attachToRobot", &orocos_opensot_ik::attachToRobot,
                this, RTT::ClientThread);

    zero3.setZero();
    Zero.setZero(4,4);
    desired_twist.setZero();
    centroidal_momentum.setZero(6);
}

void orocos_opensot_ik::setWorld(const KDL::Frame& l_sole_T_Waist, Eigen::VectorXd& q)
{
    _model->setFloatingBasePose(l_sole_T_Waist);
    _model->update();
    _model->getJointPosition(q);
}

bool orocos_opensot_ik::startHook()
{
    _joystik_port.createStream(rtt_roscomm::topic("joy"));

    _q.setZero(_model->getJointNum());
    _dq.setZero(_model->getJointNum());

    _qm.setZero(_robot->getJointNum());
    _taum.setZero(_robot->getJointNum());

    std::cout<<"_qm: "<<_qm<<std::endl;

    sense(_qm, _taum);
    _q.segment(6,_qm.size()) = _qm;

    std::cout<<"_q: "<<_q<<std::endl;

    _model->setJointPosition(_q);
    _model->setJointVelocity(_dq);
    _model->update();

    std::cout<<"Mass: "<<_model->getMass()<<std::endl;

    //Update world according this new configuration:
    KDL::Frame l_sole_T_Waist;
    _model->getPose("Waist", "l_sole", l_sole_T_Waist);

    l_sole_T_Waist.p.x(0.0);
    l_sole_T_Waist.p.y(0.0);

    this->setWorld(l_sole_T_Waist, _q);


    foot_size<<0.2,0.1;//0.2,0.1;
    std::cout<<"foot_size: "<<foot_size<<std::endl;

    Eigen::Affine3d tmp;
    _model->getPose("l_ankle", "l_sole", tmp);

    ik.reset(new opensot_ik(_q, _model, this->getPeriod(), fabs(tmp(2,3)), foot_size));



    relative_activity = 50;
    std::cout<<"relative_activity: "<<relative_activity<<std::endl;
    double __dT = this->getPeriod()*relative_activity;
    std::cout<<"__dT: "<<__dT<<std::endl;
    update_counter = 1;
    _wpg.reset(new legged_robot::Walker(*_model, __dT, 1., 0.3, //1.5, 0.6//1., 0.3
                                        foot_size,
                                        "l_sole", "r_sole", "Waist",
                                        3,
                                        2e2,2e3,1e3));
    _wpg->setStepHeight(_step_height);
    _wpg->setFootSpan(_wpg->getFootSpan());//0.8
    next_state = _wpg->getCurrentState();
    integrator.set(_wpg->getCurrentState(), next_state, _wpg->getDuration(), this->getPeriod());

    Eigen::Vector3d com;
    _model->getCOM(com);
    Eigen::Affine3d waist;
    _model->getPose("Waist", waist);

    offset = -com + waist.translation();


    return true;
}



void orocos_opensot_ik::updateHook()
{
    RTT::os::TimeService::ticks start = RTT::os::TimeService::Instance()->getTicks();

    std::chrono::milliseconds start1 = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());

    //RTT::log(RTT::Error)<<"orocos time: "<<start<<"     system time: "<<start1.count()<<RTT::endlog();



    _logger->add("q", _q);
    sense(_qm, _taum);
    _logger->add("qm", _qm);
    _logger->add("taum", _taum);


    RTT::FlowStatus fs = _joystik_port.read(joystik_msg);
    if(fs != 0)
    {
        desired_twist.setZero();
        joystick.getWalkingReferences(joystik_msg, desired_twist, _step_height);
        _wpg->setStepHeight(_step_height);
        _wpg->setReference(desired_twist.segment(0,2));
    }

    _model->setJointPosition(_q);
    _model->setJointVelocity(_dq/this->getPeriod());
    _model->update();

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
    //_out.com.pos += offset;
    ik->setWalkingReferences(_out, _robot->getForceTorque());
    integrator.Output().log(_logger, "integrator");
    _out.log(_logger, "integrator_stabilized");


    //ik->waist->update(_q);
    ik->stack->update(_q);
    ik->stack->log(_logger);
    //ik->com_z->update(_q);
    //ik->capture_point->update(_q);

    if(!ik->iHQP->solve(_dq)){
        _dq.setZero(_dq.size());
        std::cout<<"iHQP can not solve"<<std::endl;}


    _q+=_dq;

    move(_q.segment(6,_qm.size()));

    RTT::os::TimeService::Seconds time = RTT::os::TimeService::Instance()->secondsSince(start);

//    RTT::log(RTT::Info)<<time<<RTT::endlog();
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


