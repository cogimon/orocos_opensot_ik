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
    _ddq(),
    _model_loaded(false),
    _ports_loaded(false),
    Zero(4,4),
    _step_height(0.08)
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
    _joystik_port.createStream(rtt_roscomm::topic("joy"));

    _q.setZero(_model->getJointNum());
    _dq.setZero(_model->getJointNum());
    _ddq.setZero(_model->getJointNum());

    _qm.setZero(_robot->getJointNum());
    _dqm.setZero(_robot->getJointNum());
    _taum.setZero(_robot->getJointNum());

    std::cout<<"_qm: "<<_qm<<std::endl;

    sense(_qm ,_dqm,_taum);
    _q.segment(6,_qm.size()) = _qm;
    _dq.segment(6,_dqm.size()) = _dqm;

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


    std::cout<<"_q: "<<_q<<std::endl;


    std::vector<std::string> contacts;
    contacts.push_back("l_sole");
    contacts.push_back("r_sole");
    XBot::ImuSensor::ConstPtr imu;
    _model_m->setJointPosition(_q);
    _model_m->setJointVelocity(_dq);
    fb.reset(
        new OpenSoT::floating_base_estimation::qp_estimation(_model_m, imu, contacts));

    _model->getJointPosition(_q);
    _model->getJointVelocity(_dq);
    std::cout<<"_q: "<<_q<<std::endl;
    std::cout<<"_dq: "<<_dq<<std::endl;

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
    _wpg.reset(new legged_robot::Walker(*_model, __dT, 1.5, 0.6, //1.5, 0.6//1., 0.3
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

    _Qik.setZero(_q.size());
    _dQik.setZero(_q.size());
    _Qik<<_q.segment(0,6),_qm;
    _dQik<<_dq.segment(0,6),_dqm;

    return true;
}



void orocos_opensot_ik::updateHook()
{
    _logger->add("q", _q);
    _logger->add("dq", _dq);
    sense(_qm,_dqm, _taum);
    _logger->add("qm", _qm);
    _logger->add("dqm", _dqm);
    _logger->add("taum", _taum);

    _Qik<<_Qik.segment(0,6),_qm;
    _dQik<<_dQik.segment(0,6),_dqm;
    _model_m->setJointPosition(_Qik);
    _model_m->setJointVelocity(_dQik);
    //_model_m->update();

//    _q.segment(6, _qm.size()) = _qm;
//    _dq.segment(6, _dqm.size()) = _dqm;


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
    //_out.com.pos += offset;
    ik->setWalkingReferences(_out, _robot->getForceTorque());
    integrator.Output().log(_logger, "integrator");
    _out.log(_logger, "integrator_stabilized");

    Eigen::Vector6d left_wrench;
    _robot->getForceTorque().at("l_leg_ft")->getWrench(left_wrench);
    Eigen::Vector6d right_wrench;
    _robot->getForceTorque().at("r_leg_ft")->getWrench(right_wrench);
    if(left_wrench[2] >= 20.){
        if(!fb->setContactState("l_sole", true))
            std::cout<<"setContactState(l_sole, true) returned false!"<<std::endl;
    }
    else{
        if(!fb->setContactState("l_sole", false))
            std::cout<<"setContactState(l_sole, false) returned false!"<<std::endl;
    }
    if(right_wrench[2] >= 20.){
        if(!fb->setContactState("r_sole", true))
            std::cout<<"setContactState(r_sole, true) returned false!"<<std::endl;
    }
    else{
        if(!fb->setContactState("r_sole", false))
            std::cout<<"setContactState(r_sole, false) returned false!"<<std::endl;
    }

    _model->setJointPosition(_q);
    _model->setJointVelocity(_dq);
    _model->update();

    if(!fb->update(this->getPeriod()))
        std::cout<<"fb QP can not solve"<<std::endl;
    fb->log(_logger);
    _model_m->getJointPosition(_Qik);
    _model_m->getJointVelocity(_dQik);




    ik->stack->update(_q);
    ik->stack->log(_logger);

    if(!ik->iHQP->solve(_ddq)){
        _ddq.setZero(_ddq.size());
        std::cout<<"iHQP can not solve"<<std::endl;}


//    _dQik += _ddq.segment(0,6)*this->getPeriod();
//    _Qik += _dQik*this->getPeriod() + 0.5*_ddq.segment(0,6)*(this->getPeriod()*this->getPeriod());
//    _logger->add("_dQik", _dQik);
//    _logger->add("_Qik", _Qik);

//    _dq.segment(6,_dq.size()-6) += _ddq.segment(6,_dq.size()-6)*this->getPeriod();
//    _q.segment(6,_dq.size()-6) += _dq.segment(6,_dq.size()-6)*this->getPeriod() +
//            0.5*_ddq.segment(6,_dq.size()-6)*(this->getPeriod()*this->getPeriod());
    _dq += _ddq*this->getPeriod();
    _q += _dq*this->getPeriod() + 0.5*_ddq*(this->getPeriod()*this->getPeriod());


    move(_q.segment(6,_q.size()-6));


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


