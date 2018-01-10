#ifndef OROCOS_OPENSOT_IK_H
#define OROCOS_OPENSOT_IK_H

#include <rtt/RTT.hpp>
#include <string>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <urdf/model.h>
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/Wrench.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <XBotInterface/ModelInterface.h>

#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/AngularMomentum.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/CapturePoint.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/QPOases.h>

#include <sensor_msgs/Joy.h>

#include <compliant_stabilizer/compliantstabilizer.h>

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace OpenSoT;
using namespace OpenSoT::solvers;

class opensot_ik
{
public:
    opensot_ik(const Eigen::VectorXd& q, const XBot::ModelInterface::Ptr model,
               const double dT)
    {
        left_leg.reset(new Cartesian("left_leg", q, *model, "l_sole", "world"));
        left_leg->setLambda(0.1);
        right_leg.reset(new Cartesian("right_leg", q, *model, "r_sole", "world"));
        right_leg->setLambda(0.1);
        com.reset(new CoM(q, *model));
        com->setLambda(0.1);

        waist.reset(new Cartesian("waist", q, *model, "Waist", "world"));
        waist->setLambda(1.);
        //waist->setOrientationErrorGain(0.01);

        mom.reset(new AngularMomentum(q, *model));
        Eigen::Vector6d L;
        model->getCentroidalMomentum(L);
        mom->setReference(dT*L.segment(3,3));
//        Eigen::MatrixXd W(3,3);
//        W.setIdentity(3,3);
//        mom->setWeight(0.01*W);

        Eigen::MatrixXd A(4,2);
        A << Eigen::MatrixXd::Identity(2,2),
             -1.*Eigen::MatrixXd::Identity(2,2);
        Eigen::VectorXd b(4);
        b<<0.03, 0.09, 0.09, 0.09;
        capture_point.reset(new CapturePointConstraint(q, com, *model, A, b, dT,0.1));
        capture_point->computeAngularMomentumCorrection(true);

        Eigen::MatrixXd A2(2,3);
        A2 << 0, 0,  1,
              0, 0,  -1;
        Eigen::VectorXd b2(2);
        b2<< 0.51, -0.4;
        com_z.reset(new CartesianPositionConstraint(q, com, A2, b2, 0.1));
       // waist->getConstraints().push_back(com_z);

        Eigen::VectorXd qmin, qmax;
        model->getJointLimits (qmin, qmax);
        qmin[model->getDofIndex("RKneePitch")] = 0.3;
        qmin[model->getDofIndex("LKneePitch")] = 0.3;
        joint_lims.reset(new JointLimits(q, qmax, qmin));

        joint_vel_lims.reset(new VelocityLimits(10., dT, q.size()));

        std::list<unsigned int> id_com;
        id_com.push_back(0);
        id_com.push_back(1);
        std::list<unsigned int> id_waist;
        id_waist.push_back(2);
        id_waist.push_back(3);
        id_waist.push_back(4);
        id_waist.push_back(5);

        stack = ((left_leg + right_leg)/
                 (com%id_com)/
                 waist)//<<joint_lims
                      <<joint_vel_lims;

//        stack = (waist/(left_leg + right_leg)/
//                 (com%id_com))//<<joint_lims
//                      <<joint_vel_lims;

        iHQP.reset(new QPOases_sot(stack->getStack(), stack->getBounds(), 1e2));
    }

    Cartesian::Ptr left_leg;
    Cartesian::Ptr right_leg;
    Cartesian::Ptr waist;
    CoM::Ptr com;
    CapturePointConstraint::Ptr capture_point;
    CartesianPositionConstraint::Ptr com_z;
    AngularMomentum::Ptr mom;


    JointLimits::Ptr joint_lims;
    VelocityLimits::Ptr joint_vel_lims;

    AutoStack::Ptr stack;

    QPOases_sot::Ptr iHQP;

};

class orocos_opensot_ik: public RTT::TaskContext {
public:
    orocos_opensot_ik(std::string const & name);
    ~orocos_opensot_ik();

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
private:

    bool loadConfig(const std::string& config_path);
    bool attachToRobot(const std::string& robot_name);
    void sense(Eigen::VectorXd& q, Eigen::VectorXd &dq, Eigen::VectorXd &tau);
    void move(const Eigen::VectorXd& q);
    void setReferences(const sensor_msgs::Joy& msg);
    void setWorld(const KDL::Frame& l_sole_T_Waist, Eigen::VectorXd& q);
    void setVMax(const double vmax);

    std::string _config_path;
    std::string _robot_name;

    XBot::ModelInterface::Ptr _model;

    bool _model_loaded;
    bool _ports_loaded;

    Eigen::VectorXd _q;
    Eigen::VectorXd _dq;

    Eigen::VectorXd _q_m;
    Eigen::VectorXd _dq_m;
    Eigen::VectorXd _tau_m;


    std::map<std::string, std::vector<std::string> > _map_kin_chains_joints;
    std::vector<std::string> _force_torque_sensors_frames;

    std::map<std::string, boost::shared_ptr<RTT::InputPort<rstrt::robot::JointState> > > _kinematic_chains_feedback_ports;
    std::map<std::string, rstrt::robot::JointState> _kinematic_chains_joint_state_map;

    std::map<std::string, boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> > > _kinematic_chains_output_ports;
    std::map<std::string, rstrt::kinematics::JointAngles> _kinematic_chains_desired_joint_state_map;

    std::map<std::string, boost::shared_ptr<RTT::InputPort<rstrt::dynamics::Wrench> > > _frames_ports_map;
    std::map<std::string, rstrt::dynamics::Wrench> _frames_wrenches_map;

    boost::shared_ptr<opensot_ik> ik;

    RTT::InputPort<sensor_msgs::Joy> _joystik_port;
    sensor_msgs::Joy joystik_msg;

    Eigen::Vector6d centroidal_momentum;
    Eigen::Vector6d desired_twist;
    Eigen::Vector3d zero3;
    Eigen::MatrixXd Zero;

    double _v_max;

    XBot::MatLogger::Ptr _logger;

    Eigen::Vector3d getGains(const double x, const double y, const double z);
    void stabilizer();

    boost::shared_ptr<CompliantStabilizer> _stabilizer;

    Eigen::Vector3d _com_initial;

    double _jump_time;
    double _jump_counter;
};

#endif
