#ifndef _OPENSOT_IK_H
#define _OPENSOT_IK_H

#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
//#include <OpenSoT/constraints/velocity/CapturePoint.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/SubTask.h>
#include <mpcqp_walking/walker.h>
#include <rst-rt/dynamics/Wrench.hpp>

#include <compliant_stabilizer/compliantstabilizer.h>

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace OpenSoT;
using namespace OpenSoT::solvers;

class opensot_ik
{
public:
    opensot_ik(const Eigen::VectorXd& q, const XBot::ModelInterface::Ptr model,
               const double dT, const double ankle_height,
               const Eigen::Vector2d& foot_size);

    void setWalkingReferences(legged_robot::AbstractVariable &next_state,
                              const std::map< std::string, XBot::ForceTorqueSensor::ConstPtr > frames_wrenches_map);

    Cartesian::Ptr left_leg;
    Cartesian::Ptr right_leg;
    Cartesian::Ptr waist;

    Cartesian::Ptr left_arm;
    Cartesian::Ptr right_arm;

    CoM::Ptr com;

    Postural::Ptr postural;

    double _dT;

    JointLimits::Ptr joint_lims;
    VelocityLimits::Ptr joint_vel_lims;

    AutoStack::Ptr stack;

    OpenSoT::solvers::iHQP::Ptr iHQP;

    Eigen::VectorXd desired_twist;
    Eigen::MatrixXd desired_pose;

    CompliantStabilizer stabilizer;
    Vector3d olddelta;
};


#endif
