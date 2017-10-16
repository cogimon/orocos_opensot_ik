#ifndef _OPENSOT_IK_H
#define _OPENSOT_IK_H

#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/AngularMomentum.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/CapturePoint.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/QPOases.h>
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
               const double dT);

    void setWalkingReferences(const legged_robot::AbstractVariable &next_state,
                              const std::map<std::string, rstrt::dynamics::Wrench> frames_wrenches_map);

    Cartesian::Ptr left_leg;
    Cartesian::Ptr right_leg;
    Cartesian::Ptr waist;
    CoM::Ptr com;
    CapturePointConstraint::Ptr capture_point;
    CartesianPositionConstraint::Ptr com_z;
    AngularMomentum::Ptr mom;

    double _dT;

    JointLimits::Ptr joint_lims;
    VelocityLimits::Ptr joint_vel_lims;

    AutoStack::Ptr stack;

    QPOases_sot::Ptr iHQP;

    Eigen::VectorXd desired_twist;
    Eigen::MatrixXd desired_pose;

    CompliantStabilizer stabilizer;
};


#endif
