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

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace OpenSoT;
using namespace OpenSoT::solvers;

class opensot_ik
{
public:
    opensot_ik(const Eigen::VectorXd& q, const XBot::ModelInterface::Ptr model,
               const double dT);

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


#endif
