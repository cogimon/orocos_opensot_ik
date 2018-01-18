#ifndef _OPENSOT_IK_H
#define _OPENSOT_IK_H

#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/utils/Affine.h>
#include <mpcqp_walking/walker.h>
#include <rst-rt/dynamics/Wrench.hpp>

#include <compliant_stabilizer/compliantstabilizer.h>

#include <OpenSoT/constraints/acceleration/DynamicFeasibility.h>

using namespace OpenSoT::tasks::acceleration;
using namespace OpenSoT::constraints;
using namespace OpenSoT;
using namespace OpenSoT::solvers;

class opensot_ik
{
public:
    opensot_ik(const XBot::ModelInterface::Ptr model,
               const double dT, const double ankle_height,
               const Eigen::Vector2d& foot_size);

    void setWalkingReferences(legged_robot::AbstractVariable &next_state,
                              const std::map< std::string, XBot::ForceTorqueSensor::ConstPtr > frames_wrenches_map);

    Cartesian::Ptr left_leg;
    Cartesian::Ptr right_leg;
    Cartesian::Ptr waist;
    CoM::Ptr com;
    Postural::Ptr postural;




    double _dT;

    GenericConstraint::Ptr x_lims;

    AutoStack::Ptr stack;

    QPOases_sot::Ptr iHQP;

    Eigen::VectorXd desired_twist;
    Eigen::Affine3d desired_pose;

    CompliantStabilizer stabilizer;
    Vector3d olddelta;

    AffineHelper qddot;
    std::vector<AffineHelper> legs_wrench;

};


#endif
