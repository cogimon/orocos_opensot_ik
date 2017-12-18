#include <opensot_ik.h>

#define lambda 100.

Eigen::Vector3d getGains(const double x, const double y, const double z)
{
    Eigen::Vector3d tmp;
    tmp<<x,y,z;
    return tmp;
}

opensot_ik::opensot_ik(const Eigen::VectorXd &q,
                       const XBot::ModelInterface::Ptr model,
                       const double dT, const double ankle_height,
                       const Eigen::Vector2d& foot_size):
    _dT(dT),
    desired_twist(6),
    stabilizer(dT, model->getMass(), ankle_height, foot_size,
               10.,
               //getGains(0.1,0.2,0.), getGains(-0.005,-0.005,0.),
               getGains(0.1,0.1,0.), getGains(-0.005,-0.005,0.),
               getGains(DEFAULT_MaxLimsx, DEFAULT_MaxLimsy, DEFAULT_MaxLimsz),
               getGains(DEFAULT_MinLimsx, DEFAULT_MinLimsy, DEFAULT_MinLimsz))
{
    left_leg.reset(new Cartesian("left_leg", *model, "l_sole", "world", q));
    left_leg->setLambda(lambda);
    right_leg.reset(new Cartesian("right_leg", *model, "r_sole", "world", q));
    right_leg->setLambda(lambda);
    com.reset(new CoM(*model, q));
    com->setLambda(lambda);
    std::list<unsigned int> orient;
    orient.push_back(3);
    orient.push_back(4);
    orient.push_back(5);
    waist.reset(new Cartesian("waist", *model, "Waist", "world", q));
    waist->setLambda(lambda);
    postural.reset(new Postural("postural", *model, q.size()));
    postural->setLambda(lambda);

    OpenSoT::AffineHelper var = OpenSoT::AffineHelper::Identity(model->getJointNum());
    Eigen::VectorXd ddqmax = 10.*Eigen::VectorXd::Ones(model->getJointNum());
    Eigen::VectorXd ddqmin = -ddqmax;
    joint_acc_lims.reset(
        new GenericConstraint("acc_lims", var, ddqmax, ddqmin, GenericConstraint::Type::BOUND));



    stack = ((left_leg + right_leg)/(com + waist%orient)/postural)<<joint_acc_lims;


//      iHQP.reset(new QPOases_sot(stack->getStack(), stack->getBounds(),capture_point, 1e5));
    iHQP.reset(new QPOases_sot(stack->getStack(), stack->getBounds(), 1e6));




    desired_twist.setZero(6);
    desired_pose.Identity();
}

void opensot_ik::setWalkingReferences(legged_robot::AbstractVariable &next_state,
                                      const std::map< std::string, XBot::ForceTorqueSensor::ConstPtr > frames_wrenches_map)
{

    desired_twist.setZero(6);
    desired_pose.Identity();

    desired_twist[0] = next_state.lsole.vel[0];
    desired_twist[1] = next_state.lsole.vel[1];
    desired_twist[2] = next_state.lsole.vel[2];

    desired_pose.matrix()(0,3) = next_state.lsole.pos[0];
    desired_pose.matrix()(1,3) = next_state.lsole.pos[1];
    desired_pose.matrix()(2,3) = next_state.lsole.pos[2];

    left_leg->setReference(desired_pose);

    desired_twist.setZero(6);
    desired_pose.Identity();

    desired_twist[0] = next_state.rsole.vel[0];
    desired_twist[1] = next_state.rsole.vel[1];
    desired_twist[2] = next_state.rsole.vel[2];

    desired_pose.matrix()(0,3) = next_state.rsole.pos[0];
    desired_pose.matrix()(1,3) = next_state.rsole.pos[1];
    desired_pose.matrix()(2,3) = next_state.rsole.pos[2];

    right_leg->setReference(desired_pose);

    Eigen::Vector2d CopPos_L, CopPos_R;
    CopPos_L(0) = next_state.zmp[0] - next_state.lsole.pos[0];
    CopPos_L(1) = next_state.zmp[1] - next_state.lsole.pos[1];

    CopPos_R(0) = next_state.zmp[0] - next_state.rsole.pos[0];
    CopPos_R(1) = next_state.zmp[1] - next_state.rsole.pos[1];


    Eigen::Vector6d left_wrench;
    frames_wrenches_map.at("l_leg_ft")->getWrench(left_wrench);
    Eigen::Vector6d right_wrench;
    frames_wrenches_map.at("r_leg_ft")->getWrench(right_wrench);
    Eigen::Vector3d delta_com = stabilizer.update(left_wrench, right_wrench,
                                                  CopPos_L, CopPos_R,
                                                  next_state.lsole.pos, next_state.rsole.pos);

    next_state.com.pos += delta_com;
    olddelta=delta_com;
    //next_state.com.vel += (delta_com-olddelta)/_dT;
    next_state.com.vel += delta_com/_dT;
    //com->setReference(next_state.com.pos, next_state.com.vel*_dT);
    com->setReference(next_state.com.pos);
}

