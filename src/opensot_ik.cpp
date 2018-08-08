#include <opensot_ik.h>

#define lambda 0.3

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
    desired_pose(4,4),
    stabilizer(dT, model->getMass(), ankle_height, foot_size,
               10.,
               //getGains(0.1,0.2,0.), getGains(-0.005,-0.005,0.),
               getGains(0.1,0.1,0.), getGains(-0.005,-0.005,0.),
               getGains(DEFAULT_MaxLimsx, DEFAULT_MaxLimsy, DEFAULT_MaxLimsz),
               getGains(DEFAULT_MinLimsx, DEFAULT_MinLimsy, DEFAULT_MinLimsz))
{
    left_leg.reset(new Cartesian("left_leg", q, *model, "l_sole", "world"));
    left_leg->setLambda(lambda);
    right_leg.reset(new Cartesian("right_leg", q, *model, "r_sole", "world"));
    right_leg->setLambda(lambda);
    com.reset(new CoM(q, *model));
    com->setLambda(lambda);

    waist.reset(new Cartesian("waist", q, *model, "Waist", "world"));
    waist->setLambda(lambda);
    SubTask::Ptr waist_orientation;
    std::list<unsigned int> idx = {3,4,5};
    waist_orientation.reset(new SubTask(waist,idx));
    waist_orientation->setLambda(lambda);

    mom.reset(new AngularMomentum(q, *model));
    mom->setWeight(0.02*Eigen::MatrixXd(3,3).Identity(3,3));
    Eigen::Vector6d L;
    model->getCentroidalMomentum(L);
    mom->setReference(dT*L.segment(3,3));

    Eigen::MatrixXd A(4,2);
    A << Eigen::MatrixXd::Identity(2,2),
         -1.*Eigen::MatrixXd::Identity(2,2);
    Eigen::VectorXd b(4);
    b<<0.03, 0.1, 0.1, 0.1;
//    capture_point.reset(new CapturePointConstraint(q, com, *model, A, b, dT,0.1));
//    capture_point->computeAngularMomentumCorrection(true);

    Eigen::MatrixXd A2(2,3);
    A2 << 0, 0,  1,
          0, 0,  -1;
    Eigen::VectorXd b2(2);
    b2<< 0.51, -0.4;
    com_z.reset(new CartesianPositionConstraint(q, com, A2, b2, 0.1));
    //waist->getConstraints().push_back(com_z);

    Eigen::VectorXd qmin, qmax;
    model->getJointLimits (qmin, qmax);
    qmin[model->getDofIndex("RKneePitch")] = 0.3;
    qmin[model->getDofIndex("LKneePitch")] = 0.3;
    joint_lims.reset(new JointLimits(q, qmax, qmin));

    joint_vel_lims.reset(new VelocityLimits(3., dT, q.size()));

    stack = ((left_leg + right_leg)/(com + mom))<<joint_lims<<joint_vel_lims;


//      iHQP.reset(new QPOases_sot(stack->getStack(), stack->getBounds(),capture_point, 1e5));
    iHQP = boost::make_shared<OpenSoT::solvers::iHQP>(stack->getStack(), stack->getBounds(), 1e10);




    desired_twist.setZero(6);
    desired_pose.setIdentity(4,4);
}

void opensot_ik::setWalkingReferences(legged_robot::AbstractVariable &next_state,
                                      const std::map< std::string, XBot::ForceTorqueSensor::ConstPtr > frames_wrenches_map)
{

    desired_twist.setZero(6);
    desired_pose.setIdentity(4,4);

    desired_twist[0] = next_state.lsole.vel[0];
    desired_twist[1] = next_state.lsole.vel[1];
    desired_twist[2] = next_state.lsole.vel[2];

    desired_pose(0,3) = next_state.lsole.pos[0];
    desired_pose(1,3) = next_state.lsole.pos[1];
    desired_pose(2,3) = next_state.lsole.pos[2];

    left_leg->setReference(desired_pose, desired_twist*_dT);

    desired_twist.setZero(6);
    desired_pose.setIdentity(4,4);

    desired_twist[0] = next_state.rsole.vel[0];
    desired_twist[1] = next_state.rsole.vel[1];
    desired_twist[2] = next_state.rsole.vel[2];

    desired_pose(0,3) = next_state.rsole.pos[0];
    desired_pose(1,3) = next_state.rsole.pos[1];
    desired_pose(2,3) = next_state.rsole.pos[2];

    right_leg->setReference(desired_pose, desired_twist*_dT);

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

    Eigen::MatrixXd T = waist->getActualPose();
    T(0,3) = next_state.com.pos[0];
    T(1,3) = next_state.com.pos[1];
    T(2,3) = next_state.com.pos[2];
    waist->setReference(T);
}

