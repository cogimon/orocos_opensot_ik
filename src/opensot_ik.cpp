#include <opensot_ik.h>

#define lambda 50.
#define orientation_gain 0.1
#define lambda2 0.5*sqrt(lambda)

Eigen::Vector3d getGains(const double x, const double y, const double z)
{
    Eigen::Vector3d tmp;
    tmp<<x,y,z;
    return tmp;
}

opensot_ik::opensot_ik(const XBot::ModelInterface::Ptr model,
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
    OpenSoT::OptvarHelper::VariableVector variable_name_dims;
    variable_name_dims.emplace_back("qddot", model->getJointNum());
    variable_name_dims.emplace_back("left_wrench", 6);
    variable_name_dims.emplace_back("right_wrench", 6);
    OpenSoT::OptvarHelper serializer(variable_name_dims);
    qddot = serializer.getVariable("qddot");
    legs_wrench.push_back(serializer.getVariable("left_wrench"));
    legs_wrench.push_back(serializer.getVariable("right_wrench"));


    left_leg.reset(new Cartesian("left_leg", *model, "l_sole", "world", qddot));
    left_leg->setLambda(lambda,lambda2);
    right_leg.reset(new Cartesian("right_leg", *model, "r_sole", "world", qddot));
    right_leg->setLambda(lambda,lambda2);
    com.reset(new CoM(*model, qddot));
    com->setLambda(10., 0.5*sqrt(10.));
    std::list<unsigned int> orient;
    orient.push_back(3);
    orient.push_back(4);
    orient.push_back(5);
    waist.reset(new Cartesian("waist", *model, "Waist", "world", qddot));
    waist->setLambda(lambda,lambda2);
    waist->setOrientationGain(orientation_gain);
    postural.reset(new Postural(*model, qddot));
    postural->setLambda(lambda,lambda2);
    std::list<unsigned int> id;
    id.push_back(12);


    AffineHelper I = AffineHelper::Identity(serializer.getSize());
    Eigen::VectorXd xmax = 20.*Eigen::VectorXd::Ones(serializer.getSize());
    xmax[model->getJointNum()] = xmax[model->getJointNum()+6] = 1000;
    xmax[model->getJointNum()+1] = xmax[model->getJointNum()+7]= 1000;
    xmax[model->getJointNum()+2] = xmax[model->getJointNum()+8]= 1000;

    xmax[model->getJointNum()+3] = xmax[model->getJointNum()+9] = 1000;
    xmax[model->getJointNum()+4] = xmax[model->getJointNum()+10]= 1000;
    xmax[model->getJointNum()+5] = xmax[model->getJointNum()+11]= 1000;


    Eigen::VectorXd xmin = -xmax;
    xmin[model->getJointNum()+2] = xmin[model->getJointNum()+8]= 0;


    x_lims.reset(
        new GenericConstraint("acc_wrench_lims", I, xmax, xmin, GenericConstraint::Type::BOUND));


//    Eigen::MatrixXd COP = Eigen::MatrixXd::Zero(2,serializer.getSize());
//    Eigen::VectorXd cop_max = Eigen::VectorXd::Zero(2);
//    Eigen::VectorXd cop_min = Eigen::VectorXd::Zero(2);
//    Eigen::Vector3d p; com->getReference(p);
//    Eigen::Affine3d l,r; left_leg->getActualPose(l); right_leg->getActualPose(r);
//    COP.block(0,model->getJointNum(), 1, 12)<<0,0,l(0,3),0,-1,0,0,0,r(0,3),0,-1,0;
//    COP.block(1,model->getJointNum(), 1, 12)<<0,0,l(1,3),1,0,0,0,0,r(1,3),1,0,0;

//    AffineHelper A(COP, Eigen::VectorXd::Zero(2));

//    GenericConstraint::Ptr COP_constr(
//        new GenericConstraint("COP_constr", A, cop_max, cop_min, GenericConstraint::Type::CONSTRAINT));


    std::vector<std::string> contacts = {"l_sole", "r_sole"};
    OpenSoT::constraints::acceleration::DynamicFeasibility::Ptr dynamics(
        new OpenSoT::constraints::acceleration::DynamicFeasibility("Dynamics", *model, qddot,
                                                                   legs_wrench, contacts));




    stack = ((left_leg + right_leg)/(com + waist%orient)/(postural%id))<<x_lims<<dynamics;


    iHQP.reset(new QPOases_sot(stack->getStack(), stack->getBounds(), 1e2));




    desired_twist.setZero(6);
    desired_pose.Identity();
}

void opensot_ik::setWalkingReferences(legged_robot::AbstractVariable &next_state,
                                      const std::map< std::string, XBot::ForceTorqueSensor::ConstPtr > frames_wrenches_map)
{

    desired_twist.setZero(6);
    left_leg->getReference(desired_pose);

    desired_twist[0] = next_state.lsole.vel[0];
    desired_twist[1] = next_state.lsole.vel[1];
    desired_twist[2] = next_state.lsole.vel[2];

    desired_pose.matrix()(0,3) = next_state.lsole.pos[0];
    desired_pose.matrix()(1,3) = next_state.lsole.pos[1];
    desired_pose.matrix()(2,3) = next_state.lsole.pos[2];

    left_leg->setReference(desired_pose);

    desired_twist.setZero(6);
    right_leg->getReference(desired_pose);

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

