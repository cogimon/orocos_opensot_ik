#include <opensot_ik.h>

opensot_ik::opensot_ik(const Eigen::VectorXd &q,
                       const XBot::ModelInterface::Ptr model,
                       const double dT)
{
    {
        left_leg.reset(new Cartesian("left_leg", q, *model, "l_sole", "world"));
        left_leg->setLambda(1.);
        right_leg.reset(new Cartesian("right_leg", q, *model, "r_sole", "world"));
        right_leg->setLambda(1.);
        com.reset(new CoM(q, *model));
        com->setLambda(1.);

        waist.reset(new Cartesian("waist", q, *model, "Waist", "world"));
        waist->setLambda(1.);
        SubTask::Ptr waist_orientation;
        std::list<unsigned int> idx = {3,4,5};
        waist_orientation.reset(new SubTask(waist,idx));
        waist_orientation->setLambda(1.);

        mom.reset(new AngularMomentum(q, *model));
        Eigen::Vector6d L;
        model->getCentroidalMomentum(L);
        mom->setReference(dT*L.segment(3,3));

        Eigen::MatrixXd A(4,2);
        A << Eigen::MatrixXd::Identity(2,2),
             -1.*Eigen::MatrixXd::Identity(2,2);
        Eigen::VectorXd b(4);
        b<<0.03, 0.1, 0.1, 0.1;
        capture_point.reset(new CapturePointConstraint(q, com, *model, A, b, dT,0.1));
        capture_point->computeAngularMomentumCorrection(true);

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

        joint_vel_lims.reset(new VelocityLimits(2., dT, q.size()));

        stack = ((left_leg + right_leg)/(com + waist_orientation))<<joint_lims<<joint_vel_lims;

//        iHQP.reset(new QPOases_sot(stack->getStack(), stack->getBounds(),capture_point, 1e5));
        iHQP.reset(new QPOases_sot(stack->getStack(), stack->getBounds(), 1e8));
    }

}
