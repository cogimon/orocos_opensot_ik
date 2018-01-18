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

#include <mpcqp_walking/walker.h>
#include <mpcqp_walking/integrator.h>

#include <XBotInterface/RobotInterface.h>

#include <opensot_ik.h>
#include <OpenSoT/floating_base_estimation/qp_estimation.h>

#include <joystick_handler.h>
#include <rst-rt/geometry/Pose.hpp>
#include <rst-rt/kinematics/Twist.hpp>

#define GROUND_TRUTH_GAZEBO true

class orocos_opensot_ik: public RTT::TaskContext {
public:
    orocos_opensot_ik(std::string const & name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
private:

    bool attachToRobot(const std::string &robot_name, const std::string &config_path);
    void sense(Eigen::VectorXd& q, Eigen::VectorXd& dq, Eigen::VectorXd& tau);
    void move(const Eigen::VectorXd& q);
    void setReferences(const sensor_msgs::Joy& msg);
    void setWorld(const KDL::Frame& l_sole_T_Waist, Eigen::VectorXd& q);
    void logRobot(const XBot::ModelInterface::Ptr robot);

    std::string _config_path;
    std::string _robot_name;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;

    bool _model_loaded;
    bool _ports_loaded;

    Eigen::VectorXd _qm;
    Eigen::VectorXd _dqm;
    Eigen::VectorXd _taum;


    boost::shared_ptr<opensot_ik> ik;

    RTT::InputPort<sensor_msgs::Joy> _joystik_port;
    sensor_msgs::Joy joystik_msg;
    joystick_handler joystick;

    Eigen::Vector6d desired_twist;
    Eigen::Vector3d zero3;
    Eigen::MatrixXd Zero;

    Eigen::Affine3d tmp;
    Eigen::Vector2d foot_size;

    double _v_max;

    double _step_height;

    boost::shared_ptr<legged_robot::Walker> _wpg;
    legged_robot::AbstractVariable next_state;
    int update_counter;
    int relative_activity;
    legged_robot::Integrator integrator;



    XBot::MatLogger::Ptr _logger;


    legged_robot::AbstractVariable _out;
    Eigen::Vector3d offset;

    bool _compute_fb;

    bool _change_control_mode;

    Eigen::VectorXd _tau;

//#if GROUND_TRUTH_GAZEBO
    void FloatingBaseFromGazebo(Eigen::Affine3d& fb_pose, Eigen::Vector6d& fb_twist);
    Eigen::Affine3d _fb_offset;
    RTT::OperationCaller<bool(const std::string&, rstrt::geometry::Pose&,
                              rstrt::kinematics::Twist&)> getLinkPoseVelocityGazebo;
    boost::shared_ptr<TaskContext> _task_peer_ptr;
//#endif
};

#endif
