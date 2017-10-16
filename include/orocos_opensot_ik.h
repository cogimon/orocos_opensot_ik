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


#include <opensot_ik.h>

#include <joystick_handler.h>

class orocos_opensot_ik: public RTT::TaskContext {
public:
    orocos_opensot_ik(std::string const & name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
private:

    bool loadConfig(const std::string& config_path);
    bool attachToRobot(const std::string& robot_name);
    void sense(Eigen::VectorXd& q);
    void move(const Eigen::VectorXd& q);
    void setReferences(const sensor_msgs::Joy& msg);
    void setWorld(const KDL::Frame& l_sole_T_Waist, Eigen::VectorXd& q);
    void logRobot(const XBot::ModelInterface::Ptr robot);

    std::string _config_path;
    std::string _robot_name;

    XBot::ModelInterface::Ptr _model;

    bool _model_loaded;
    bool _ports_loaded;

    Eigen::VectorXd _q;
    Eigen::VectorXd _qm;
    Eigen::VectorXd _dq;


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
    joystick_handler joystick;

    Eigen::Vector6d centroidal_momentum;
    Eigen::Vector6d desired_twist;
    Eigen::Vector3d zero3;
    Eigen::MatrixXd Zero;

    Eigen::Affine3d tmp;
    Eigen::Vector2d foot_size;

    double _v_max;

    boost::shared_ptr<legged_robot::Walker> _wpg;
    legged_robot::AbstractVariable next_state;
    int update_counter;
    int relative_activity;
    legged_robot::Integrator integrator;



    XBot::MatLogger::Ptr _logger;
};

#endif
