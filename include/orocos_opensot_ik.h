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
#include <XBotInterface/ModelInterface.h>

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

    std::string _config_path;
    std::string _robot_name;

    XBot::ModelInterface::Ptr _model;

    bool _model_loaded;
    bool _ports_loaded;

    Eigen::VectorXd _q;
    Eigen::VectorXd _dq;

    std::map<std::string, std::vector<std::string> > _map_kin_chains_joints;
    std::vector<std::string> _force_torque_sensors_frames;

    std::map<std::string, boost::shared_ptr<RTT::InputPort<rstrt::robot::JointState> > > _kinematic_chains_feedback_ports;
    std::map<std::string, rstrt::robot::JointState> _kinematic_chains_joint_state_map;

    std::map<std::string, boost::shared_ptr<RTT::InputPort<rstrt::dynamics::Wrench> > > _frames_ports_map;
    std::map<std::string, rstrt::dynamics::Wrench> _frames_wrenches_map;
};

#endif
