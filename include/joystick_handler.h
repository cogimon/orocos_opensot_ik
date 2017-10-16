#ifndef _JOYSTICK_HANDLER_H_
#define _JOYSTICK_HANDLER_H_

#include <sensor_msgs/Joy.h>
#include <mpcqp_walking/walker.h>

class joystick_handler
{
public:
    joystick_handler();
    ~joystick_handler();

    bool setWalkingVelMax(const double vmax);
    bool setWalkingVelMax(const double vmax_x, const double vmax_y);

    void getWalkingVelMax(double& v_max_x, double& v_max_y);

    void setWalkingReferences(const sensor_msgs::Joy &msg,
                              Eigen::Vector6d& desired_twist);


private:

    double _walking_vmax_x;
    double _walking_vmax_y;

};

#endif
