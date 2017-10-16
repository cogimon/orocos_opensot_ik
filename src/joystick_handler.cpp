#include <joystick_handler.h>

#define DEFAULT_WALKING_VEL_MAX_X 0.3
#define DEFAULT_WALKING_VEL_MAX_Y 0.3*1.5

joystick_handler::joystick_handler():
    _walking_vmax_x(DEFAULT_WALKING_VEL_MAX_X),
    _walking_vmax_y(DEFAULT_WALKING_VEL_MAX_Y)
{

}

joystick_handler::~joystick_handler()
{

}

bool joystick_handler::setWalkingVelMax(const double vmax_x, const double vmax_y)
{
    if(vmax_x < 0. || vmax_y < 0.)
        return false;
    _walking_vmax_x = vmax_x;
    _walking_vmax_y = vmax_y;
    return true;
}

bool joystick_handler::setWalkingVelMax(const double vmax)
{
    return setWalkingVelMax(vmax, vmax);
}

void joystick_handler::getWalkingVelMax(double &v_max_x, double &v_max_y)
{
    v_max_x = _walking_vmax_x;
    v_max_y = _walking_vmax_y;
}

void joystick_handler::getWalkingReferences(const sensor_msgs::Joy &msg,
                                            Eigen::Vector6d& desired_twist)
{
    desired_twist[1] = _walking_vmax_y*msg.axes[0];
    desired_twist[0] = _walking_vmax_x*msg.axes[1];
}

