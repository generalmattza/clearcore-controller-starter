#ifndef __AXIS_HPP__
#define __AXIS_HPP__

#include "ClearCore.h"
#include "motor.hpp"

class Axis
{
    Motor *motor;
    float ratio;
    AnalogIn *torque_limit_control_pin;
    AnalogIn *velocity_limit_control_pin;

public:
    Axis(Motor *motor, float ratio, AnalogIn *torque_limit_control_pin, AnalogIn *velocity_limit_control_pin)
    {
        this->motor = motor;
        this->ratio = ratio;
        this->torque_limit_control_pin = torque_limit_control_pin;
        this->velocity_limit_control_pin = velocity_limit_control_pin;
    }

    void initAxis(void);
    double readTorqueCommand(void);
    double readVelocityCommand(void);


}
#endif // __AXIS_HPP__