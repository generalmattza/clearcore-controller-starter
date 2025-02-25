#ifndef __AXIS_HPP__
#define __AXIS_HPP__

#include "ClearCore.h"
#include "motor.hpp"

#define adcResolution 12

class Axis
{
    MCMotor *motor;
    float ratio;
    AnalogIn *torque_limit_control_pin;
    AnalogIn *velocity_limit_control_pin;
    double position_current;
    double velocity_current;
    double motor_direction_ref;

public:
    Axis(MCMotor *motor, float ratio, AnalogIn *torque_limit_control_pin, AnalogIn *velocity_limit_control_pin, bool motor_direction_ref = true)
    {
        this->motor = motor;
        this->ratio = ratio;
        this->torque_limit_control_pin = torque_limit_control_pin;
        this->velocity_limit_control_pin = velocity_limit_control_pin;
        this->motor_direction_ref = motor_direction_ref ? -1.0 : 1.0;
    }

    void initAxis(void);
    double readTorqueCommand(void);
    double readVelocityCommand(void);
    void limitMotorTorque(double torque_limit_command) const;
    int32_t commandMotorVelocity(double velocity_command) const;
    void zeroPosition(void);
    double readCurrentPosition(void) const;
}

#endif // __AXIS_HPP__