#ifndef __AXIS_HPP__
#define __AXIS_HPP__

#include "ClearCore.h"
#include "motor.hpp"


class Axis
{
private:
    MCMotor *motor;
    float drive_ratio;
    double position_current;
    double velocity_current;
    double torque_current;
    double motor_direction_ref;
    double velocity_limit;
    double torque_limit;

public:
    Axis(MCMotor *motor, float drive_ratio, double velocity_limit, double torque_limit = 1.0, bool motor_direction_ref = true)
    {
        this->motor = motor;
        this->drive_ratio = drive_ratio;
        this->motor_direction_ref = motor_direction_ref ? -1.0 : 1.0;
        this->position_current = 0;
        this->velocity_limit = velocity_limit;
        this->torque_limit = torque_limit;
    }

    void init(void);
    void limitMotorTorque(double torque_limit_command);
    int32_t MoveAtVelocity(double velocity_command);
    void zeroPosition(void);
    double readCurrentPosition(void);
    double getMotorTorque(void);
    double getMotorVelocity(void);
    double getPositionCurrent(void);
    double getVelocityCurrent(void);

};

#endif // __AXIS_HPP__