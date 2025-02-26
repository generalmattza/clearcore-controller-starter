
#include "axis.hpp"

void Axis::init(void)
{
    motor->initMotor();
    // analogReadResolution(adcResolution);

}


int32_t Axis::MoveAtVelocity(double velocity_command)
{
    velocity_current = velocity_command * velocity_limit;
    int32_t motor_velocity = velocity_current * drive_ratio;
    motor->MoveAtVelocity(motor_velocity);
    return motor_velocity;
}

void Axis::limitMotorTorque(double torque_limit_command)
{
    double torque_command = torque_limit_command * torque_limit;
    motor->LimitTorque(torque_command);
    torque_current = torque_command;
}

double Axis::readCurrentPosition(void)
{
    position_current = motor_direction_ref * motor->getPositionCurrent() / drive_ratio;
    return position_current;
}

void Axis::zeroPosition(void)
{
    motor->zeroPosition();
    position_current = 0.0;
}

double Axis::getMotorTorque(void)
{
    return torque_current;
}
// Return the current velocity of the motor in rpm
double Axis::getMotorVelocity(void)
{
    return (double)motor->getVelocityCurrent();
}

double Axis::getPositionCurrent(void)
{
    return position_current;
}

double Axis::getVelocityCurrent(void)
{
    return velocity_current;
}