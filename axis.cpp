
#include "axis.hpp"

void Axis::initAxis(void)
{
    this->motor->initMotor();

}

double Axis::readVelocityCommand(void) const
{
    int velocity_command_raw = analogRead(this->velocity_limit_control_pin);
    double velocity_command = 1.0 * adcResult / ((1 << adcResolution) - 1);
    return velocity_command;
}

double Axis::readTorqueCommand(void) const
{
    int torque_command_raw = analogRead(this->torque_limit_control_pin);
    double torque_command = 1.0 * adcResult / ((1 << adcResolution) - 1);
    return torque_command;
}

