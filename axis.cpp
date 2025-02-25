
#include "axis.hpp"

void Axis::initAxis(void)
{
    this->motor->initMotor();

}

double Axis::readVelocityCommand(void) const
{
    int adc_result = analogRead(this->velocity_limit_control_pin);
    double velocity_command = 1.0 * adc_result / ((1 << adcResolution) - 1);
    return velocity_command;
}

double Axis::readTorqueCommand(void) const
{
    int adc_result = analogRead(this->torque_limit_control_pin);
    double torque_command = 1.0 * adc_result / ((1 << adcResolution) - 1);
    return torque_command;
}


int32_t Axis::commandMotorVelocity(double velocity_command) const
{
    int32_t motor_velocity = velocity_command * this->ratio;
    this->motor->MoveAtVelocity(motor_velocity);
    return motor_velocity;
}

void Axis::limitMotorTorque(double torque_limit_command) const
{
    this->motor->LimitTorque(torque_limit_command);
}

double Axis::readCurrentPosition(void) const
{
    this->position_current = this->motor_direction_ref * this->motor->getPositionCurrent() / this->ratio;
    return this->position_current;
}

void Axis::zeroPosition(void)
{
    this->motor->zeroPosition();
    this->position_current = 0.0;
}