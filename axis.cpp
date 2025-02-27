#include "axis.hpp"

/**
 * @brief Initializes the axis and its associated motor.
 *
 * Calls the motor's initialization routine.
 */
void Axis::init(void)
{
    motor->initMotor();
    // analogReadResolution(adcResolution);
}

/**
 * @brief Moves the axis at the specified velocity command.
 *
 * This function scales the input velocity command by the velocity limit and drive ratio,
 * then commands the motor to move at that velocity.
 *
 * @param velocity_command The desired velocity command.
 * @return int32_t The scaled motor velocity that was commanded.
 */
int32_t Axis::MoveAtVelocity(double velocity_command)
{
    velocity_current = velocity_command * velocity_limit;
    int32_t motor_velocity = velocity_current * drive_ratio;
    motor->MoveAtVelocity(motor_velocity);
    return motor_velocity;
}

/**
 * @brief Limits the motor torque based on the provided torque limit command.
 *
 * This function computes a torque command using the maximum and minimum torque limits,
 * applies a scaling factor, and then sets the motor's torque limit.
 *
 * @param torque_limit_command The commanded torque limit.
 */
void Axis::limitMotorTorque(double torque_limit_command)
{
    double torque_command = torque_limit_min + (torque_limit_command * (torque_limit_max - torque_limit_min));
    motor->LimitTorque(torque_command);
    torque_current = torque_command;
}

/**
 * @brief Reads and returns the current position of the axis.
 *
 * The position is calculated by adjusting the motor's current position by the drive ratio
 * and motor direction.
 *
 * @return double The current position of the axis.
 */
double Axis::readCurrentPosition(void)
{
    position_current = motor_direction_ref * motor->getPositionCurrent() / drive_ratio;
    return position_current;
}

/**
 * @brief Resets the current position of the axis to zero.
 *
 * Calls the motor's zeroPosition function and updates the internal position.
 */
void Axis::zeroPosition(void)
{
    motor->zeroPosition();
    position_current = 0.0;
}

/**
 * @brief Retrieves the current motor torque.
 *
 * @return double The current torque measured from the motor.
 */
double Axis::getMotorTorque(void)
{
    return torque_current;
}

/**
 * @brief Retrieves the current motor velocity in RPM.
 *
 * @return double The current velocity of the motor.
 */
double Axis::getMotorVelocity(void)
{
    return (double)motor->getVelocityCurrent();
}

/**
 * @brief Returns the internally stored current position.
 *
 * @return double The current position of the axis.
 */
double Axis::getPositionCurrent(void)
{
    return position_current;
}

/**
 * @brief Returns the internally stored current velocity.
 *
 * @return double The current velocity of the axis.
 */
double Axis::getVelocityCurrent(void)
{
    return velocity_current;
}
