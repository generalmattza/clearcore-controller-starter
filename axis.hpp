#ifndef __AXIS_HPP__
#define __AXIS_HPP__

#include "ClearCore.h"
#include "motor.hpp"

/**
 * @brief Represents an axis that controls a motor.
 *
 * This class provides an interface to initialize the axis, move at a specified velocity,
 * limit motor torque, and retrieve current position, velocity, and torque information.
 */
class Axis
{
private:
    MCMotor *motor;             ///< Pointer to the motor associated with this axis.
    double gearbox_ratio;          ///< Drive ratio for the axis.
    double leadscrew_ratio;     // Ratio for the leadscrew
    double position_current;    ///< Current position of the axis.
    double velocity_current;    ///< Current velocity of the axis.
    double torque_current;      ///< Current torque of the axis.
    double motor_direction_ref; ///< Motor direction multiplier (-1.0 or 1.0).
    double velocity_limit;      ///< Maximum allowable velocity.
    double torque_limit;        ///< [Deprecated or unused variable?]
    double torque_limit_max;    ///< Maximum torque limit.
    double torque_limit_min;    ///< Minimum torque limit.

public:
    /**
     * @brief Construct a new Axis object.
     *
     * @param motor Pointer to an MCMotor object representing the motor.
     * @param gearbox_ratio The drive ratio of the axis.
     * @param leadscrew_ratio The drive ratio of the axis.
     * @param velocity_limit The maximum velocity allowed.
     * @param torque_limit_max The maximum torque limit (default is 1.0).
     * @param torque_limit_min The minimum torque limit (default is 0.0).
     * @param motor_direction_ref Motor direction reference flag (default is true). If true, motor direction is reversed (-1.0).
     */
    Axis(MCMotor *motor, double gearbox_ratio, double leadscrew_ratio, double velocity_limit, double torque_limit_max = 1.0, double torque_limit_min = 0.0, bool motor_direction_ref = true)
    {
        this->motor = motor;
        this->gearbox_ratio = gearbox_ratio;
        this->leadscrew_ratio = leadscrew_ratio;
        this->motor_direction_ref = motor_direction_ref ? 1.0 : -1.0;
        this->position_current = 0;
        this->velocity_limit = velocity_limit;
        this->torque_limit_max = torque_limit_max;
        this->torque_limit_min = torque_limit_min;
    }

    /**
     * @brief Initialize the axis.
     */
    void init(void);

    /**
     * @brief Limit the motor torque.
     *
     * @param torque_limit_command The commanded torque limit.
     */
    void limitMotorTorque(double torque_limit_command);

    /**
     * @brief Move the axis at a specified velocity.
     *
     * @param velocity_command The velocity command.
     * @return int32_t Return status of the operation.
     */
    int32_t MoveAtVelocity(double velocity_command);

    /**
     * @brief Zero the current position of the axis.
     */
    void zeroPosition(void);

    /**
     * @brief Read the current position of the axis.
     *
     * @return double The current position.
     */
    double readCurrentPosition(void);

    /**
     * @brief Get the current motor torque.
     *
     * @return double The current motor torque.
     */
    float getMotorTorque(void);

    /**
     * @brief Get the current motor velocity.
     *
     * @return double The current motor velocity.
     */
    double getMotorVelocity(void);

    /**
     * @brief Get the current calculated position.
     *
     * This value is adjusted by the drive ratio and motor direction.
     *
     * @return double The current position.
     */
    double getPositionCurrent(void);

    /**
     * @brief Get the current calculated velocity.
     *
     * @return double The current velocity.
     */
    double getVelocityCurrent(void);

    int32_t getAlerts(void) const;
    const char* getStatusName(void) const;
};

#endif // __AXIS_HPP__
