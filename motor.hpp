#ifndef __MOTOR_HPP__
#define __MOTOR_HPP__

#include "ClearCore.h"
// #include "MotorDriver.h"
#include <Arduino.h>
#include "motor_parameters.hpp"

#define HANDLE_ALERTS true


/**
 * @brief Base class for managing motor operations.
 *
 * This class is responsible for controlling the motor, including initialization,
 * movement commands, status monitoring, and error handling.
 */
class Motor {
protected:
    char motorName[16]; ///< Motor name

public:
    MotorDriver *connector; ///< Pointer to the motor driver connector

    /**
     * @brief Constructor for the Motor class.
     * @param connector Pointer to a MotorDriver instance.
     * @param motorName C-string representing the motor name.
     */
    Motor(MotorDriver *connector, const char *motorName)
        : connector(connector)
    {
        setMotorName(motorName);
    }

    /**
     * @brief Initializes the motor.
     */
    virtual void initMotor(void);

    /**
     * @brief Performs post-initialization steps for the motor.
     */
    virtual void postInitMotor(void) const;

    /**
     * @brief Clears any errors present on the motor.
     */
    void clearErrors(void) const;

    /**
     * @brief Retrieves the current alert status.
     * @return int32_t Bitfield representing alert status.
     */
    int32_t getAlerts(void) const;

    /**
     * @brief Clears motor faults.
     */
    void clearFaults(void) const;

    /**
     * @brief Sets the motor name.
     * @param name New motor name as a C-string.
     */
    void setMotorName(const char *name);

    /**
     * @brief Retrieves the motor's status.
     * @return int32_t Motor status from the status register.
     */
    int32_t getStatus(void) const;

    /**
     * @brief Retrieves the motor's status as a char.
     * @return const char* Motor status from the status register.
     */
    const char* getStatusName(void);

    /**
     * @brief Handles any motor alerts.
     * @return true if alerts are handled successfully; false otherwise.
     */
    bool handleAlerts(void) const;

    /**
     * @brief Disables the motor.
     */
    void disableMotor(void);

    /**
     * @brief Enables the motor.
     */
    void enableMotor(void) const;

    /**
     * @brief Prints a message related to the motor operation.
     * @param message C-string message to be printed.
     */
    void printMessage(const char *message) const;

    /**
     * @brief Configures the positive limit switch.
     * @param pin The ClearCore pin to use.
     * @return true if the limit switch is set successfully; false otherwise.
     */
    bool setLimitSwitchPositive(ClearCorePins pin);

    /**
     * @brief Configures the negative limit switch.
     * @param pin The ClearCore pin to use.
     * @return true if the limit switch is set successfully; false otherwise.
     */
    bool setLimitSwitchNegative(ClearCorePins pin);

    /**
     * @brief Retrieves the HLFB percentage.
     * @return int8_t The HLFB percentage, or 0 if not available.
     */
    int8_t getHlfbPercent(void) const;

    /**
     * @brief Retrieves the current commanded velocity.
     * @return int32_t The current commanded velocity.
     */
    virtual int32_t getVelocityCurrent(void) const = 0;

    /**
     * @brief Commands the motor to move at a specified velocity.
     * @param velocity Commanded velocity.
     * @return true if the command was accepted; false otherwise.
     */
    virtual bool MoveAtVelocity(int32_t velocity) = 0;

    /**
     * @brief Stops the motor.
     */
    virtual void stopMotor(void);

    /**
     * @brief Callback function for position pulse interrupts.
     */
    void positionPulseCallback(void);

    /**
     * @brief Increments the motor's position.
     * @param increment The amount to increment the position (default is 1).
     */
    virtual void incrementPosition(int32_t increment = 1) = 0;

    /**
     * @brief Retrieves the motor name.
     * @return C-string representing the motor name.
     */
    const char *getMotorName(void) const {
        return motorName;
    }

    // Function to get the enum name as a string
    const char* getMotorReadyStateName(MotorDriver::MotorReadyStates state);
};

/**
 * @brief Class for SD-type motors.
 *
 * Inherits from Motor and provides functionality specific to SD-type motors,
 * including distance moves, velocity control, and parameter handling.
 */
class SDMotor : public Motor {
protected:
    int32_t position_current;  ///< Current position command
    double velocity_current;   ///< Current velocity command (as double)
    int32_t velocity_limit;    ///< Maximum velocity limit
    int32_t accel_limit;       ///< Acceleration limit
    int32_t decel_limit;       ///< Deceleration limit
    int32_t estop_decel_limit; ///< Emergency stop deceleration limit
    int32_t motor_ppr;         ///< Pulses per revolution for the motor

public:
    /**
     * @brief Constructor for the SDMotor class.
     * @param connector Pointer to a MotorDriver instance.
     * @param motorName C-string representing the motor name.
     * @param m_params Pointer to SDMotorParameters struct.
     */
    SDMotor(MotorDriver *connector, const char *motorName, SDMotorParameters *m_params)
        : Motor(connector, motorName),
          velocity_limit(m_params->velocity_limit),
          accel_limit(m_params->accel_limit),
          decel_limit(m_params->decel_limit),
          estop_decel_limit(m_params->estop_decel_limit),
          motor_ppr(m_params->motor_ppr)
    {
        setMotorName(motorName);
    }

    /**
     * @brief Initializes the SDMotor.
     */
    virtual void initMotor(void);

    /**
     * @brief Commands an incremental distance move.
     * @param distance The distance to move.
     * @return true if the move was successful; false otherwise.
     */
    bool MoveDistance(int distance) const;

    /**
     * @brief Commands the motor to limit the applied torque.
     * @param limit The desired torque limit.
     * @return true if the command was accepted; false otherwise.
     */
    bool LimitTorque(double limit);

    /**
     * @brief Increments the current position.
     * @param increment The amount to increment (default is 1).
     */
    virtual void incrementPosition(int32_t increment = 1);

    /**
     * @brief Commands the motor to move at a specified velocity.
     * @param velocity Commanded velocity.
     * @return true if the command was accepted; false otherwise.
     */
    virtual bool MoveAtVelocity(int32_t velocity) override;

    /**
     * @brief Retrieves the motor direction.
     * @return int_8t The motor direction.
     */
    uint8_t getMotorDirection(void) const;

    /**
     * @brief Retrieves the current commanded velocity.
     * @return int32_t The current velocity command.
     */
    virtual int32_t getVelocityCurrent(void) const override;

    /**
     * @brief Retrieves the current position command.
     * @return int32_t The current position command.
     */
    int32_t getPositionCurrent(void) const;

    /**
     * @brief Stops the SDMotor.
     */
    virtual void stopMotor(void) override;

    /**
     * @brief Validates if a move in a given direction is allowed.
     * @param negDirection True for negative direction (default is false).
     * @return true if the move is valid; false otherwise.
     */
    bool validateMove(bool negDirection = false) const;

    /**
     * @brief Template guard for velocity movement commands.
     * @tparam T Type of the velocity parameter.
     * @param velocity The commanded velocity.
     * @return true if the movement is allowed; false otherwise.
     */
    template <typename T>
    bool velocityMovementGuard(T velocity) const;

    /**
     * @brief Retrieves the velocity limit.
     * @return int32_t The maximum velocity limit.
     */
    int32_t getVelocityLimit(void) const {
        return velocity_limit;
    }

    /**
     * @brief Retrieves the acceleration limit.
     * @return int32_t The acceleration limit.
     */
    int32_t getAccelLimit(void) const {
        return accel_limit;
    }

    /**
     * @brief Retrieves the deceleration limit.
     * @return int32_t The deceleration limit.
     */
    int32_t getDecelLimit(void) const {
        return decel_limit;
    }

    /**
     * @brief Retrieves the emergency stop deceleration limit.
     * @return int32_t The estop deceleration limit.
     */
    int32_t getEstopDecelLimit(void) const {
        return estop_decel_limit;
    }

    /**
     * @brief Sets the motor's position reference.
     * @param position The new position reference (default is 0).
     */
    void zeroPosition(int32_t position = 0);

    /**
     * @brief Checks the state of the emergency stop connector.
     * @param pin The ClearCore pin for emergency stop.
     * @return true if engaged; false otherwise.
     */
    bool EStopConnector(ClearCorePins pin) const;
};

/**
 * @brief Class for MC-type motors.
 *
 * Inherits from Motor and provides functionality specific to MC-type motors,
 * including interrupt handling for position pulses and torque limiting.
 */
class MCMotor : public Motor {
protected:
    int32_t velocity_limit;   // Maximum velocity limit for MC motors
    int32_t position_current; // Current position command
    double velocity_current;  // Current velocity command (as double)
    double torque_limit_max;      // maximum torque limit
    double torque_limit_min;      // Minimum torque limit
    double torque_limit_current;  // Current torque limit
    double pwm_deadband;      // PWM deadband value
    DigitalIn *position_pulse_pin = &ConnectorDI6; ///< Digital input for position pulse

public:
    /**
     * @brief Constructor for the MCMotor class.
     * @param connector Pointer to a MotorDriver instance.
     * @param motorName C-string representing the motor name.
     * @param m_params Pointer to MCMotorParameters struct.
     */
    MCMotor(MotorDriver *connector, const char *motorName, MCMotorParameters *m_params)
        : Motor(connector, motorName),
          velocity_limit(m_params->velocity_limit),
          torque_limit_max(m_params->torque_limit_max),
          torque_limit_min(m_params->torque_limit_min),
          pwm_deadband(m_params->pwm_deadband)
    {
        setMotorName(motorName);
    }

    /**
     * @brief Initializes the MCMotor and sets up the position pulse interrupt.
     */
    virtual void initMotor(void);

    /**
     * @brief Commands the motor to limit the applied torque.
     * @param limit The desired torque limit.
     * @return true if the command was accepted; false otherwise.
     */
    bool LimitTorque(double limit);

    /**
     * @brief Commands the motor to move at a specified velocity.
     * @param velocity Commanded velocity.
     * @return true if the command was accepted; false otherwise.
     */
    virtual bool MoveAtVelocity(int32_t velocity) override;

    /**
     * @brief Retrieves the current commanded velocity.
     * @return int32_t The current velocity command.
     */
    virtual int32_t getVelocityCurrent(void) const override;

    /**
     * @brief Retrieves the current position command.
     * @return int32_t The current position command.
     */
    int32_t getPositionCurrent(void) const;

    /**
     * @brief Increments the current position.
     * @param increment The amount to increment (default is 1).
     */
    virtual void incrementPosition(int32_t increment = 1) override;

    /**
     * @brief Sets the motor's position reference.
     * @param position The new position reference (default is 0).
     */
    void zeroPosition(int32_t position = 0);

    /**
     * @brief Stops the MCMotor.
     */
    virtual void stopMotor(void) override;


    /**
     * @brief Retrieves the motor direction.
     * @return int_8t The motor direction.
     */
    uint8_t getMotorDirection(void) const;
};

#endif // __MOTOR_HPP_
