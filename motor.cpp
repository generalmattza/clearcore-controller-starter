/**
 * @file motor.cpp
 * @brief Implementation file for Motor, SDMotor, and MCMotor classes.
 *
 * This file implements functions to manage motor operations including
 * initialization, error handling, movement control, and interrupt handling.
 */

#include "motor.hpp"
#include <type_traits> // For std::is_same

// Global pointer for instance access in the interrupt callback (assumes a single instance)
static Motor* gMotorInstance = nullptr;

/**
 * @brief Static wrapper for the position pulse callback.
 *
 * This function is provided to attach to an interrupt since attachInterrupt
 * requires a plain function pointer. It calls the non-static member function
 * positionPulseCallback on the global instance.
 */
static void positionPulseCallbackWrapper() {
    if (gMotorInstance != nullptr) {
        gMotorInstance->positionPulseCallback();
    }
}

// -------------------- Motor Class Implementation --------------------

/**
 * @brief Prints a formatted message for the motor.
 *
 * If the message is null, a default message is printed.
 *
 * @param message The message to print.
 */
void Motor::printMessage(const char *message) const {
    if (message == nullptr) {
        message = "Invalid message (null).";
    }
    char buffer[100]; // Adjust size based on the maximum expected message length
    snprintf(buffer, sizeof(buffer), "Motor %s: %s", motorName, message);
    Serial.println(buffer);
}

/**
 * @brief Initializes the motor with default HLFB settings.
 *
 * Sets the HLFB mode to bipolar PWM and the carrier frequency.
 */
void Motor::initMotor(void) {
    // Set the motor's HLFB mode to bipolar PWM
    connector->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    // Set the HLFB carrier frequency to 482 Hz
    connector->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

}

/**
 * @brief Performs post-initialization steps for the motor.
 *
 * Enables the motor, waits for homing to complete, clears errors,
 * and prints an initialization message.
 */
void Motor::postInitMotor(void) const {
    // Enable the motor; homing will begin automatically if enabled
    connector->EnableRequest(true);

    // Wait until homing is complete or an alert is present
    while ((!connector->StepsComplete() || connector->HlfbState() != MotorDriver::HLFB_ASSERTED) &&
           !connector->StatusReg().bit.AlertsPresent) {
        continue;
    }

    clearErrors();
    printMessage("Motor initialized.");
}

/**
 * @brief Clears motor errors.
 *
 * If alerts are present, stops the motor abruptly, clears faults, and prints a message.
 */
void Motor::clearErrors(void) const {
    if (connector->StatusReg().bit.AlertsPresent) {
        connector->MoveStopAbrupt();
        clearFaults();
        printMessage("Motor errors cleared.");
    }
}

/**
 * @brief Handles motor alerts.
 *
 * Clears faults if necessary and returns whether the motor is alert-free.
 *
 * @return true if no alerts are present or alerts have been cleared; false otherwise.
 */
bool Motor::handleAlerts(void) const {
    if (connector->StatusReg().bit.AlertsPresent) {
        // Motor alert detected.
        return false;
    } else if (HANDLE_ALERTS) {
        clearFaults();
        return true;
    } else {
        return true;
    }
}

/**
 * @brief Clears motor faults by cycling the enable signal and clearing alerts.
 */
void Motor::clearFaults() const {
    if (connector->AlertReg().bit.MotorFaulted) {
        // If a motor fault is present, clear it by cycling the enable signal.
        connector->EnableRequest(false);
        Delay_ms(10);
        connector->EnableRequest(true);
    }
    // Clear alerts and re-enable the motor.
    connector->ClearAlerts();
    enableMotor();
}

/**
 * @brief Retrieves the current alert status.
 *
 * Each bit in the returned integer corresponds to a specific alert.
 *
 * @return int32_t The alert status as a bitfield.
 */
int32_t Motor::getAlerts() const {
    int32_t alerts = 0;
    alerts |= (connector->AlertReg().bit.MotionCanceledInAlert ? 1 : 0) << 0;
    alerts |= (connector->AlertReg().bit.MotionCanceledPositiveLimit ? 1 : 0) << 1;
    alerts |= (connector->AlertReg().bit.MotionCanceledNegativeLimit ? 1 : 0) << 2;
    alerts |= (connector->AlertReg().bit.MotionCanceledSensorEStop ? 1 : 0) << 3;
    alerts |= (connector->AlertReg().bit.MotionCanceledMotorDisabled ? 1 : 0) << 4;
    alerts |= (connector->AlertReg().bit.MotorFaulted ? 1 : 0) << 5;
    return alerts;
}

/**
 * @brief Sets the motor's name.
 *
 * @param name The new motor name.
 */
void Motor::setMotorName(const char *name) {
    strncpy(motorName, name, sizeof(motorName) - 1);
    motorName[sizeof(motorName) - 1] = '\0';
}

/**
 * @brief Retrieves the motor's status.
 *
 * @return int32_t The ready state from the motor's status register.
 */
int32_t Motor::getStatus(void) const {
    return connector->StatusReg().bit.ReadyState;
}

/**
 * @brief Disables the motor by stopping it and disabling the enable request.
 */
void Motor::disableMotor(void) {
    this->stopMotor();
    connector->EnableRequest(false);
}

/**
 * @brief Enables the motor.
 */
void Motor::enableMotor(void) const {
    connector->EnableRequest(true);
}

/**
 * @brief Stops the motor by commanding a zero velocity.
 */
void Motor::stopMotor(void) {
    this->MoveAtVelocity(0);
}

/**
 * @brief Configures the positive limit switch.
 *
 * @param pin The ClearCore pin to use.
 * @return true if the limit switch is set successfully; false otherwise.
 */
bool Motor::setLimitSwitchPositive(ClearCorePins pin) {
    return connector->LimitSwitchPos(pin);
}

/**
 * @brief Configures the negative limit switch.
 *
 * @param pin The ClearCore pin to use.
 * @return true if the limit switch is set successfully; false otherwise.
 */
bool Motor::setLimitSwitchNegative(ClearCorePins pin) {
    return connector->LimitSwitchNeg(pin);
}

/**
 * @brief Retrieves the HLFB percentage.
 *
 * If a measurement is available, returns the HLFB percentage as a percent of the motor peak value.
 *
 * @return int8_t The HLFB percentage, or 0 if unavailable.
 */
int8_t Motor::getHlfbPercent(void) const {
    MotorDriver::HlfbStates hlfbState = connector->HlfbState();
    if (hlfbState == MotorDriver::HLFB_HAS_MEASUREMENT) {
        return int(round(connector->HlfbPercent()));
    }
    return 0;
}

/**
 * @brief Interrupt callback for position pulses.
 *
 * This function is called by the interrupt handler and increments the motor's position.
 */
void Motor::positionPulseCallback(void) {
    this->incrementPosition();
}



// -------------------- SDMotor Class Implementation --------------------

/**
 * @brief Initializes the SDMotor with specific motion parameters.
 *
 * Calls the base initialization, sets velocity and acceleration limits,
 * and performs post-initialization.
 */
void SDMotor::initMotor(void) {
    Motor::initMotor();
    connector->VelMax(getVelocityLimit());
    connector->AccelMax(getAccelLimit());
    connector->EStopDecelMax(getEstopDecelLimit());
    Motor::postInitMotor();
}

/**
 * @brief Retrieves the current commanded position.
 *
 * @return int32_t The current position reference.
 */
int32_t SDMotor::getPositionCurrent(void) const {
    return connector->PositionRefCommanded();
}

/**
 * @brief Retrieves the current commanded velocity.
 *
 * @return int32_t The current velocity reference.
 */
int32_t SDMotor::getVelocityCurrent(void) const {
    return connector->VelocityRefCommanded();
}

/**
 * @brief Validates whether a move is allowed in a specified direction.
 *
 * @param negDirection True if moving in the negative direction.
 * @return true if the move is valid; false otherwise.
 */
bool SDMotor::validateMove(bool negDirection) const {
    return connector->ValidateMove(negDirection);
}

/**
 * @brief Sets the motor's position reference.
 *
 * @param position The new position reference.
 */
void SDMotor::setPositionRef(int32_t position) const {
    connector->PositionRefSet(position);
}

/**
 * @brief Stops the SDMotor by decelerating.
 */
void SDMotor::stopMotor(void) {
    connector->MoveStopDecel(getDecelLimit());
}

/**
 * @brief Guards velocity movement commands by validating the move.
 *
 * Checks if a velocity command is within valid limits.
 *
 * @tparam T The type of the velocity parameter.
 * @param velocity The commanded velocity.
 * @return true if the move is valid; false otherwise.
 */
template <typename T>
bool SDMotor::velocityMovementGuard(T velocity) const {
    bool negDirection = (velocity < 0);
    if (!validateMove(negDirection)) {
        // Move not allowed; motor is at its limit.
        return false;
    }
    return true;
}

/**
 * @brief Checks the state of the emergency stop connector.
 *
 * @param pin The ClearCore pin associated with the emergency stop.
 * @return true if the emergency stop is engaged; false otherwise.
 */
bool SDMotor::EStopConnector(ClearCorePins pin) const {
    return connector->EStopConnector(pin);
}

/**
 * @brief Commands an incremental distance move.
 *
 * Checks for alerts before and after the move.
 *
 * @param distance The incremental distance to move.
 * @return true if the move was successful; false otherwise.
 */
bool SDMotor::MoveDistance(int distance) const {
    if (!handleAlerts()) {
        return false;
    }
    connector->Move(distance);
    while ((!connector->StepsComplete() || connector->HlfbState() != MotorDriver::HLFB_ASSERTED) &&
           !connector->StatusReg().bit.AlertsPresent) {
        continue;
    }
    if (!handleAlerts()) {
        return false;
    }
    return true;
}

/**
 * @brief Commands the motor to move at a specified velocity.
 *
 * Checks for alerts before issuing the command.
 *
 * @param velocity The commanded velocity.
 * @return true if the command was accepted; false otherwise.
 */
bool SDMotor::MoveAtVelocity(int32_t velocity) {
    if (!handleAlerts()) {
        return false;
    }
    connector->MoveVelocity(velocity);
    return true;
}

// -------------------- MCMotor Class Implementation --------------------

/**
 * @brief Initializes the MCMotor and sets up the position pulse interrupt.
 *
 * Configures the pin mode, attaches the interrupt, and performs post-initialization.
 */
void MCMotor::initMotor(void) {
    Motor::initMotor();

    position_pulse_pin.Mode(Connector::INPUT_DIGITAL);

    // Set the global instance pointer for the interrupt callback
    gMotorInstance = this;
    position_pulse_pin.InterruptHandlerSet(positionPulseCallbackWrapper, InputManager::FALLING, false);
    position_pulse_pin.InterruptEnable(true);
    Motor::postInitMotor();
}

/**
 * @brief Stops the MCMotor by commanding a zero velocity.
 */
void MCMotor::stopMotor(void) {
    this->MoveAtVelocity(0);
}

/**
 * @brief Commands the motor to move at a specified velocity.
 *
 * Scales the velocity command to a PWM duty cycle and updates the current velocity.
 *
 * @param velocity The commanded velocity.
 * @return true if the command was accepted; false otherwise.
 */
bool MCMotor::MoveAtVelocity(int32_t velocity) {
    if (abs(velocity) > this->velocity_limit) {
        printMessage("Move rejected, invalid velocity requested.");
        return false;
    }

    // Calculate PWM duty cycle scaling
    double range_unsigned = 127.5 - (this->pwm_deadband * 255);
    double scale_factor = range_unsigned / this->velocity_limit;
    double duty_request;

    if (velocity < 0) {
        duty_request = 127.5 - (this->pwm_deadband * 255) + (velocity * scale_factor);
    } else if (velocity > 0) {
        duty_request = 127.5 + (this->pwm_deadband * 255) + (velocity * scale_factor);
    } else {
        duty_request = 128.0;
    }

    connector->MotorInBDuty(duty_request);

    this->velocity_current = velocity;
    return true;
}

/**
 * @brief Commands the motor to limit the applied torque.
 *
 * Scales the torque limit command to a PWM duty cycle and sends it.
 * 
 *
 * @param limit The torque limit percentage as a factor 0-1.
 * @return true if the command was accepted; false otherwise.
 */
bool MCMotor::LimitTorque(double limit) {
    if ((limit > torque_limit_max) || (limit < torque_limit_min)) {
        printMessage("Torque limiting rejected, invalid torque requested.");
        return false;
    }
    double scale_factor = 255.0 / (torque_limit_max - torque_limit_min);
    uint8_t duty_request = (torque_limit_max - limit) * scale_factor;
    connector->MotorInADuty(duty_request);
    torque_limit_current = limit;
    return true;
}

/**
 * @brief Retrieves the current commanded velocity.
 *
 * @return int32_t The current commanded velocity.
 */
int32_t MCMotor::getVelocityCurrent(void) const {
    return this->velocity_current;
}

uint8_t MCMotor::getMotorDirection(void) const
{
    static uint8_t last_direction = 1; // default direction (-1 for CW, 1 for CCW)

    if (this->velocity_current > 0)
    {
        last_direction = 1;
    }
    else if (this->velocity_current < 0)
    {
        last_direction = -1;
    }
    // If velocity_current is 0, we simply return the last known direction.
    return last_direction;
}

/**
 * @brief Retrieves the current commanded position.
 *
 * @return int32_t The current commanded position.
 */
int32_t MCMotor::getPositionCurrent(void) const {
    return this->position_current;
}

/**
 * @brief Increments the motor's current position.
 *
 * @param increment The amount by which to increment the position.
 */
void MCMotor::incrementPosition(int32_t increment) {
    int8_t direction = this->getMotorDirection();
    this->position_current += direction * increment;
}

/**
 * @brief Zeroes the motor's position.
 * @param position The new position reference.
 */
void MCMotor::zeroPosition(int32_t position) {
    this->position_current = position;
}
