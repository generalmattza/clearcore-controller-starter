#ifndef __MOTOR_HPP_
#define __MOTOR_HPP_

#include "ClearCore.h"
#include <Arduino.h>
#include "motor_parameters.hpp"

#define HANDLE_ALERTS true

/**
 * @brief Class to manage motor operation, including movement and status.
 *
 * This class is responsible for controlling the motor, including initialization,
 * movement commands, monitoring status, and handling alerts.
 */
class Motor
{
  char motorName[16];

public:
  MotorDriver *connector;

  Motor(MotorDriver *connector, const char *motorName)
      : connector(connector)
  {
    setMotorName(motorName);
  }

  void initMotor(void) const;
  void clearErrors(void) const;
  int32_t getAlerts(void) const;
  void clearFaults(void) const;
  void setMotorName(const char *name);
  int32_t getStatus(void) const;
  bool handleAlerts(void) const;
  void disableMotor(void) const;
  void enableMotor(void) const;
  void printMessage(const char *message) const;
  bool setLimitSwitchPositive(ClearCorePins pin);
  bool setLimitSwitchNegative(ClearCorePins pin);
  const char *getMotorName(void) const
  {
    return motorName;
  }
};

class SDMotor: public Motor
{
  int32_t velocity_limit;
  int32_t accel_limit;
  int32_t decel_limit;
  int32_t estop_decel_limit;
  int32_t motor_ppr;

public:
  SDMotor(MotorDriver *connector, const char *motorName, SDMotorParameters *m_params)
      : Motor(connector, motorName), // Call base class initializer
        velocity_limit(m_params->velocity_limit),
        accel_limit(m_params->accel_limit),
        decel_limit(m_params->decel_limit),
        estop_decel_limit(m_params->estop_decel_limit),
        motor_ppr(m_params->motor_ppr)
  {
    setMotorName(motorName);
  }

  bool MoveDistance(int distance) const;
  bool MoveAtVelocity(int32_t velocity) const;
  int8_t getTorqueCurrent(void) const;
  int32_t getVelocityCurrent(void) const;
  int32_t getPositionCurrent(void) const;
  void stopMotor(void) const;
  bool validateMove(bool negDirection = false) const;
  template <typename T>
  bool velocityMovementGuard(T velocity) const;
  int32_t getVelocityLimit(void) const
  {
    return velocity_limit;
  }
  int32_t getAccelLimit(void) const
  {
    return accel_limit;
  }
  int32_t getDecelLimit(void) const
  {
    return decel_limit;
  }
  int32_t getEstopDecelLimit(void) const
  {
    return estop_decel_limit;
  }
  void setPositionRef(int32_t position = 0) const;
  bool EStopConnector(ClearCorePins pin) const;
};

class MCMotor: public Motor
{
  double velocity_limit;

public:
  MCMotor(MotorDriver *connector, const char *motorName, MCMotorParameters *m_params)
      : Motor(connector, motorName), // Call base class initializer
        velocity_limit(m_params->velocity_limit),
  {
    setMotorName(motorName);
  }

  bool LimitTorque(double limit);
  bool MoveAtVelocity(int32_t velocity);
};
#endif // __MOTOR_HPP__
