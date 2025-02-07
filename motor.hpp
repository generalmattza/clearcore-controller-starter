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
  int32_t velocity_limit;
  int32_t velocity_normal;
  int32_t velocity_fine;
  int32_t velocity_rapid;
  int32_t accel_limit;
  int32_t decel_limit;
  int32_t estop_decel_limit;
  char motorName[16];
  int32_t motor_ppr;

public:
  MotorDriver *connector;

  Motor(MotorDriver *connector, const char *motorName, MotorParameters *m_params)
      : connector(connector), velocity_limit(m_params->velocity_limit),
        velocity_normal(m_params->velocity_normal), velocity_fine(m_params->velocity_fine),
        velocity_rapid(m_params->velocity_rapid), accel_limit(m_params->accel_limit),
        decel_limit(m_params->decel_limit), estop_decel_limit(m_params->estop_decel_limit),
        motor_ppr(m_params->motor_ppr)
  {
    setMotorName(motorName);
  }

  void initMotor(void) const;
  void clearErrors(void) const;
  bool MoveDistance(int distance) const;
  bool MoveAtVelocity(int32_t velocity) const;
  int32_t getAlerts(void) const;
  void clearFaults(void) const;
  void setMotorName(const char *name);
  int8_t getTorqueCurrent(void) const;
  int32_t getVelocityCurrent(void) const;
  int32_t getStatus(void) const;
  int32_t getPositionCurrent(void) const;
  bool handleAlerts(void) const;
  void disableMotor(void) const;
  void enableMotor(void) const;
  void stopMotor(void) const;
  void printMessage(const char *message) const;
  bool validateMove(bool negDirection = false) const;
  bool setLimitSwitchPositive(ClearCorePins pin);
  bool setLimitSwitchNegative(ClearCorePins pin);
  template <typename T>
  bool velocityMovementGuard(T velocity) const;
  int32_t getVelocityLimit(void) const
  {
    return velocity_limit;
  }
  int32_t getVelocityNormal(void) const
  {
    return velocity_normal;
  }
  int32_t getVelocityRapid(void) const
  {
    return velocity_rapid;
  }
  int32_t getVelocityFine(void) const
  {
    return velocity_fine;
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
  const char *getMotorName(void) const
  {
    return motorName;
  }
  void setPositionRef(int32_t position = 0) const;
  bool EStopConnector(ClearCorePins pin) const;
};

#endif // __MOTOR_HPP__
