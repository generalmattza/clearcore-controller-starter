#include "motor.hpp"
#include <type_traits> // For std::is_same

void Motor::printMessage(const char *message) const
{
  if (message == nullptr)
  {
    message = "Invalid message (null).";
  }

  char buffer[100]; // Adjust size based on max expected message length
  snprintf(buffer, sizeof(buffer), "Motor %s: %s", motorName, message);
  Serial.println(buffer); // Single call to Serial
}

void Motor::initMotor(void) const
{
  // Set the motor's HLFB mode to bipolar PWM
  connector->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);

  // Set the HFLB carrier frequency to 482 Hz
  connector->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

  // Sets the maximum velocity for each move
  connector->VelMax(getVelocityLimit());

  // Set the maximum acceleration for each move
  connector->AccelMax(getAccelLimit());

  connector->EStopDecelMax(getEstopDecelLimit());

  // Enables the motor; homing will begin automatically if enabled
  connector->EnableRequest(true);

  while ((!connector->StepsComplete() || connector->HlfbState() != MotorDriver::HLFB_ASSERTED) && !connector->StatusReg().bit.AlertsPresent)
  {
    continue;
  }

  clearErrors();

  // printMessage("Motor initialized.");
}

void Motor::clearErrors(void) const
{

  if (connector->StatusReg().bit.AlertsPresent)
  {
    connector->MoveStopAbrupt();
    clearFaults();
    // printMessage("Motor errors cleared.");
  }
}

bool Motor::MoveDistance(int distance) const
{
  // Check if a motor alert is currently preventing motion
  // Clear alert if configured to do so
  if (!handleAlerts())
  {
    return false;
  }

  // Command the move of incremental distance
  connector->Move(distance);

  // Waits for HLFB to assert (signaling the move has successfully completed)
  while ((!connector->StepsComplete() || connector->HlfbState() != MotorDriver::HLFB_ASSERTED) && !connector->StatusReg().bit.AlertsPresent)
  {
    continue;
  }
  // Check if motor alert occurred during move
  // Clear alert if configured to do so
  if (!handleAlerts())
  {
    return false;
  }
  return true;
}

bool Motor::MoveAtVelocity(int32_t velocity) const
{
  // Check if a motor alert is currently preventing motion
  // Clear alert if configured to do so
  if (!handleAlerts())
  {
    // Motor alert detected. Attempting to clear alert."
    return false;
  }

  // Command the velocity move
  connector->MoveVelocity(velocity);

  return true;
}

bool Motor::handleAlerts(void) const
{
  if (connector->StatusReg().bit.AlertsPresent)
  {
    // Motor alert detected. Attempting to clear alert."
    return false;
  }
  else if (HANDLE_ALERTS)
  {
    clearFaults();
    return true;
  }
  else
  {
    return true;
  }
}

void Motor::clearFaults() const
{
  if (connector->AlertReg().bit.MotorFaulted)
  {
    // if a motor fault is present, clear it by cycling enable
    // SerialPort.SendLine("Faults present. Cycling enable signal to motor to clear faults.");
    connector->EnableRequest(false);
    Delay_ms(10);
    connector->EnableRequest(true);
  }
  // clear alerts
  connector->ClearAlerts();
  // Enable motor
  enableMotor();
}
//------------------------------------------------------------------------------

int32_t Motor::getAlerts() const
{
  int32_t alerts = 0;

  // Set each bit based on the corresponding alert state
  alerts |= (connector->AlertReg().bit.MotionCanceledInAlert ? 1 : 0) << 0;
  alerts |= (connector->AlertReg().bit.MotionCanceledPositiveLimit ? 1 : 0) << 1;
  alerts |= (connector->AlertReg().bit.MotionCanceledNegativeLimit ? 1 : 0) << 2;
  alerts |= (connector->AlertReg().bit.MotionCanceledSensorEStop ? 1 : 0) << 3;
  alerts |= (connector->AlertReg().bit.MotionCanceledMotorDisabled ? 1 : 0) << 4;
  alerts |= (connector->AlertReg().bit.MotorFaulted ? 1 : 0) << 5;

  return alerts; // Return the integer representation of the alert states
}

void Motor::setMotorName(const char *name)
{
  strncpy(motorName, name, sizeof(motorName) - 1);
  motorName[sizeof(motorName) - 1] = '\0';
}

int32_t Motor::getStatus(void) const
{
  return connector->StatusReg().bit.ReadyState;
}

int8_t Motor::getTorqueCurrent(void) const
{
  MotorDriver::HlfbStates hlfbState = connector->HlfbState();
  // Write the HLFB state to the serial port
  if (hlfbState == MotorDriver::HLFB_HAS_MEASUREMENT)
  {
    // Writes the torque measured, as a percent of motor peak torque rating
    return int(round(connector->HlfbPercent()));
  }
  return 0;
}

int32_t Motor::getPositionCurrent(void) const
{
  return connector->PositionRefCommanded();
}

int32_t Motor::getVelocityCurrent(void) const
{
  return connector->VelocityRefCommanded();
}
void Motor::disableMotor(void) const
{
  connector->MoveStopDecel(getEstopDecelLimit());
  connector->EnableRequest(false);
}

void Motor::enableMotor(void) const
{
  connector->EnableRequest(true);
}

void Motor::stopMotor(void) const
{
  connector->MoveStopDecel(getDecelLimit());
}

bool Motor::validateMove(bool negDirection) const
{
  return connector->ValidateMove(negDirection);
}

void Motor::setPositionRef(int32_t position) const
{
  connector->PositionRefSet(position);
}

bool Motor::setLimitSwitchPositive(ClearCorePins pin)
{
  return connector->LimitSwitchPos(pin);
}
bool Motor::setLimitSwitchNegative(ClearCorePins pin)
{
  return connector->LimitSwitchNeg(pin);
}

template <typename T>
bool Motor::velocityMovementGuard(T velocity) const
{
  bool negDirection;
  negDirection = velocity < 0;

  if (!validateMove(negDirection))
  {
    // SerialPort.SendLine("Move not allowed. Motor is at limit.");
    return false;
  }
  return true;
}

bool Motor::EStopConnector(ClearCorePins pin) const
{
  return connector->EStopConnector(pin);
}
