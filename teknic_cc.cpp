#include "teknic_cc.hpp"


void teknic_cc::init(void)
{

  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

  // MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_A_PWM_B_PWM);


  for (uint8_t i = 0; i < motor_count; i++)
  {
    motors[i]->initMotor();
  }

  enableWatchdog();

  clearFaults();


}

void teknic_cc::enableWatchdog()
{
  // Enable the GCLK for WDT using the 1kHz oscillator
  GCLK->PCHCTRL[ID_WDT].reg = GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK1_Val) | GCLK_PCHCTRL_CHEN;
  while (!(GCLK->PCHCTRL[ID_WDT].reg & GCLK_PCHCTRL_CHEN))
  {
    // Wait for synchronization
  }

  // Disable the WDT before configuration
  WDT->CTRLA.reg &= ~WDT_CTRLA_ENABLE;
  while (WDT->SYNCBUSY.bit.ENABLE)
  {
    // Wait for synchronization
  }

  // Set the time-out period and window mode
  WDT->CONFIG.reg = 0xA; // Time-out period of ~8 seconds
  WDT->EWCTRL.reg = 0xA; // Early warning offset

  // Enable the watchdog with reset enabled
  WDT->CTRLA.reg = WDT_CTRLA_ENABLE | WDT_CTRLA_WEN; // Watchdog enabled with window mode
  while (WDT->SYNCBUSY.bit.ENABLE)
  {
    // Wait for synchronization
  }
}

// reset watchdog = kick the dog = pet the dog
// this function needs to be called every 8 seconds. otherwise hardware reset
void teknic_cc::resetWatchdog()
{
  // Clear the watchdog timer to prevent reset
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while (WDT->SYNCBUSY.bit.CLEAR)
  {
    // Wait for synchronization
  }
}

void teknic_cc::handleAlerts(void) {
  for (uint8_t i = 0; i < motor_count; i++)
  {
    motors[i]->handleAlerts();
  }
}

void teknic_cc::clearFaults(void)
{
  for (uint8_t i = 0; i < motor_count; i++)
  {
    motors[i]->clearFaults();
  }
}

void teknic_cc::stopMotors(void)
{
  for (uint8_t i = 0; i < motor_count; i++)
  {
    motors[i]->stopMotor();
  }
}

bool teknic_cc::motorsReady(bool negDirection)
{
  // Check if motor all motors are ready
  bool motor_ready[motor_count];
  for (uint8_t i = 0; i < motor_count; i++)
  {
    motor_ready[i] = motors[i]->validateMove(negDirection);
  }
  // Check if all motors are ready
  for (uint8_t i = 0; i < motor_count; i++)
  {
    if (!motor_ready[i])
    {
      return false; // Return false immediately if any motor is not ready
    }
  }

  return true; // All motors are ready
}

void teknic_cc::disableMotors(void)
{
  for (uint8_t i = 0; i < motor_count; i++)
  {
    motors[i]->disableMotor();
  }
}

void teknic_cc::gatherMotorData(uint8_t *buffer, size_t buffer_size)
{

  size_t required_size = motor_count * packet_size;

  if (buffer_size < required_size)
  {
    // Handle insufficient buffer case (e.g., return or throw an error)
    return;
  }

  size_t offset = 0;

  for (uint8_t i = 0; i < motor_count; i++)
  {
    if (motors[i] != nullptr)
    {
      MotorData packet = {
          motors[i]->getPositionCurrent(),
          motors[i]->getVelocityCurrent(),
          (int32_t)motors[i]->getTorqueCurrent(),
          motors[i]->getStatus(),
          motors[i]->getAlerts()};

      memcpy(&buffer[offset], &packet, packet_size);
      offset += packet_size;
    }
  }
}

bool teknic_cc::motorsMoving(void) {
  for (uint8_t i = 0; i < motor_count; i++)
  {
    if (motors[i] != nullptr)
    {
      if (motors[i]->getVelocityCurrent() > 10) {
        return true;
      }
    }
  }
  return false;
}
