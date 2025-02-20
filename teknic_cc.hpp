#ifndef __TEKNIC_CC_HPP__
#define __TEKNIC_CC_HPP__

#include "HardwareSerial.h"
#include "MotorDriver.h"
#include "ClearCore.h"
#include <Arduino.h>
#include "motor.hpp"

// This example has built-in functionality to automatically clear motor alerts,
//	including motor shutdowns. Any uncleared alert will cancel and disallow motion.
// WARNING: enabling automatic alert handling will clear alerts immediately when
//	encountered and return a motor to a state in which motion is allowed. Before
//	enabling this functionality, be sure to understand this behavior and ensure
//	your system will not enter an unsafe state.
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)

// #define HANDLE_ALERTS false

struct MotorData
{
  int32_t position;
  int32_t velocity;
  int32_t torque;
  int32_t status;
  int32_t faults;
};

class teknic_cc
{
private:
  Motor **motors;
  uint8_t motor_count;
  const size_t packet_size = sizeof(MotorData);
  HardwareSerial *serial;
  size_t switch_filter_count = 5;
  DigitalIn *motor_enable_switch;

public:
  teknic_cc(Motor **motors, uint8_t count, HardwareSerial *serial, DigitalIn *motor_enable_switch_) : motors(motors), motor_count(count), serial(serial), motor_enable_switch(motor_enable_switch_) {}
  // teknic_cc(Motor **motors, uint8_t count, HardwareSerial *serial, DigitalIn *motor_enable_switch_, DigitalIn *homing_guard_switch_, DigitalIn *operation_mode_switch_) : motors(motors), motor_count(count), serial(serial), motor_enable_switch(motor_enable_switch_), homing_guard_switch(homing_guard_switch_), operation_mode_switch(operation_mode_switch_) {}
  void init(void);
  void enableWatchdog(void);
  void resetWatchdog(void);
  void stopMotors(void);
  void clearFaults(void);
  void handleAlerts(void);
  uint8_t getMotorCount(void)
  {
    return motor_count;
  }
  void disableMotors(void);
  void gatherMotorData(uint8_t *buffer, size_t buffer_size);
  bool motorsReady(bool negDirection = false);
  bool getOperationModeSwitch(void);
  bool motorsMoving(void);

};

#endif // __TEKNIC_CC_HPP__