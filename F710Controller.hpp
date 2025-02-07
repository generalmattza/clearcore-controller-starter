#include "HardwareSerial.h"
#ifndef __F710_CONTROLLER_HPP__
#define __F710_CONTROLLER_HPP__

#include <Arduino.h>
#include <stdlib.h>

#define BUTTON_A_MASK 0x1
#define BUTTON_B_MASK 0x2
#define BUTTON_X_MASK 0x4
#define BUTTON_Y_MASK 0x8

#define BUTTON_LB_MASK 0x1
#define BUTTON_RB_MASK 0x2
#define BUTTON_BACK_MASK 0x4
#define BUTTON_START_MASK 0x8

#define BUTTON_MODE_MASK 0x1
#define BUTTON_LJS_MASK 0x2
#define BUTTON_RJS_MASK 0x4

#define DPAD_X_MASK 0x3
#define DPAD_Y_MASK 0xC

#define DPAD_RIGHT_MASK 0x1
#define DPAD_LEFT_MASK 0x2
#define DPAD_UP_MASK 0x4
#define DPAD_DOWN_MASK 0x8

class F710Controller
{
private:

  HardwareSerial *serial;

  uint8_t buffer[16];

  struct JOYSTICK
  {
    uint8_t AXIS_X = false;
    uint8_t AXIS_Y = false;
    bool PRESS = false;
  };

  struct DPAD
  {
    bool UP = false;
    bool DOWN = false;
    bool LEFT = false;
    bool RIGHT = false;
  };

  struct BUTTON
  {
    bool A = false;
    bool B = false;
    bool X = false;
    bool Y = false;
    bool LB = false;
    uint8_t LT = 0;
    bool RB = false;
    uint8_t RT = 0;
    bool START = false;
    bool MODE = false;
    bool BACK = false;
  };

  JOYSTICK LEFT;
  JOYSTICK RIGHT;
  BUTTON BUTTON;
  DPAD DPAD;

  const size_t controller_state_size = sizeof(LEFT) + sizeof(RIGHT) + sizeof(BUTTON) + sizeof(DPAD);
  
  const bool invert_left_joystick = true;
  const bool invert_right_joystick = true;
  const float joystick_scale = 1.0/127;
  const int joystick_offset = -127;
  const float joystick_deadband = 0.04;
  const float joystick_saturation = 0.85;
  const float joystick_response_exponent = 4.0f;

  const unsigned long timeout_millis; // Timeout in milliseconds

public:

  F710Controller(HardwareSerial *serialPort, unsigned long timeout = 1000)
      : serial(serialPort), timeout_millis(timeout) {}

  void begin();
  bool parseData();
  void getControllerState(byte* buffer, size_t buffer_size);
  size_t getControllerStateSize(void) {return controller_state_size;}
  bool getButtonA(void) const { return BUTTON.A; }
  bool getButtonB(void) const { return BUTTON.B; }
  bool getButtonX(void) const { return BUTTON.X; }
  bool getButtonY(void) const { return BUTTON.Y; }
  bool getButtonLB(void) const { return BUTTON.LB; }
  bool getButtonRB(void) const { return BUTTON.RB; }
  uint8_t getButtonLT(void) const { return BUTTON.LT; }
  uint8_t getButtonRT(void) const { return BUTTON.RT; }
  bool getButtonStart(void) const { return BUTTON.START; }
  bool getButtonMode(void) const { return BUTTON.MODE; }
  bool getButtonBack(void) const { return BUTTON.BACK; }
  bool getLeftJoystickPress(void) const { return LEFT.PRESS; }
  bool getRightJoystickPress(void) const { return RIGHT.PRESS; }

  uint8_t getLeftJoystickAxisX(void) const { return LEFT.AXIS_X; }
  uint8_t getLeftJoystickAxisY(void) const;
  float getLeftJoystickAxisXFloat(void) const;
  float getLeftJoystickAxisYFloat(void) const;
  
  uint8_t getRightJoystickAxisX(void) const { return RIGHT.AXIS_X; }
  uint8_t getRightJoystickAxisY(void) const;
  float getRightJoystickAxisXFloat(void) const;
  float getRightJoystickAxisYFloat(void) const;

  bool getDPADLeft(void) const { return DPAD.LEFT; }
  bool getDPADRight(void) const { return DPAD.RIGHT; }
  bool getDPADUp(void) const { return DPAD.UP; }
  bool getDPADDown(void) const { return DPAD.DOWN; }

  uint8_t clampValue(uint8_t value, uint8_t min, uint8_t max) const;
  float clampValue(float value, float min, float max) const;
  float conditionAxisValue(uint8_t value) const;
  float remapExponential(float value, float base) const;
  float getJoystickScale(void) const {return joystick_scale;}
  int getJoystickOffset(void) const {return joystick_offset;}
  float getJoystickDeadband(void) const {return joystick_deadband;}
  float getJoystickSaturation(void) const {return joystick_saturation;}
  float getJoystickResponseExponent(void) const {return joystick_response_exponent;}
  uint8_t* readPacketFromSerial(size_t window_size, size_t* packet_length, uint8_t start_byte = 0x41, uint8_t end_byte = 0x35, unsigned long timeout_ms = 50);
  uint8_t asciiToInt(char ascii);
};

#endif // __F710_CONTROLLER_HPP__