#include "F710Controller.hpp"


uint8_t* F710Controller::readPacketFromSerial(size_t window_size, size_t* packet_length, uint8_t start_byte, uint8_t end_byte, unsigned long timeout_ms) {

  static uint8_t buffer[256];  // Adjust size if needed
  if (window_size > sizeof(buffer)) {
    window_size = sizeof(buffer);  // Prevent overflow
  }
  unsigned long start_time = millis();
  size_t buffer_size = 0;

  while (millis() - start_time <= timeout_ms) {
    // Read bytes from the serial port
    if (serial->available()) {
      uint8_t byte = serial->read();

      // Add the byte to the buffer
      if (buffer_size < window_size) {
        buffer[buffer_size++] = byte;
      } else {
        // Slide the window by removing the oldest byte and adding the new one
        memmove(buffer, buffer + 1, window_size - 1);
        buffer[window_size - 1] = byte;
      }

      // Check for valid packet
      if (buffer_size >= window_size && buffer[0] == start_byte && buffer[window_size - 1] == end_byte) {
        *packet_length = window_size - 2;  // Exclude start and end bytes
        return &buffer[1];                 // Return pointer to the payload
      }
    }
  }

  // Timeout reached
  *packet_length = 0;
  return nullptr;
}

uint8_t F710Controller::asciiToInt(char ascii) {
  if (ascii >= '0' && ascii <= '9')
    return (ascii - 0x30);
  else if (ascii >= 'A' && ascii <= 'F')
    return (ascii + 9 - 0x40);
  else if (ascii >= 'a' && ascii <= 'f')
    return (ascii + 9 - 0x60);
  else
    return 0;
}

void F710Controller::begin() {
  // Loop here if serial port is not initialized before the controller.
  while (!serial)
    continue;
}

bool F710Controller::parseData() {

  const size_t window_size = 18;
  static unsigned long last_received_time = 0;  // Last time data was received

  if (serial->available()) {

    size_t packet_length;
    uint8_t* buffer = readPacketFromSerial(window_size, &packet_length);

    if (buffer != nullptr) {
      // Flag to note that the controller has sent valid data before.
      // From now on if this changes, the timeout condition will be applied if data is not recevied
      last_received_time = millis();

      for (uint8_t i = 0; i < packet_length; i++) {
        buffer[i] = asciiToInt(buffer[i]);
      }

      // Parse raw bytes to update controller state
      LEFT.AXIS_X = buffer[0] << 4 | buffer[1];
      LEFT.AXIS_Y = buffer[2] << 4 | buffer[3];
      BUTTON.LT = buffer[4] << 4 | buffer[5];
      RIGHT.AXIS_X = buffer[6] << 4 | buffer[7];
      RIGHT.AXIS_Y = buffer[8] << 4 | buffer[9];
      BUTTON.RT = buffer[10] << 4 | buffer[11];

      switch (buffer[12] & DPAD_X_MASK) {
        case (DPAD_RIGHT_MASK):
          {
            DPAD.RIGHT = true;
            DPAD.LEFT = false;
            break;
          }
        case (DPAD_LEFT_MASK):
          {
            DPAD.RIGHT = false;
            DPAD.LEFT = true;
            break;
          }
        default:
          {
            DPAD.RIGHT = false;
            DPAD.LEFT = false;
            break;
          }
      }

      switch (buffer[12] & DPAD_Y_MASK) {
        case (DPAD_UP_MASK):
          {
            DPAD.UP = true;
            DPAD.DOWN = false;
            break;
          }
        case (DPAD_DOWN_MASK):
          {
            DPAD.UP = false;
            DPAD.DOWN = true;
            break;
          }
        default:
          {
            DPAD.UP = false;
            DPAD.DOWN = false;
            break;
          }
      }

      (buffer[13] & BUTTON_MODE_MASK) ? BUTTON.MODE = true : BUTTON.MODE = false;
      (buffer[13] & BUTTON_LJS_MASK) ? LEFT.PRESS = true : LEFT.PRESS = false;
      (buffer[13] & BUTTON_RJS_MASK) ? RIGHT.PRESS = true : RIGHT.PRESS = false;

      (buffer[14] & BUTTON_LB_MASK) ? BUTTON.LB = true : BUTTON.LB = false;
      (buffer[14] & BUTTON_RB_MASK) ? BUTTON.RB = true : BUTTON.RB = false;
      (buffer[14] & BUTTON_BACK_MASK) ? BUTTON.BACK = true : BUTTON.BACK = false;
      (buffer[14] & BUTTON_START_MASK) ? BUTTON.START = true : BUTTON.START = false;

      (buffer[15] & BUTTON_A_MASK) ? BUTTON.A = true : BUTTON.A = false;
      (buffer[15] & BUTTON_B_MASK) ? BUTTON.B = true : BUTTON.B = false;
      (buffer[15] & BUTTON_X_MASK) ? BUTTON.X = true : BUTTON.X = false;
      (buffer[15] & BUTTON_Y_MASK) ? BUTTON.Y = true : BUTTON.Y = false;
    }
  }
  if (millis() - last_received_time > timeout_millis) {

    // Check for timeout condition.
    // If no data received within the timeout period,
    // THEN return false to inform user that something has gone wrong
    // This prevents runaway of the device being controlled
    return false;
  } else {
    // ELSE Return true to let user know that data was received and parsed properly
    return true;
  }
}

float F710Controller::remapExponential(float value, float base = 2.0) const {
  // Clamp axisValue within the range -1 to +1
  value = clampValue(value, -1.0, 1.0);

  // Apply exponential remapping, preserving the sign
  if (value >= 0) {
    return (pow(base, value) - 1) / (base - 1);
  } else {
    return -(pow(base, -value) - 1) / (base - 1);
  }
}


uint8_t F710Controller::getLeftJoystickAxisY(void) const {
  if (invert_left_joystick) {
    return 255 - LEFT.AXIS_Y;  // Invert the value
  }
  return LEFT.AXIS_Y;  // Return as-is if not inverted
}

uint8_t F710Controller::getRightJoystickAxisY(void) const {
  if (invert_right_joystick) {
    return 255 - RIGHT.AXIS_Y;  // Invert the value
  }
  return RIGHT.AXIS_Y;  // Return as-is if not inverted
}

float F710Controller::getRightJoystickAxisXFloat(void) const {
  return conditionAxisValue(getRightJoystickAxisX());
}

float F710Controller::getRightJoystickAxisYFloat(void) const {
  return conditionAxisValue(getRightJoystickAxisY());
}

float F710Controller::getLeftJoystickAxisXFloat(void) const {
  return conditionAxisValue(getLeftJoystickAxisX());
}

float F710Controller::getLeftJoystickAxisYFloat(void) const {
  return conditionAxisValue(getLeftJoystickAxisY());
}

uint8_t F710Controller::clampValue(uint8_t value, uint8_t min = 0, uint8_t max = 255) const {
  // Clamp the value to the specified range
  return (value < min) ? min : (value > max) ? max
                                             : value;
}

float F710Controller::clampValue(float value, float min = -1.0, float max = 1.0) const {
  // Clamp the value to the specified range
  return (value < min) ? min : (value > max) ? max
                                             : value;
}

float F710Controller::conditionAxisValue(uint8_t value) const {

  // Clamp the input value, apply offset, and scale
  float adjusted_value = (clampValue(value) + joystick_offset) * joystick_scale;

  // Apply deadband and remap
  if (adjusted_value > -joystick_deadband && adjusted_value < joystick_deadband) {
    return 0.0;  // Values within deadband are treated as zero
  } else {
    // Remap values outside the deadband to the range [-1.0, 1.0]
    float sign = (adjusted_value > 0) ? 1.0f : -1.0f;  // Determine sign of the value
    adjusted_value = (fabs(adjusted_value) - joystick_deadband) / (joystick_saturation - joystick_deadband);
    adjusted_value = clampValue(sign * adjusted_value, -1.0, 1.0);
    if (joystick_response_exponent > 1) {
      return remapExponential(adjusted_value, joystick_response_exponent);
    }
    return adjusted_value;
  }
}

void F710Controller::getControllerState(byte* buffer, size_t buffer_size) {

    if (buffer_size > controller_state_size) {
      // Handle insufficient buffer case (e.g., return or throw an error)
      return;
    }

    size_t offset = 0;

    // Pack JOYSTICK structs (LEFT + RIGHT)
    memcpy(buffer + offset, &LEFT, sizeof(LEFT));
    offset += sizeof(LEFT);
    memcpy(buffer + offset, &RIGHT, sizeof(RIGHT));
    offset += sizeof(RIGHT);

    // Pack DPAD struct
    memcpy(buffer + offset, &DPAD, sizeof(DPAD));
    offset += sizeof(DPAD);

    // Pack BUTTON struct
    memcpy(buffer + offset, &BUTTON, sizeof(BUTTON));
    offset += sizeof(BUTTON);
}
