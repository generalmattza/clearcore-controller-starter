#include "teknic_cc.hpp"
#include "motor.hpp"
#include "ClearCore.h"
// #include "machine_state.hpp"
#include "motor_parameters.hpp"
#include "controller.hpp"
#include "axis.hpp"

// defines the Serial port as the USB connector
// Options are Serial (USB), Serial0 (COM-0), Serial1 (COM-1)
#define SerialPort Serial

// define BAUD rate for the UART command. Using 115200
// as that's the maximum for the ClearCore
#define SerialBaudRate 115200

// Flag to enable debug output
#define DEBUG_OUTPUT false

#define axisJoystickControlPin ConnectorA12
#define clearFaultsButtonPin ConnectorDI8
#define zeroAxisButtonPin ConnectorDI7
#define estopSwitchPin ConnectorIO0

// Interval for data collection and transmission
const unsigned long serial_update_interval_ms = 250;  // ~10Hz
// // Desired loop interval (static const)
// const unsigned long interval_ms = 0;  // ~ 100Hz

struct SerialData
{
  int32_t position;
  int32_t velocity;
  int32_t motor_speed;
  int32_t torque_current;
  int32_t torque_limit;
  int32_t status;
  int32_t faults;
  int32_t controller_state;
};

const size_t serial_data_size = sizeof(SerialData);

// Define Motor Parameters
// ************************************************************************************************
// Defaults are set in the MotorParameters constructor
SDMotorParameters motor_params;

// Define Motor objects
SDMotor motor0(&ConnectorM0, "M0", &motor_params);

Motor *motors[] = { &motor0 };
uint8_t motor_count = sizeof(motors) / sizeof(motors[0]);

// Define Management Objects
// ************************************************************************************************

// Controller
Controller controller(&axisJoystickControlPin, &clearFaultsButtonPin, &zeroAxisButtonPin, &estopSwitchPin);

// Axis
// Motor instance, pulley_ratio, leadscrew_ratio, axis_velocity_limit (RPM), torque_limit_max (0-1), torque_limit_min (0-1), motor_direction_ref (true/false)
Axis axis(&motor0, 2.22, 2.54, 300, 0.9, 0.05, true);

// ClearCore - Motion controller Interface
teknic_cc clearcore(motors, motor_count, &SerialPort);

// Manager for machine state
// MachineState machine_state(&controller, &clearcore);

// Initialization
// ************************************************************************************************

/**
 * @brief Connects the Serial port
 * 
 * This function is used to attempt to connect the Serial port.
 * If the Serial port is not connected within 5 seconds, it will return false.
 * 
 * @return bool Returns true if the Serial port is connected, false otherwise.
 */
bool connectSerial(void) {

  // Attempt to reinitialize the Serial connection
  Serial.begin(SerialBaudRate);

  // Small delay to allow proper connection
  delay(1000);  

  // Wait for Serial to reconnect
  unsigned long start_time = millis();
  while (!Serial && (millis() - start_time < 3000)) {
    continue;  // Wait up to 5 seconds
  }

  if (Serial) {
    Serial.println("Serial port connected successfully.");
  } else {
    // Serial.println("Serial port disconnected. Failed to reconnect Serial. Retrying...");
  }

  return (bool)Serial;
}

SerialData buildSerialData(void) {
  SerialData data;
  data.position = axis.readCurrentPosition();
  data.velocity = axis.getVelocityCurrent();
  data.motor_speed = axis.getMotorVelocity();
  data.torque_current = axis.getMotorTorque();
  data.torque_limit = axis.getTorqueLimit();
  data.status = motor0.getStatus();
  data.faults = axis.getAlerts();
  data.controller_state = controller.getControllerState();

  return data;
}

void setup()
{

  // Initialize the Serial port
  connectSerial();

  // Initialize the axis
  axis.init();

  // Initialize the motion controller
  clearcore.init();

  // Initialize the controller
  controller.init();

  pinMode(IO3, OUTPUT);
  digitalWrite(IO3, false);

  // Setup limit switches
  // motor0.setLimitSwitchNegative(DI6);
  // motor0.setLimitSwitchPositive(DI7);
  // motor1.setLimitSwitchNegative(IO2);
  // motor1.setLimitSwitchPositive(IO3);
  // dirSelectSwitch.Mode(Connector::INPUT_DIGITAL);

  // Z-axis limit switches are not currently used
  // motor2.setLimitSwitchPositive(IO4);
  // motor2.setLimitSwitchPositive(IO5);
  // motor3.setLimitSwitchPositive(IO4);
  // motor3.setLimitSwitchPositive(IO5);

  // Assign E-Stop connectors to motors
  // motor0.EStopConnector(IO7);
  // motor1.EStopConnector(IO7);
  // motor2.EStopConnector(IO7);
  // motor3.EStopConnector(IO7);
}

// Main Loop
// ************************************************************************************************
/** 
 * @brief Manages the loop frequency by applying a delay if required
 * 
 * This function will manage the loop frequency by applying a delay if required.
 * It will calculate the elapsed time since the start of the loop and compare it to the desired interval.
 * If the elapsed time is less than the desired interval, it will apply a delay to ensure the loop runs at the desired frequency.
 * 
 * @param start_time The start time of the loop in milliseconds
 * 
 */
// void manageLoopFrequency(unsigned long start_time) {

//   unsigned long elapsed_time = millis() - start_time;
//   if (elapsed_time < interval_ms) {
//     delay(interval_ms - elapsed_time);  // Delay the remaining time
//   }
// }

void loop() {
  // static unsigned long last_update_time = 0;  // Record the start time of the loop
  // Update the machine state
  // Also fetches the latest controller data
  // machine_state.updateMachineState();
  // Watchdog on receipt of last data from controller
  // IF no data has been received within a specified time window
  // THEN perform a hard stop and disable the motors (generates a motor fault)
  // ELSE continue with program
  if (false) {
    clearcore.disableMotors();
    SerialPort.println("Controller communication timeout. Disabling motors.");
  } else {
    /** GUARDED ROUTINE
    ****************************************
    *
    * LOGIC
    * Read the controller inputs
    * IF the joystick is moved up
    * THEN move the axis up
    * IF the joystick is moved down
    * THEN move the axis down
    * IF the zero axis button is pressed
    * THEN zero the axis
    **/

    double joystick_voltage; // volts
    joystick_voltage = controller.readJoystickVoltage();
    
    double velocity_command; // steps/sec
    double normalized_signal = (joystick_voltage - 2.56) / 2.56; // Normalize to range [-1, 1]

    // quadratic scaling so that the velocity is proportional to the square of the joystick signal
    // in other words, the more extreme the joystick position is, the faster the velocity; the closer to the center, the slower the velocity
    velocity_command = normalized_signal * normalized_signal;

    if (joystick_voltage > 2.7 && joystick_voltage < 5.05) {
      axis.MoveAtVelocity(velocity_command);
    } else if (joystick_voltage < 2.3 && joystick_voltage >= 0.05) {
      axis.MoveAtVelocity(-velocity_command);
    } else {
      axis.MoveAtVelocity(0);
    }
    
    // double motor_velocity = axis.getMotorVelocity();
    // Serial.print("Motor Velocity: ");
    // Serial.println(motor_velocity);
  
   

    // END OF GUARDED ROUTINE
  }
  /** NON-GUARDED ROUTINE
    ****************************************
    * LOGIC
    **/
    // IF the clear faults button is pressed
    // THEN clear all faults
    // if (controller.readClearFaultsButton()) {
    //   SerialPort.println("Clearing faults ...");
    //   clearcore.clearFaults();
    // }
    // if (controller.readZeroAxisButton()) {
    //   SerialPort.println("Zeroing Axis ...");
    //   axis.zeroPosition();
    // }



  // START OF NON-GUARDED ROUTINE

  // Manage the loop frequency by applying a delay if required
  // manageLoopFrequency(start_time);
  // Pet watchdog to prevent system reset.
  // clearcore.resetWatchdog();

  // END OF NON-GUARDED ROUTINE
}

// /**
//  * @brief Returns the largest value in magntiude from two given values
//  *
//  * @param value1 First value to compare
//  * @param value2 Second value to compare
//  * @return The largest signed value in magnitude
//  */
// float getLargestMagnitude(float value1, float value2)
// {
//   // Compare absolute values and return the one with the largest magnitude
//   return (abs(value1) > abs(value2)) ? value1 : value2;
// }

// void publishSerialDataPeriodically(void)
// {
//   static unsigned long last_transmit_time = 0;

//   // Check if it's time to transmit data
//   if (millis() - last_transmit_time >= serial_update_interval_ms)
//   {
//     const uint8_t start_byte = 0xA5;
//     const uint8_t end_byte = 0x5A;

//     // Packet Structure = [Start byte][SerialData][End byte][CR][LF]
//     const size_t buffer_size = 1 + serial_data_size + 1 + 2;
//     uint8_t buffer[buffer_size]; // Statically allocated buffer

//     SerialData serial_data = buildSerialData();

//     // Build the complete buffer
//     size_t buffer_index = 0;
//     buffer[buffer_index++] = start_byte; // Start byte

//     // Copy serial data into the complete buffer
//     memcpy(&buffer[buffer_index], &serial_data, serial_data_size);
//     buffer_index += serial_data_size;

//     buffer[buffer_index++] = end_byte; // End byte
//     buffer[buffer_index++] = '\r';     // Carriage return
//     buffer[buffer_index++] = '\n';     // Line feed (newline)

//     // Transmit the complete buffer
//     SerialPort.write(buffer, buffer_index);

//     // Update the last transmit time
//     last_transmit_time = millis();

//     // Optionally wait for data to be written before proceeding
//     // SerialPort.flush();
//   }
//}