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

// #define dirSelectSwitch ConnectorDI8
#define motorTorqueControlPin ConnectorA12
#define motorVelocityControlPin ConnectorA11
#define axisUpButtonPin ConnectorA10
#define axisDownButtonPin ConnectorA9
#define clearFaultsButtonPin ConnectorDI8
#define zeroAxisButtonPin ConnectorDI7
#define estopSwitchPin ConnectorDI6

// Interval for data collection and transmission
const unsigned long serial_update_interval_ms = 100;  // ~20Hz
// Desired loop interval (static const)
const unsigned long interval_ms = 0;  // ~ 100Hz
// Delay before attempting to reconnect the serial port (ms)
const unsigned long reconnectInterval = 500;

// Define Motor Parameters
// ************************************************************************************************
// Defaults are set in the MotorParameters constructor
MCMotorParameters motor_params;

// Define Motor objects
MCMotor motor0(&ConnectorM0, "M0", &motor_params);

Motor *motors[] = { &motor0 };
uint8_t motor_count = sizeof(motors) / sizeof(motors[0]);

// Define Management Objects
// ************************************************************************************************

// Controller
Controller controller(&motorTorqueControlPin, &motorVelocityControlPin, &axisUpButtonPin, &axisDownButtonPin, &clearFaultsButtonPin, &zeroAxisButtonPin, &estopSwitchPin);

// Axis
// Motor instance, drive_ratio, axis_velocity_limit (RPM), torque_limit_max (0-1), torque_limit_min (0-1), motor_direction_ref (true/false)
Axis axis(&motor0, 120.0, 12.0, 0.8, 0.05);

// ClearCore - Motion controller Interface
teknic_cc clearcore(motors, motor_count, &SerialPort);

// Manager for machine state
// MachineState machine_state(&controller, &clearcore);

// Initialization
// ************************************************************************************************

bool serialReconnecting = false;
unsigned long lastReconnectAttempt = 0;

/**
 * @brief Reconnects the Serial port if it is disconnected
 * 
 * This function is used to attempt to reconnect the Serial port if it is disconnected.
 * It will close the Serial port, wait for a small delay, and then attempt to reopen the Serial port.
 * If the Serial port is not reconnected within 5 seconds, it will retry the reconnection.
 * 
 * @return bool Returns true if the Serial port is reconnected, false otherwise.
 */
bool reconnectSerial(void) {
  // Attempt to reinitialize the Serial connection
  // Serial.end();  // Ensure Serial is properly closed
  // delay(1000);   // Small delay to ensure proper reset
  Serial.begin(SerialBaudRate);

  // Wait for Serial to reconnect
  unsigned long start_time = millis();
  while (!Serial && (millis() - start_time < 2000)) {
    continue;  // Wait up to 5 seconds
  }

  if (Serial) {
    Serial.println("Serial port disconnected, but was reconnected successfully.");
  } else {
    Serial.println("Serial port disconnected. Failed to reconnect Serial. Retrying...");
  }

  return (bool)Serial;
}
/**
 * @brief Checks the Serial connection and attempts to reconnect if disconnected
 * 
 * This function is used to check the Serial connection and attempt to reconnect if it is disconnected.
 * It will check if the Serial port is connected, and if not, it will attempt to reconnect the Serial port.
 * 
 * @return bool Returns true if the Serial port is connected, false otherwise.
 * 
 * @note This function will attempt to reconnect the Serial port if it is disconnected.
 *      It will wait for a small delay before attempting to reconnect the Serial port.
 *      It will not attempt to reconnect the first time the Serial port is disconnected.
 *      This is to allow the main loop to run at least once before attempting to reconnect the Serial port.
 */
bool checkSerialConnection(void) {
  if (Serial) {
    serialReconnecting = false;  // Reset if serial is connected
    return true;
  }

  // Begin reconnection process
  if (!serialReconnecting) {
    serialReconnecting = true;
    lastReconnectAttempt = millis();
  }

  // Perform reconnection steps based on timer
  if (millis() - lastReconnectAttempt >= reconnectInterval) {
    lastReconnectAttempt = millis();  // Update timer

    // Perform a step in the reconnection process
    if (!reconnectSerial()) {
      return false;
    } else {
      serialReconnecting = false;  // Stop reconnection attempts
      return true;
    }
  }
}

void setup() {

  // Initialize the Serial port
  reconnectSerial();
  // Serial.begin(SerialBaudRate);


  // Initialize the axis
  axis.init();

  // Initialize the motion controller
  clearcore.init();

  // Initialize the controller
  controller.init();

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
void manageLoopFrequency(unsigned long start_time) {

  unsigned long elapsed_time = millis() - start_time;
  if (elapsed_time < interval_ms) {
    delay(interval_ms - elapsed_time);  // Delay the remaining time
  }
}

void loop() {
  unsigned long start_time = millis();  // Record the start time of the loop

  // Update the machine state
  // Also fetches the latest controller data
  // machine_state.updateMachineState();
  // Watchdog on receipt of last data from controller
  // IF no data has been received within a specified time window
  // THEN perform a hard stop and diable the motors (generates a motor fault)
  // ELSE continue with program
  if (false) {
    clearcore.disableMotors();
  } else {
    /** GUARDED ROUTINE
    ****************************************
    *
    * LOGIC
    * Read the controller inputs
    * IF the axis up button is pressed
    * THEN move the axis up
    * IF the axis down button is pressed
    * THEN move the axis down
    * IF the zero axis button is pressed
    * THEN zero the axis
    **/ 

    double velocity_command = controller.readVelocityCommand();
    double torque_limit = controller.readTorqueCommand();
    bool axis_up_button_state = controller.readAxisUpButton();
    bool axis_down_button_state = controller.readAxisDownButton();


    // Limit motor torque to the value read from the dial
    axis.limitMotorTorque(torque_limit);
    int32_t motor_speed;

    if (axis_up_button_state) {
      Serial.println("Moving UP");
      motor_speed = axis.MoveAtVelocity(velocity_command);
    } else if (axis_down_button_state) {
      Serial.println("Moving DOWN");
      motor_speed = axis.MoveAtVelocity(-velocity_command);
    } else {
      Serial.println("Not Moving");
      motor_speed = axis.MoveAtVelocity(0);
    }

    // END OF GUARDED ROUTINE
  }
    /** NON-GUARDED ROUTINE
    ****************************************
    * LOGIC
    **/


    // START OF NON-GUARDED ROUTINE
    // IF the clear faults button is pressed
    // THEN clear all faults
    if (controller.readClearFaultsButton()) {
      SerialPort.println("Clearing faults ...");
      clearcore.clearFaults();
    }
    if (controller.readZeroAxisButton()) {
      axis.zeroPosition();
    }

  // Update the serial port every serial_update_interval_ms
  if ((millis() - start_time) < serial_update_interval_ms) {
    // Publish data periodically, every DataCollectorUpdateInterval ms
    // publishSerialDataPeriodically();
    Serial.print("Axis Position: ");
    Serial.print(axis.getPositionCurrent());
    Serial.print("\tAxis Velocity: ");
    Serial.print(axis.getVelocityCurrent());
    Serial.print("\tMotor Speed: ");
    Serial.print(axis.getMotorVelocity());
    Serial.print("\tMotor Torque: ");
    Serial.println(axis.getMotorTorque());
  }
  // Manage the loop frequency by applying a delay if required
  // manageLoopFrequency(start_time);
  // Pet watchdog to prevent system reset.
  clearcore.resetWatchdog();

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

// void publishSerialDataPeriodically(void) {
//   static unsigned long last_transmit_time = 0;

//   // Check if it's time to transmit data
//   if (millis() - last_transmit_time >= data_collector_iterval_ms) {
//     const uint8_t start_byte = 0xA5;
//     const uint8_t end_byte = 0x5A;
//     const size_t motordata_packet_size = sizeof(MotorData);
//     const size_t motordata_buffer_size = motor_count * motordata_packet_size;

//     // Create a dynamic buffer to hold the complete message
//     // Packet Structure = [Start byte][Motor data][Machine state][controller state][End byte][CR]
//     const size_t machine_state_size = sizeof(int32_t);
//     const size_t controller_state_size = controller.getControllerStateSize();
//     // const size_t controller_state_size = 0; // Disabled
//     const size_t complete_buffer_size = 1 + motordata_buffer_size + machine_state_size + controller_state_size + 1 + 2;
//     uint8_t complete_buffer[complete_buffer_size];  // Statically allocated buffer

//     // Gather motor data into the buffer
//     uint8_t motordata_buffer[motordata_buffer_size];
//     clearcore.gatherMotorData(motordata_buffer, motordata_buffer_size);

//     // Get the machine state as a 32-bit integer
//     int32_t machine_state_current = machine_state.getMachineState();

//     uint8_t controller_state_buffer[controller_state_size];
//     controller.getControllerState(controller_state_buffer, controller_state_size);

//     // Build the complete buffer
//     size_t buffer_index = 0;
//     complete_buffer[buffer_index++] = start_byte;  // Start byte

//     // Copy motor data into the complete buffer
//     memcpy(&complete_buffer[buffer_index], motordata_buffer, motordata_buffer_size);
//     buffer_index += motordata_buffer_size;

//     // Copy machine state into the complete buffer
//     memcpy(&complete_buffer[buffer_index], &machine_state_current, machine_state_size);
//     buffer_index += machine_state_size;

//     // Copy controller state into the complete buffer
//     memcpy(&complete_buffer[buffer_index], controller_state_buffer, controller_state_size);
//     buffer_index += controller_state_size;

//     complete_buffer[buffer_index++] = end_byte;  // End byte
//     complete_buffer[buffer_index++] = '\r';      // Carriage return
//     complete_buffer[buffer_index++] = '\n';      // Line feed (newline)

//     // Transmit the complete buffer
//     SerialPort.write(complete_buffer, buffer_index);

//     // Update the last transmit time
//     last_transmit_time = millis();

//     // Wait for data to be written before proceeding
//     // SerialPort.flush();
//   }
// }