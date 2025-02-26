#ifndef __MOTOR_PARAMETERS_HPP__
#define __MOTOR_PARAMETERS_HPP__

#include <Arduino.h>

/**
 * @brief Class to define motor parameters like limits, velocity, and acceleration.
 *
 * This class encapsulates various parameters used for controlling a motor's
 * behavior, such as velocity limits, acceleration, deceleration, and pulses per
 * revolution (PPR).
 */
class SDMotorParameters {
public:
  int32_t velocity_limit;     // Velocity limit in PPS
  int32_t velocity_normal;    // Normal motion velocity in PPS
  int32_t velocity_fine;      // Fine motion velocity in PPS
  int32_t velocity_rapid;     // Rapid movement velocity in PPS
  int32_t accel_limit;        // Acceleration limit in PPS
  int32_t decel_limit;        // Deceleration limit in PPS
  int32_t estop_decel_limit;  // Emergency stop deceleration limit in PPS
  int32_t motor_ppr;          // Pulses per revolution

  /**
   * @brief Constructor to initialize motor parameters and convert velocities to PPS.
   *
   * Initializes motor parameters and converts the RPM values into PPS using motor_ppr.
   *
   * @param v_limit Overall velocity limit in RPM (default 600).
   * @param v_normal Normal velocity limit in RPM (default 300).
   * @param v_fine Fine motion velocity limit in RPM (default 10).
   * @param v_rapid Rapid movement velocity limit in RPM (default 600).
   * @param a_limit Normal acceleration limit in PPS (default 100000).
   * @param d_limit Normal deceleration limit in PPS (default 10000).
   * @param e_stop_d_limit Emergency deceleration limit in PPS (default 1000000).
   * @param m_ppr Pulses per revolution (default 6400).
   */
  SDMotorParameters(int32_t v_limit = 1800, int32_t v_normal = 600,
                    int32_t v_fine = 20, int32_t v_rapid = 1700,
                    int32_t a_limit = 400000, int32_t d_limit = 400000,
                    int32_t e_stop_d_limit = 1000000, int32_t m_ppr = 6400)
    : motor_ppr(m_ppr) {
    // Convert RPM to PPS for velocity limits
    velocity_limit = scaleRPMToPPS(v_limit);
    velocity_normal = scaleRPMToPPS(v_normal);
    velocity_fine = scaleRPMToPPS(v_fine);
    velocity_rapid = scaleRPMToPPS(v_rapid);

    // Set acceleration and deceleration limits
    accel_limit = a_limit;
    decel_limit = d_limit;
    estop_decel_limit = e_stop_d_limit;
  }

  /**
   * @brief Converts velocity from RPM to PPS.
   *
   * @param rpm Velocity in RPM to be converted.
   * @return The converted velocity in PPS.
   */
  int32_t scaleRPMToPPS(int32_t rpm) {
    // Conversion: RPM to PPS = RPM * motor_ppr / 60
    return rpm * motor_ppr / 60;
  }
};

// MotorParameters can be subclassed to create custom default values at initiliazation
class MotorParametersX : public SDMotorParameters {
public:
  MotorParametersX()
    : SDMotorParameters(
      800,      // v_limit: Max velocity limit (RPM)
      100,      // v_normal: Normal operating velocity (RPM)
      10,       // v_fine: Fine motion velocity (RPM)
      600,      // v_rapid: Rapid movement velocity (RPM)
      100000,   // a_limit: Acceleration limit (PPS)
      100000,   // d_limit: Deceleration limit (PPS)
      1000000,  // e_stop_d_limit: Emergency stop deceleration limit (PPS)
      6400      // m_ppr: Pulses per revolution
    ) {
    // Custom properties for X-axis can be added here if needed
  }
};




class MCMotorParameters {
public:
  int32_t velocity_limit;          // Velocity limit in RPM
  double pwm_deadband;            // PWM deadband in percentage
  double torque_limit;            // Torque limit as a factor 0-1

  MCMotorParameters(int32_t v_limit = 1500, double pwm_db = 0.03, double t_limit = 1.0)
    : velocity_limit(v_limit), pwm_deadband(pwm_db), torque_limit(t_limit) {
  }
};
#endif  // __MOTOR_PARAMETERS_HPP__