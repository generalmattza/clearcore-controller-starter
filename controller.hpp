#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__

#include "ClearCore.h"

#define adcResolution 12

class Controller
{

    DigitalInAnalogIn *torque_limit_dial_pin;
    DigitalInAnalogIn *velocity_limit_dial_pin;
    DigitalInAnalogIn *axis_up_button_pin;
    DigitalInAnalogIn *axis_down_button_pin;
    DigitalIn *clear_faults_button_pin;
    DigitalIn *zero_axis_button_pin;

public:
    Controller(
        DigitalInAnalogIn *torque_limit_dial_pin,
        DigitalInAnalogIn *velocity_limit_dial_pin,
        DigitalInAnalogIn *axis_up_button_pin,
        DigitalInAnalogIn *axis_down_button_pin,
        DigitalIn *clear_faults_button_pin,
        DigitalIn *zero_axis_button_pin
    )
    {
        this->torque_limit_dial_pin = torque_limit_dial_pin;
        this->velocity_limit_dial_pin = velocity_limit_dial_pin;
        this->axis_up_button_pin = axis_up_button_pin;
        this->axis_down_button_pin = axis_down_button_pin;
        this->clear_faults_button_pin = clear_faults_button_pin;
        this->zero_axis_button_pin = zero_axis_button_pin;
    }
    void init(void);
    double readTorqueCommand(void);
    double readVelocityCommand(void);
    bool readAxisUpButton(void);
    bool readAxisDownButton(void);
    bool readClearFaultsButton(void);
    bool readZeroAxisButton(void);
};

#endif // __CONTROLLER_HPP__