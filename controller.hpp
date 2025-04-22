#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__

#include "ClearCore.h"

#define adcResolution 12

class Controller
{
    DigitalInAnalogIn *joystick_pin;
    DigitalIn *clear_faults_button_pin;
    DigitalIn *zero_axis_button_pin;
    DigitalIn *e_stop_switch_pin;

public:
    Controller(
        DigitalInAnalogIn *joystick_pin,
        DigitalIn *clear_faults_button_pin,
        DigitalIn *zero_axis_button_pin,
        DigitalIn *e_stop_switch_pin
    )
    {
        this->joystick_pin = joystick_pin;
        this->clear_faults_button_pin = clear_faults_button_pin;
        this->zero_axis_button_pin = zero_axis_button_pin;
        this->e_stop_switch_pin = e_stop_switch_pin;
    }
    void init(void);
    double readJoystickVoltage(void);
    bool readClearFaultsButton(void);
    bool readZeroAxisButton(void);
    bool readEstopSwitch(void);
    int32_t getControllerState(void);

};

#endif // __CONTROLLER_HPP__