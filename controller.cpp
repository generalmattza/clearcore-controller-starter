#include "controller.hpp"

double Controller::readVelocityCommand(void)
{
    // int adc_result = analogRead(this->velocity_limit_control_pin);
    int16_t adc_result = this->velocity_limit_dial_pin->State();
    double velocity_command = 1.0 * adc_result / ((1 << adcResolution) - 1);
    return velocity_command;
}

double Controller::readTorqueCommand(void)
{
    // int adc_result = analogRead(this->torque_limit_control_pin);
    int16_t adc_result = this->torque_limit_dial_pin->State();
    double torque_command = 1.0 * adc_result / ((1 << adcResolution) - 1);
    return torque_command;
}

void Controller::init(void)
{
    AdcMgr.AdcResolution(adcResolution);
    this->torque_limit_dial_pin->Mode(Connector::INPUT_ANALOG);
    this->velocity_limit_dial_pin->Mode(Connector::INPUT_ANALOG);
    this->axis_up_button_pin->Mode(Connector::INPUT_DIGITAL);
    this->axis_down_button_pin->Mode(Connector::INPUT_DIGITAL);
    this->clear_faults_button_pin->Mode(Connector::INPUT_DIGITAL);
    this->zero_axis_button_pin->Mode(Connector::INPUT_DIGITAL);
    this->e_stop_switch_pin->Mode(Connector::INPUT_DIGITAL);
}

bool Controller::readAxisUpButton(void)
{
    return this->axis_up_button_pin->State();
}

bool Controller::readAxisDownButton(void)
{
    return this->axis_down_button_pin->State();
}

bool Controller::readClearFaultsButton(void)
{
    return this->clear_faults_button_pin->State();
}

bool Controller::readZeroAxisButton(void)
{
    return this->zero_axis_button_pin->State();
}

bool Controller::readEstopSwitch(void)
{
  return this->e_stop_switch_pin->State();
}