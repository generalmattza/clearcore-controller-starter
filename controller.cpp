#include "controller.hpp"

// double Controller::readVelocityCommand(void)
// {
//     int16_t adc_result = this->velocity_limit_dial_pin->State();
//     double velocity_command = 1.0 * adc_result / ((1 << adcResolution) - 1);
//     return velocity_command;
// }

double Controller::readJoystickVoltage(void)
{
    int16_t adc_result = this->joystick_pin->State();
    double joystick_voltage = 10.0 * adc_result / ((1 << adcResolution) - 1);
    return joystick_voltage;
}

void Controller::init(void)
{
    AdcMgr.AdcResolution(adcResolution);
    this->joystick_pin->Mode(Connector::INPUT_ANALOG);
    this->clear_faults_button_pin->Mode(Connector::INPUT_DIGITAL);
    this->zero_axis_button_pin->Mode(Connector::INPUT_DIGITAL);
    this->e_stop_switch_pin->Mode(Connector::INPUT_DIGITAL);
}

bool Controller::readClearFaultsButton(void)
{
    // return this->clear_faults_button_pin->State();
    return this->clear_faults_button_pin->InputFallen();
}

bool Controller::readZeroAxisButton(void)
{
    // return this->zero_axis_button_pin->State();
    return this->zero_axis_button_pin->InputFallen();
}

bool Controller::readEstopSwitch(void)
{
  return !this->e_stop_switch_pin->State();
}

int32_t Controller::getControllerState(void)
{
    int32_t state = 0;

    // Set binary flags (using bits 0-4)
    if (this->readEstopSwitch())
        state |= (1 << 0); // Bit 0 for e-stop (active low, remember inversion in readEstopSwitch)

    // if (this->readAxisUpButton())
    //     state |= (1 << 1); // Bit 1 for axis up button

    // if (this->readAxisDownButton())
    //     state |= (1 << 2); // Bit 2 for axis down button

    if (this->readClearFaultsButton())
        state |= (1 << 3); // Bit 3 for clear faults button

    if (this->readZeroAxisButton())
        state |= (1 << 4); // Bit 4 for zero axis button

    // Convert the dial values from their ADC range to 8-bit values (0-255)
    // uint8_t velocity_value = static_cast<uint8_t>(
    //     this->readVelocityCommand() * 255.0);
    uint8_t joystick_value = static_cast<uint8_t>(
        this->readJoystickVoltage() * 255.0);

    // Pack the dial values into the state:
    // Bits 8-15 for velocity dial value
    // state |= (velocity_value << 8);

    // Bits 16-23 for the joystick value
    state |= (joystick_value << 16);

    return state;
}
