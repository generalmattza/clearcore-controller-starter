// #include "machine_state.hpp"

// MachineState::MachineState(teknic_cc *clearcore_)
//     : clearcore(clearcore_) {}

// void MachineState::updateMachineState(void)
// {
//   // This function is used to update the machine state
//   return;

// }

// /**
//  * @brief Get the machine state as a byte.
//  *
//  * The machine state is a byte with each bit representing a different state.
//  *
//  * Bit 0: Motors Enabled
//  * Bit 1: E-Stop Activated
//  * Bit 2: Clear Faults Button Pressed
//  *
//  * @return uint8_t The machine state as a byte.
//  */
// int32_t MachineState::getMachineState(void) const
// {
//   int32_t state = 0;

//   state |= getMotorsEnabled() << 0;
//   state |= getEStopActivated() << 1;

//   return state;
// }

// bool MachineState::getEStopActivated() const
// {
//   return estop_activated;
// }

// bool MachineState::getMotorsEnabled() const
// {
//   // Check if motors are enabled by checking:

//   return (!getEStopActivated() && motorsReady());
// }


// void MachineState::setEStopActivated(bool state)
// {
//   estop_activated = state;
// }

// bool MachineState::motorsReady(void) const
// {
//   return clearcore->motorsReady();
// }
