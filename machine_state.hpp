#ifndef __MACHINE_STATE_HPP__
#define __MACHINE_STATE_HPP__

#include <Arduino.h>
#include "teknic_cc.hpp"

class MachineState
{
private:
    teknic_cc *clearcore;
    bool estop_activated;

public:
    MachineState(teknic_cc *clearcore_);
    void updateMachineState(void);
    int32_t getMachineState(void) const;
    void setMotorsDisabledOverride(bool state);
    void setEStopActivated(bool state);
    bool getMotorsDisabledOverride() const;
    bool getMotorsEnabled() const;
    bool getEStopActivated() const;
    bool motorsReady(void) const;
};

#endif // __MACHINE_STATE_HPP__
