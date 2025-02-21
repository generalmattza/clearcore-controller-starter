#ifndef __AXIS_HPP_
#define __AXIS_HPP_

#include "motor.hpp"




class Axis
{
    Motor* motor;
    float ratio;
    DigitalIn* torque_limit;
    DigitalIn* velocity_limit;

    public:
    
}
#endif // __AXIS_HPP__