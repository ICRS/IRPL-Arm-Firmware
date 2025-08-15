#include <Arduino.h>

#ifndef STEPPER_H
#define STEPPER_H

class Stepper
{
private:
    uint8_t step_pin;
    uint8_t dir_pin;
public:
    Stepper(uint8_t step_pin,uint8_t dir_pin);
    
    void setSpeed(float target);
};

#endif