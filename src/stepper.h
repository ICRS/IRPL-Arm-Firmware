#include <Arduino.h>

#ifndef STEPPER_H
#define STEPPER_H

class Stepper
{
private:
    uint8_t step_pin;
    uint8_t dir_pin;
    bool pos_dir;
public:
    Stepper(uint8_t step_pin,uint8_t dir_pin, uint8_t flip);
    
    void setSpeed(float target);
};

#endif