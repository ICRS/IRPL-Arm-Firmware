#include "stepper.h"
#define MAX_FREQ 5000

// Static LEDC channel allocator
static uint8_t nextChannel = 0;

#define LEDC_TIMER_BITS  8            // PWM resolution
#define LEDC_BASE_FREQ   1000          // Initial/default frequency
#define offset 1

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Stepper::Stepper(uint8_t step_pin, uint8_t dir_pin, uint8_t flip=0)
: step_pin(step_pin), dir_pin(dir_pin)
{
    uint8_t channel = nextChannel++;
    
    pinMode(dir_pin, OUTPUT);

    ledcSetup(channel, LEDC_BASE_FREQ, LEDC_TIMER_BITS);
    ledcAttachPin(step_pin, channel);
    this->pos_dir = flip?LOW:HIGH;
    this->step_pin = channel;
}

void Stepper::setSpeed(float target)
{   
    if (target > 0) {
        digitalWrite(dir_pin, pos_dir);
    }
    else if (target < 0) {
        digitalWrite(dir_pin, !pos_dir);
        target = -target;   
    }
    else {
        ledcWriteTone(step_pin, 0);
        return;
    }
    if (target > offset || target <-offset){
        float scaled_freq = mapf(target,0,180.0,0,MAX_FREQ);
        ledcWriteTone(step_pin, scaled_freq);
    }else {
        ledcWriteTone(step_pin, 0);
        return;
    }
}


