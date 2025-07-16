#ifndef TASKS_H
#define TASKS_H

// === INCLUDE === //

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include "config.h"

// === EXTERNALS === //

// Task Handles
extern TaskHandle_t debugTaskHandle;
extern TaskHandle_t encoderTaskHandle;
extern TaskHandle_t serialTaskHandle;
extern TaskHandle_t controlTaskHandle;

// Tasks
void debugTask(void *pvParameters);
void encoderTask(void *pvParameters);
void serialTask(void *pvParameters);
void controlTask(void *pvParameters);

// Global variables
extern std::array<uint32_t, N_ENCODERS> desiredAngleArray;
extern std::array<uint32_t, N_ENCODERS> currentAngleArray;

// Shared Types
enum MessageType {
    PING,
    DES_ANG,
    DES_POS,
    CUR_ANG,
    CUR_POS,
    ERROR,
};

enum SerialErrorCode {
    NOT_SERIAL_MSG,
    NO_KEY_VAL_PAIR,
    UNKNOWN_EXECUTION,
    UNKNOWN_KEY,
};

struct Message {
    MessageType type;
    union {
        int pingValue;     // PING
        int motorID;        // ALL EXCEPT PING, ERROR
        int errorCode;      // ERROR
    };
    union {
        int angleValue;     // DES_ANG, CUR_ANG
        int positionValue;  // DES_POS, CUR_POS
    };
};

// Functions
void motorCommand(int ID, int newValue, bool isAngle);

// Interrupts
void IRAM_ATTR sampleEncodersISR();

#endif