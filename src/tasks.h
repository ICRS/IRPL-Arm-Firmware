#ifndef TASKS_H
#define TASKS_H

// === INCLUDE === //

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include "config.h"
#include "esp_task_wdt.h"

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

// Shared Types
enum MessageType {
    PING,
    DES_VAL,
    CUR_ANG,
    CUR_POS,
    ERROR,
};

enum SerialErrorCode {
    NOT_SERIAL_MSG,
    NO_KEY_VAL_PAIR,
    UNKNOWN_EXECUTION,
    UNKNOWN_KEY,
    UNKNOWN_MOTOR,
};

struct Message {
    MessageType type;
    union {
        int pingValue;     // PING
        int motorID;        // ALL EXCEPT PING, ERROR
        int errorCode;      // ERROR
    };
    union {
        float motorValue;     // DES_VAL, CUR_ANG
        int positionValue;  // CUR_POS
    };
};

// Functions
int motorCommand(int ID, int newValue);
float motorStatus(int ID, bool isAngle);

// Interrupts
void IRAM_ATTR sampleEncodersISR();

#endif