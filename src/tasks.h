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

#endif