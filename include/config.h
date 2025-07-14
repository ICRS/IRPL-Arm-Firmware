#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Set all pin numbers for various motors.
#define SHOULDER_STEP_PIN 24
#define SHOULDER_DIR_PIN  23
#define SHOULDER_EN_PIN   22

#define ELBOW_STEP_PIN    27
#define ELBOW_DIR_PIN     26
#define ELBOW_EN_PIN      25

#define WRIST_STEP_PIN    30
#define WRIST_DIR_PIN     29
#define WRIST_EN_PIN      28

#define BASE_STEP_PIN     31
#define BASE_DIR_PIN      32
#define BASE_EN_PIN       40

#define ROLL_EN_PIN       9
#define IN_1_PIN          4
#define IN_2_PIN          5

// Pi for degree-radian conversions
#define PI 3.141

// Arm initialisers

// Period of square wave to steppers.
#define DEFAULT_PERIOD 500;

#define DEFAULT_STEPS 400;

#define GEAR_RATIO_SHOULDER 100.0;
#define GEAR_RATIO_ELBOW 50.0;
#define GEAR_RATIO_WRIST 91.8;
#define GEAR_RATIO_BASE 102.0;
#define UPPER_ARM_LENGTH 325;
#define FORE_ARM_LENGTH 330;
#define GRIPPER_LENGTH 195;

#define FOUR_BAR_L1 330;
#define FOUR_BAR_L2 110;
#define FOUR_BAR_L3 142.147;
#define FOUR_BAR_L4 330;
#define FOUR_BAR_L5 325;
#define FOUR_BAR_L6 142.147;
#define FOUR_BAR_L7 200;
#define FOUR_BAR_L8 320;

#define FOUR_BAR_BETA_DEG -101.42;

// === TASKS === //

// DEBUG Task
#define DEBUG_TASK_ENABLE true
#define DEBUG_TASK_FREQUENCY 10
#define DEBUG_TASK_STACK_SIZE 2500
#define DEBUG_TASK_PRIORITY 1

// ENCODER Task
#define ENCODER_TASK_ENABLE false
#define ENCODER_TASK_FREQUENCY 100
#define ENCODER_TASK_STACK_SIZE 5000
#define ENCODER_TASK_PRIORITY 4

// SERIAL Task
#define SERIAL_TASK_ENABLE false
#define SERIAL_TASK_FREQUENCY 50
#define SERIAL_TASK_STACK_SIZE 5000
#define SERIAL_TASK_PRIORITY 2

// CONTROL Task
#define CONTROL_TASK_sENABLE false
#define CONTROL_TASK_FREQUENCY 10
#define CONTROL_TASK_STACK_SIZE 5000
#define CONTROL_TASK_PRIORITY 3

#endif