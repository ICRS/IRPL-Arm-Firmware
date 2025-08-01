#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Joint IDs
#define BASE_ID     0
#define SHOULDER_ID 1
#define ELBOW_ID    2
#define WRIST_ID    3

#define ROLL_ID     4
#define GRASP_ID    5

#define JOINT_ERROR -1.01

// Set all pin numbers for various motors. 
#define BASE_STEP_PIN     14
#define BASE_DIR_PIN      27
#define BASE_EN_PIN       26

#define SHOULDER_STEP_PIN 25
#define SHOULDER_DIR_PIN  33
#define SHOULDER_EN_PIN   32

#define ELBOW_STEP_PIN    4
#define ELBOW_DIR_PIN     16
#define ELBOW_EN_PIN      17

#define WRIST_STEP_PIN    5
#define WRIST_DIR_PIN     18
#define WRIST_EN_PIN      19

#define ROLL_EN_PIN       0
#define GRASP_PIN         12
#define IN_1_PIN          2
#define IN_2_PIN          15

// Set all pin numbers for various encoders
#define N_ENCODERS 4
#define ENC_SCL_PIN 22
#define ENC_SDA_PIN 21

#define BASE_ENC_ADDR 0x20
#define SHOULDER_ENC_ADDR 0x21
#define ELBOW_ENC_ADDR 0x22
#define WRIST_ENC_ADDR 0x23
#define START_REGISTER 0X10

// Period of square wave to steppers.
#define STEPPER_PERIOD 500
#define DEFAULT_STEPS 400

#define SERVO_FREQUENCY 330
#define SERVO_MINIMUM 500
#define SERVO_MAXIMUM 2500

#define GEAR_RATIO_SHOULDER 100.0
#define GEAR_RATIO_ELBOW 50.0
#define GEAR_RATIO_WRIST 91.8
#define GEAR_RATIO_BASE 102.0
#define UPPER_ARM_LENGTH 325
#define FORE_ARM_LENGTH 330
#define GRIPPER_LENGTH 195

#define FOUR_BAR_L1 330
#define FOUR_BAR_L2 110
#define FOUR_BAR_L3 142.147
#define FOUR_BAR_L4 330
#define FOUR_BAR_L5 325
#define FOUR_BAR_L6 142.147
#define FOUR_BAR_L7 200
#define FOUR_BAR_L8 320

#define FOUR_BAR_BETA_DEG -101.42

// === TASKS === //
#define HARDWARE_TIMER_PRESCALER 80 // FOR AN ESP32

// DEBUG Task
#define DEBUG_TASK_ENABLE false
#define DEBUG_TASK_FREQUENCY 10
#define DEBUG_TASK_STACK_SIZE 2500
#define DEBUG_TASK_PRIORITY 1

// ENCODER Task
#define ENCODER_TASK_ENABLE false
#define ENCODER_TASK_FREQUENCY 50
#define ENCODER_TASK_STACK_SIZE 5000
#define ENCODER_TASK_PRIORITY 4

// SERIAL Task
#define SERIAL_TASK_ENABLE true
#define SERIAL_TASK_FREQUENCY 50
#define SERIAL_TASK_STACK_SIZE 5000
#define SERIAL_TASK_PRIORITY 2

// CONTROL Task
#define CONTROL_TASK_ENABLE true
#define CONTROL_TASK_FREQUENCY 10
#define CONTROL_TASK_STACK_SIZE 5000
#define CONTROL_TASK_PRIORITY 3

#endif