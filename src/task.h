#ifndef TASK_H
#define TASK_H

#include <Arduino.h>

/*<-----    Task functions  ----->*/

void samsCerealTask(void * parameter);
void motorTask(void * parameter);

/*<-----    Task handles    ----->*/

extern TaskHandle_t samsCerealTaskHandle;
extern TaskHandle_t motorTaskHandle;



/*<-----    Task frequencies    ----->*/

#define SAMS_CEREAL_FREQ 50
#define MOTOR_TASK_FREQ 50



/*<-----    Task enables    ----->*/

#define ENABLE_SAMS_CEREAL true
#define ENABLE_MOTOR true

/*<-----    Shared variables    ----->*/
extern volatile int motor_0_val;
extern volatile int motor_1_val;
extern volatile int motor_2_val;
extern volatile int motor_3_val;
extern volatile int motor_4_val;

/*<-----    Shared structs  ----->*/
enum MessageType {
    PING_IN, 
    PING_OUT,
    ERROR,
    MOTOR_0,
    MOTOR_1,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4,
};
 

struct Message {
    MessageType type;
    union {
        bool pingValue;     // PING_IN, PING_OUT
        int errorCode;      // ERROR
        int motorValue;     // MOTOR
    };
};

#endif