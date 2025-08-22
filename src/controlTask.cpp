// === INCLUDE === //

#include "tasks.h"
#include "config.h"
#include "stepper.h"

#include "PID_v1.h"
#include <ESP32Servo.h>
#define ANGLE_THRESHOLD 5.0

// === GLOBAL VARIABLES === //
Stepper shoulderMotor(SHOULDER_STEP_PIN,SHOULDER_DIR_PIN, 0);
Stepper baseMotor(BASE_STEP_PIN,BASE_DIR_PIN, 1);
Stepper elbowMotor(ELBOW_STEP_PIN,ELBOW_DIR_PIN, 1);
Stepper wristMotor(WRIST_STEP_PIN,WRIST_DIR_PIN, 0);

// Task handles
TaskHandle_t controlTaskHandle = nullptr;

// Create Servo object
// Servo gripper;
float gripperSpeed = 0;

int wristVelocity = 0;

std::array<int, N_ENCODERS> stepPinArray;


std::array<uint32_t, N_ENCODERS> stepsNum;
std::array<uint8_t, N_ENCODERS> stepsDir;

// === EXTERNALS === //

// remove
extern std::array<float, N_ENCODERS> desiredAngleArray;
extern std::array<float, N_ENCODERS> currentAngleArray;

void operateGripper(float normalisedSpeed);
void rollWrist(float time);

int motorCommand(int ID, int newValue)
{
    if (ID <= WRIST_ID)
    {
        desiredAngleArray[ID] = newValue;
    }
    else if (ID == GRASP_ID)
    {
        operateGripper(newValue);
    }
    else if (ID == ROLL_ID)
    {
        rollWrist(newValue);
    }
    else
    {
        return -1;
    }
    return 0;
}

float motorStatus(int ID, bool isAngle)
{
    if (ID <= WRIST_ID)
    {
        if (isAngle)
        {
            return currentAngleArray[ID];
        }
        else
        {
            return float(stepsNum[ID]);
        }
    }
    else if (ID == GRASP_ID)
    {
        return gripperSpeed;
    }
    else if (ID == ROLL_ID)
    {
        return float(wristVelocity);
    }
    else
        return JOINT_ERROR; // TODO: This is a terrible solution to throwing an error, need to change
}

// === FUNCTIONS === //

// Gripper is controlled by a continuous servo.
// The servo can't read its current angle so it just goes forward and backwards when told to go to limit angles.
// Input is speed of opening the gripper, with +1 opening and -1 closing (max speeds)
// e.g. operateGripper(-0.5) will close the gripper at half its maximum speed.
void operateGripper(float normalisedSpeed)
{
    // Limit speed
    if (abs(normalisedSpeed) > 1)
    {
        normalisedSpeed = (normalisedSpeed < 0) ? -1 : 1;
    }
    // Since the servo is continuous, writing an angle of "90" to it will stop it
    // And writing maximum angles (0, 180) to it maximises its speed in either direction
    // http://www.spt-servo.com/Product/5621733416.html
    // float newGripperSpeed = 90 + (85*normalisedSpeed);
    // gripperSpeed = newGripperSpeed;
    // gripper.write(newGripperSpeed);
    // 75
    float newGripperSpeed = 127 + (25 * normalisedSpeed);
    gripperSpeed = newGripperSpeed;
    ledcWrite(0, newGripperSpeed);

    // TODO: Add a time limit
}

// The wrist "roll" (rotation around its axis) is controlled by a brushed DC motor.
// Time is the time in ms to rotate by. Start with a small time and see how it goes. 500 is a good starting point.
void rollWrist(float time)
{
    // Set inputs of H-Bridge adequately to direction of rotation.
    if (time > 0)
    {
        analogWrite(ROLL_EN_PIN, 200);
        digitalWrite(IN_1_PIN, HIGH);
        digitalWrite(IN_2_PIN, LOW);
    }
    else
    {
        analogWrite(ROLL_EN_PIN, 200);
        digitalWrite(IN_1_PIN, LOW);
        digitalWrite(IN_2_PIN, HIGH);
        time = -time;
    }
    wristVelocity = (time > 0) ? 1 : -1;

    unsigned long startRollTime = micros();
    while (micros() - startRollTime < 2 * abs(time))
    {
    };
    // Stop motor
    analogWrite(ROLL_EN_PIN, 0);
    digitalWrite(IN_1_PIN, LOW);
    digitalWrite(IN_2_PIN, LOW);
    wristVelocity = 0;
}

// Angle manipulation functions.
// Degree to radians.
float d2r(int deg)
{
    return static_cast<float>(deg) / 180.0 * PI;
}

// Radians to degrees.
float r2d(float rad)
{
    return static_cast<float>(rad) / PI * 180.0;
}


void updateMotors(){
    float jointError[4];
    for(int i = 0; i<4; i++){
            jointError[i] = desiredAngleArray[i]-currentAngleArray[i];
            #ifdef TELEMETRY
            Serial.printf(">error_%d:%.2f\n",i,jointError[i]);
            #endif
        }

    baseMotor.setSpeed(jointError[0]);
    shoulderMotor.setSpeed(jointError[1]);
    elbowMotor.setSpeed(jointError[2]);
    wristMotor.setSpeed(-1*jointError[3]);
}

// === TASK === //

void controlTask(void *pvParameters)
{
    (void)pvParameters;


    /* Make the task execute at a specified frequency */
    const TickType_t xFrequency = configTICK_RATE_HZ / CONTROL_TASK_FREQUENCY;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("Set up controlTask");
    esp_task_wdt_delete(NULL);

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        updateMotors();
    }
}