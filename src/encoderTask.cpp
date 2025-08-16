// === INCLUDE === //

#include "tasks.h"
#include "config.h"
#include <Wire.h>

// === GLOBAL VARIABLES === //

// Task handles
TaskHandle_t encoderTaskHandle = nullptr;

std::array<float, N_ENCODERS> encoderAngleArray;

std::array<float, N_ENCODERS> offsetArray = {0, 323, 277, 0 /*TODO:DEFINE THESE ONCE TESTED*/};
std::array<float, N_ENCODERS> signArray = {1, 1, -1, 1 /*TODO:DEFINE THESE ONCE TESTED*/};
std::array<float, N_ENCODERS> scaleArray = {1, 0.25, 1, 1 /*TODO:DEFINE THESE ONCE TESTED*/};

// === EXTERNALS === //

extern std::array<float, N_ENCODERS> currentAngleArray;
extern std::array<float, N_ENCODERS> desiredAngleArray;
volatile bool encoder_read = false;

float encode(int encoderAddr)
{
    Wire.beginTransmission(encoderAddr);
    Wire.write(START_REGISTER);
    Wire.endTransmission(false);

    uint16_t angle;
    float angle_float;
    Wire.requestFrom(encoderAddr, 2);
    if (Wire.available() >= 2)
    {
        uint8_t angle_high = Wire.read();
        uint8_t angle_low = Wire.read();

        angle = ((uint8_t)angle_high << 8) | angle_low;
    }
    angle = angle & 0x0FFF; // mask to 12 bits
    angle_float = (angle * 360.0) / 4095.0;
    return angle_float;
}

// === INTERRUPT === //

void readEncoders()
{
    if (encoder_read == true)
    {
        encoder_read = false;
        encoderAngleArray[0] = 0;
        encoderAngleArray[1] = encode(SHOULDER_ENC_ADDR);
        encoderAngleArray[2] = encode(ELBOW_ENC_ADDR);
        encoderAngleArray[3] = encode(WRIST_ENC_ADDR);
    }
}

void IRAM_ATTR sampleEncodersISR()
{
    encoder_read = true;
}

// === FUNCTIONS === //

#include <math.h>

void convertToIKAngles()
{
    for (int i = 0; i < N_ENCODERS; i++)
    {
        // Step 1: Wrap encoder angle
        float angle = fmod(encoderAngleArray[i], 360.0f);
        if (angle < 0)
            angle += 360.0f;

        // Step 2: Apply sign
        if (signArray[i] == -1)
            angle = 360.0f - angle;

        // Step 3: Apply offset
        float delta = angle - offsetArray[i] / scaleArray[i];

        // Optional: wrap delta to [0,360)
        delta = fmod(delta, 360.0f);
        if (delta < 0)
            delta += 360.0f;

        // Step 4: Apply gear ratio / scale
        float jointAngle = delta * scaleArray[i]; // scale = 0.5 for 2:1

        if (i == SHOULDER_ID && jointAngle <45){
            jointAngle += 90;
        }

        // Step 5: Clamp to joint limits
        if (jointAngle < 0)
            jointAngle = 0;
        if (jointAngle > 180)
            jointAngle = 180;

        currentAngleArray[i] = jointAngle;
        #ifdef TELEMETRY
        Serial.printf(">%d:%.2f\n", i, currentAngleArray[i]);
        #endif
    }
}

void printAngles()
{
    Serial.printf("<CUR_ANG:%.2f,%.2f,%.2f,%.2f>\n", currentAngleArray[0], currentAngleArray[1], currentAngleArray[2], currentAngleArray[3]);
}

// === TASK === //

void encoderTask(void *pvParameters)
{
    (void)pvParameters;

    // Set up encoders
    Wire.begin();

    /* Make the task execute at a specified frequency */
    const TickType_t xFrequency = configTICK_RATE_HZ / ENCODER_TASK_FREQUENCY;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    encoder_read = true;
    // attempt to ensure desiredAngle = currentAngle on startUp (Need to test again)
    desiredAngleArray = {0, 0, 0, 0};
    readEncoders();
    convertToIKAngles();
    desiredAngleArray = currentAngleArray;

    Serial.println("Desired start");
    for (int i = 0; i < N_ENCODERS; i++)
    {
        Serial.println(desiredAngleArray[i]);
    }

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        readEncoders();
        convertToIKAngles();
        #ifndef TELEMETRY
        printAngles();
        #endif
    }
}