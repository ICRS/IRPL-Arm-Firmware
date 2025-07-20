// === INCLUDE === //

#include "tasks.h"
#include "config.h"
#include <Wire.h>

// === GLOBAL VARIABLES === //

// Task handles
TaskHandle_t encoderTaskHandle = nullptr;

std::array<float, N_ENCODERS> encoderAngleArray;

std::array<float, N_ENCODERS> offsetArray = {0,0,0,0/*TODO:DEFINE THESE ONCE TESTED*/};
std::array<float, N_ENCODERS> signArray = {1,1,1,1/*TODO:DEFINE THESE ONCE TESTED*/};

// === EXTERNALS === //

extern std::array<int, N_ENCODERS> currentAngleArray;

// === INTERRUPT === //

uint16_t readEncoders(int encoderAddr){
    Wire.beginTransmission(encoderAddr);
    Wire.write(START_REGISTER);
    Wire.endTransmission(false);

    uint16_t angle;

    Wire.requestFrom(encoderAddr, 2);
    if (Wire.available() >= 2){
        uint8_t angle_high = Wire.read();
        uint8_t angle_low = Wire.read();

        angle = ((uint8_t)angle_high << 8) | angle_low;

    }
    return angle;
}

void IRAM_ATTR sampleEncodersISR() {
    encoderAngleArray[0] = readEncoders(BASE_ENC_ADDR);
    encoderAngleArray[1] = readEncoders(SHOULDER_ENC_ADDR);
    encoderAngleArray[2] = readEncoders(ELBOW_ENC_ADDR);
    encoderAngleArray[3] = readEncoders(WRIST_ENC_ADDR);
}

// === FUNCTIONS === //

void convertToIKAngles(){
    for (int i = 0; i < N_ENCODERS; i++){
        currentAngleArray[i] = (signArray[i]*encoderAngleArray[i]) + offsetArray[i];
    }
}

// === TASK === //

void encoderTask(void *pvParameters) {
    (void)pvParameters;

    // Set up encoders
    Wire.begin();

    /* Make the task execute at a specified frequency */
    const TickType_t xFrequency = configTICK_RATE_HZ / ENCODER_TASK_FREQUENCY;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("Set up encoderTask");

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        Serial.println(encoderAngleArray[0]); //use for debugging, since can't print from ISR
    
        convertToIKAngles();
    }
}