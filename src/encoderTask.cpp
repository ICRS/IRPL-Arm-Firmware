// === INCLUDE === //

#include "tasks.h"
#include "config.h"
#include <Wire.h>

// === GLOBAL VARIABLES === //

// Task handles
TaskHandle_t encoderTaskHandle = nullptr;

std::array<float, N_ENCODERS> encoderAngleArray;

std::array<float, N_ENCODERS> offsetArray = {0,414,-90,31/*TODO:DEFINE THESE ONCE TESTED*/};
std::array<float, N_ENCODERS> signArray = {1,1,-1,1/*TODO:DEFINE THESE ONCE TESTED*/};

// === EXTERNALS === //

extern std::array<int, N_ENCODERS> currentAngleArray;
extern std::array<int, N_ENCODERS> desiredAngleArray;
volatile bool encoder_read = false;

uint16_t encode(int encoderAddr){
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
    angle = angle & 0x0FFF; // mask to 12 bits
    angle = (angle * 360.0) / 4096.0;
    return angle;
}


// === INTERRUPT === //

void readEncoders(){
    if (encoder_read == true){
        encoder_read = false;
        encoderAngleArray[0] = 0;
        encoderAngleArray[1] = encode(SHOULDER_ENC_ADDR);
        encoderAngleArray[2] = encode(ELBOW_ENC_ADDR);
        encoderAngleArray[3] = encode(WRIST_ENC_ADDR);
    }
    
}


void IRAM_ATTR sampleEncodersISR() {
    encoder_read = true;
}

// === FUNCTIONS === //

void convertToIKAngles(){
    for (int i = 0; i < N_ENCODERS; i++){
        if (signArray[i] == -1){
            encoderAngleArray[i] = 360 - encoderAngleArray[i];
        }
        //Attempts to ensure angle values loop round correctly (untested)
        double adjusted = fmod(encoderAngleArray[i] - offsetArray[i],360);
        currentAngleArray[i] = adjusted;
        Serial.println(currentAngleArray[i]);
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

    encoder_read = true;
    //attempt to ensure desiredAngle = currentAngle on startUp (Need to test again)
    desiredAngleArray = {0, 0, 0, 0};
    readEncoders();
    convertToIKAngles();
    desiredAngleArray = currentAngleArray;
    
    Serial.println("Desired start");
    for (int i=0; i<N_ENCODERS; i++){
        Serial.println(desiredAngleArray[i]);
    }
    

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        //Serial.println(encoderAngleArray[0]); //use for debugging, since can't print from ISR
        Serial.println("Current");
        readEncoders();
        convertToIKAngles();
    }
}