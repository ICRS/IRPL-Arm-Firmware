// === INCLUDE === //

#include "tasks.h"
#include "config.h"
#include <Wire.h>

// === GLOBAL VARIABLES === //

// Task handles
TaskHandle_t encoderTaskHandle = nullptr;

std::array<float, N_ENCODERS> encoderAngleArray;
std::array<float, N_ENCODERS> prevAngleArray;
std::array<float, N_ENCODERS> accumulatedAngleArray;

std::array<float, N_ENCODERS> signArray = {1, 1, -1, -1};
std::array<float, N_ENCODERS> offsetArray = {0, -95, -235, -277};
std::array<float, N_ENCODERS> scaleArray = {1, 0.333, 1, 1};
std::array<float, N_ENCODERS> lowerLimitArray = {-75, -180, -90, -90};
std::array<float, N_ENCODERS> upperLimitArray = {75, 0, 90, 90};

// === EXTERNALS === //

extern std::array<float, N_ENCODERS> currentAngleArray;
extern std::array<float, N_ENCODERS> desiredAngleArray;
volatile bool encoder_read = false;
volatile uint16_t ph_adc_reading;


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


uint16_t read_ph_sensor(int addr){
    Wire.beginTransmission(addr);
    Wire.write(PH_START_REGISTER);
    Wire.endTransmission(false);

    uint16_t adc_reading;

    Wire.requestFrom(addr, 2);
    if (Wire.available() >= 2){
        uint8_t adc_reading_high = Wire.read();
        uint8_t adc_reading_low = Wire.read();

        adc_reading = ((uint8_t)adc_reading_high << 8) | adc_reading_low;

    }
    return adc_reading;
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
        // Step 1: Use previous raw angles to account for discontinuity
        float angle = encoderAngleArray[i];
        float angleChange = angle - prevAngleArray[i];
        if (abs(angleChange) > 340){
            if (angleChange > 0){ // go from prev:5 to angle:355, -10 degrees in reality, angleChange = 350
                angleChange = angleChange - 360;
            }
            else{ // go from prev:355 to angle:5, +10 degrees in reality, angleChange = -350
                angleChange = angleChange + 360;
            }
        }
        prevAngleArray[i] = angle; // set previous raw angle
        accumulatedAngleArray[i] = accumulatedAngleArray[i] + angleChange; // accumulate
        angle = accumulatedAngleArray[i]; // set as accumulated angle

        // Step 2: Apply sign; -1 is to reverse the sense
        if (signArray[i] == -1)
            angle = 360.0f - angle;

        // Step 3: Apply gear ratio / scale
        angle = angle * scaleArray[i]; // scale = 0.5 for 2:1

        // Step 4: Apply offset
        angle = angle + offsetArray[i];

        // Step 5: Clamp to joint limits
        // if (angle < lowerLimitArray[i])
        //     angle = lowerLimitArray[i];
        // if (angle > upperLimitArray[i])
        //     angle = upperLimitArray[i];

        currentAngleArray[i] = angle;
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

    // Initial encoder read
    encoder_read = true;
    readEncoders();
    // Set initial absolute angles and populate currentAngleArray
    accumulatedAngleArray = currentAngleArray;
    prevAngleArray = accumulatedAngleArray;
    convertToIKAngles();
    // attempt to ensure desiredAngle = currentAngle on startUp (Need to test again)
    desiredAngleArray = {0, 0, 0, 0};
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

        /* Read the 16-bit ADC output of the pH sensor. This corresponds to a voltage between 0V and 3.3V */
        ph_adc_reading = read_ph_sensor(WRIST_ENC_ADDR);
    }
}