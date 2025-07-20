// === INCLUDE === //

#include "tasks.h"
#include "config.h"

#include "PID_v1.h"
#include <ESP32Servo.h>

// === GLOBAL VARIABLES === //

// Task handles
TaskHandle_t controlTaskHandle = nullptr;

//Specify the links and initial tuning parameters
double Kp=2, Ki=0, Kd=0;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Create Servo object
// Servo gripper;
 float gripperSpeed = 0;

int wristVelocity = 0;

std::array<int, N_ENCODERS> stepPinArray;

std::array<float, N_ENCODERS> newAngleArray;

std::array<uint32_t, N_ENCODERS> stepsNum;
std::array<uint8_t, N_ENCODERS> stepsDir;

// Angles
float currentAngleBase;
float currentAngleShoulder;
float currentAngleElbow;
float currentAngleWrist;
float newAngleBase;
float newAngleShoulder;
float newAngleElbow;
float newAngleWrist;
float angleChangeWristMotor;

// === EXTERNALS === //

extern std::array<float, N_ENCODERS> desiredAngleArray;
extern std::array<float, N_ENCODERS> currentAngleArray;

void operateGripper(float normalisedSpeed);
void rollWrist(float time);

int motorCommand(int ID, int newValue){
    if (ID <= WRIST_ID){
        desiredAngleArray[ID] = newValue;
    }
    else if (ID == GRASP_ID){
        operateGripper(newValue);
    }
    else if (ID == ROLL_ID){
        rollWrist(newValue);
    }
    else{
        return -1;
    }
    return 0;
}

float motorStatus(int ID, bool isAngle){
    if (ID <= WRIST_ID){
        if (isAngle){
            return currentAngleArray[ID];
        }
        else{
            return float(stepsNum[ID]);
        }
    }
    else if (ID == GRASP_ID){
        return gripperSpeed;
    }
    else if (ID == ROLL_ID){
        return float(wristVelocity);
    }
    else return JOINT_ERROR; //TODO: This is a terrible solution to throwing an error, need to change
}

// === FUNCTIONS === //

// Gripper is controlled by a continuous servo. 
// The servo can't read its current angle so it just goes forward and backwards when told to go to limit angles.
// Input is speed of opening the gripper, with +1 opening and -1 closing (max speeds)
// e.g. operateGripper(-0.5) will close the gripper at half its maximum speed.
void operateGripper(float normalisedSpeed){
    // Limit speed
    if (abs(normalisedSpeed) > 1){
        normalisedSpeed = (normalisedSpeed<0) ? -1 : 1;
    }
    // Since the servo is continuous, writing an angle of "90" to it will stop it
    // And writing maximum angles (0, 180) to it maximises its speed in either direction
    // http://www.spt-servo.com/Product/5621733416.html
    // float newGripperSpeed = 90 + (85*normalisedSpeed);
    // gripperSpeed = newGripperSpeed;
    //gripper.write(newGripperSpeed);
    //75
    float newGripperSpeed =127 + (25*normalisedSpeed);
    gripperSpeed = newGripperSpeed;
    ledcWrite(0,newGripperSpeed);

    //TODO: Add a time limit
}

 // The wrist "roll" (rotation around its axis) is controlled by a brushed DC motor.
  // Time is the time in ms to rotate by. Start with a small time and see how it goes. 500 is a good starting point.
  void rollWrist(float time){
    // Set inputs of H-Bridge adequately to direction of rotation.
    if (time>0){
        analogWrite(ROLL_EN_PIN, 200);
        digitalWrite(IN_1_PIN, HIGH);
        digitalWrite(IN_2_PIN, LOW);
    } else {
        analogWrite(ROLL_EN_PIN, 200);
        digitalWrite(IN_1_PIN, LOW);
        digitalWrite(IN_2_PIN, HIGH);
        time = -time;
    }
    wristVelocity = (time > 0) ? 1 : -1;

    unsigned long startRollTime = micros();
    while (micros() - startRollTime < 2*abs(time)){};
    // Stop motor
    analogWrite(ROLL_EN_PIN, 0);
    digitalWrite(IN_1_PIN, LOW);
    digitalWrite(IN_2_PIN, LOW);
    wristVelocity = 0;
}

// Angle manipulation functions.
// Degree to radians.
float d2r(int deg){
    return static_cast<float>(deg)/180.0 * PI;
}

// Radians to degrees.
float r2d(float rad){
    return static_cast<float>(rad)/PI * 180.0;
}

// Function to return the angle the wrist base gear should move to to attain a given wrist angle. 
// The kinematics are complex so just trust they work.
float calculateNewAngleWrist(float angleWrist, float newAngleShoulder, float angleElbow){
    float fourBarTheta2 = d2r(angleWrist + 90);
    float fourBarDiagonal = sq(FOUR_BAR_L1) + sq(FOUR_BAR_L2) - 2*FOUR_BAR_L1*FOUR_BAR_L2*cos(fourBarTheta2);
    float fourBarCosTheta4 = (sq(FOUR_BAR_L3) + sq(FOUR_BAR_L4) - fourBarDiagonal) / (2*FOUR_BAR_L3*FOUR_BAR_L4);
    float fourBarSinTheta4 = sqrt(1 - sq(fourBarCosTheta4));
    float fourBarTheta4 = atan2(fourBarSinTheta4, fourBarCosTheta4);
    float fourBarTheta3 = d2r(180) - asin(FOUR_BAR_L2*sin(fourBarTheta2)/sqrt(fourBarDiagonal)) - asin(FOUR_BAR_L4*sin(fourBarTheta4)/sqrt(fourBarDiagonal));
    float fourBarAlpha = d2r(newAngleShoulder+angleElbow);

    float fourBarTheta2Prime = d2r(angleElbow) + fourBarTheta3 + FOUR_BAR_BETA_DEG;
    float fourBarAlphaPrime = fourBarAlpha - d2r(angleElbow);
    float fourBarDiagonalPrime = sq(FOUR_BAR_L5) + sq(FOUR_BAR_L6) - 2*FOUR_BAR_L5*FOUR_BAR_L6*cos(fourBarTheta2Prime);
    float fourBarCosTheta4Prime = (sq(FOUR_BAR_L7) + sq(FOUR_BAR_L8) - fourBarDiagonalPrime) / (2*FOUR_BAR_L7*FOUR_BAR_L8);
    float fourBarSinTheta4Prime = sqrt(1 - sq(fourBarCosTheta4Prime));
    float fourBarTheta4Prime = atan2(fourBarSinTheta4Prime, fourBarCosTheta4Prime);
    float fourBarTheta3Prime = d2r(180) - asin(FOUR_BAR_L6*sin(fourBarTheta2Prime)/sqrt(fourBarDiagonalPrime)) - asin(FOUR_BAR_L8*sin(fourBarTheta4Prime)/sqrt(fourBarDiagonalPrime));

    float fourBarInputAngle = (r2d(fourBarTheta3Prime) - newAngleShoulder);
    return fourBarInputAngle;
}

void updateSteps(){
    // Convert new and current motor angles to number of steps to be sent to motors.
    // Shoulder and base only change angle when their motors are told to. 
    stepsNum[SHOULDER_ID] = abs(newAngleShoulder - currentAngleShoulder) * GEAR_RATIO_SHOULDER * static_cast<float>(DEFAULT_STEPS) / 360.0;
    stepsNum[BASE_ID] = abs(newAngleBase - currentAngleBase) * GEAR_RATIO_BASE * static_cast<float>(DEFAULT_STEPS) / 360.0;
    // Elbow changes angle when its motor moves, but also when the elbow motor moves due to the physical implementation of power transmission.
    stepsNum[ELBOW_ID] = abs(newAngleElbow - currentAngleElbow + newAngleShoulder - currentAngleShoulder) * GEAR_RATIO_ELBOW * static_cast<float>(DEFAULT_STEPS) / 360.0;
    // Wrist angle has been dealt with above.
    stepsNum[WRIST_ID] = abs(angleChangeWristMotor) * GEAR_RATIO_WRIST * static_cast<float>(DEFAULT_STEPS) / 360.0;
    
    // Ensure all motors rotate in correct direction. From fully extend forward, moving any link "up" should be a negative angle, "down" should be positive.
    stepsDir[SHOULDER_ID] = (newAngleShoulder > currentAngleShoulder ? LOW : HIGH);
    stepsDir[ELBOW_ID] = (newAngleElbow + newAngleShoulder > currentAngleElbow + currentAngleShoulder ? HIGH : LOW);
    stepsDir[WRIST_ID] = (angleChangeWristMotor > 0 ? LOW : HIGH);
    stepsDir[BASE_ID] = (newAngleBase > currentAngleBase ? HIGH : LOW);
}

// Update current angles and wrist motor angle 
// (this is mostly for readability in other functions, and to prevent values updating partway through calculations)
void updateAngles(std::array<float, N_ENCODERS> newAngleArray){
    // Update angles
    newAngleBase = newAngleArray[BASE_ID];
    newAngleShoulder = newAngleArray[SHOULDER_ID];
    newAngleElbow = newAngleArray[ELBOW_ID];
    newAngleWrist = newAngleArray[WRIST_ID];

    currentAngleBase = currentAngleArray[BASE_ID];
    currentAngleShoulder = currentAngleArray[SHOULDER_ID];
    currentAngleElbow = currentAngleArray[ELBOW_ID];
    currentAngleWrist = currentAngleArray[WRIST_ID];

    // Calculate new wrist motor angle
    angleChangeWristMotor = calculateNewAngleWrist(newAngleWrist, newAngleShoulder, newAngleElbow) - calculateNewAngleWrist(currentAngleWrist, currentAngleShoulder, currentAngleElbow);
}

// Perform PID control on current angles and desired angles to update stepsNum and stepsDir
void updateControl(){
    for(int i=0; i<N_ENCODERS; i++){
        Input = currentAngleArray[i];
        Setpoint = desiredAngleArray[i];
        myPID.Compute();
        newAngleArray[i] = round(Output);
    }
    updateAngles(newAngleArray);
    updateSteps();
}

// Write to motors
void writeToMotors(){
    // Ensure all motors rotate in correct direction. From fully extend forward, moving any link "up" should be a negative angle, "down" should be positive.
    digitalWrite(SHOULDER_DIR_PIN, stepsDir[SHOULDER_ID]);
    digitalWrite(ELBOW_DIR_PIN, stepsDir[ELBOW_ID]);
    digitalWrite(WRIST_DIR_PIN, stepsDir[WRIST_ID]);
    digitalWrite(BASE_DIR_PIN, stepsDir[BASE_ID]);

    // Pulses each stepper motor in turn
    bool nonzeroSteps = true;
    while (nonzeroSteps){
        nonzeroSteps = false;
        for(int i = 0; i<N_ENCODERS; i++){
            if(stepsNum[i]>0){
                digitalWrite(stepPinArray[i], HIGH);
                unsigned long startTime = micros();
                nonzeroSteps = true;
                while(micros()-startTime<STEPPER_PERIOD){}
                digitalWrite(stepPinArray[i], LOW);
                startTime = micros();
                stepsNum[i] = stepsNum[i]-1;
                while(micros()-startTime<STEPPER_PERIOD){}
                //TODO: this can be parallelised
            }
        }
    }
}

void initMotors(){
    pinMode(BASE_STEP_PIN, OUTPUT);
    pinMode(BASE_DIR_PIN, OUTPUT);
    pinMode(BASE_EN_PIN, OUTPUT);
    digitalWrite(BASE_EN_PIN, HIGH);

    pinMode(SHOULDER_STEP_PIN, OUTPUT);
    pinMode(SHOULDER_DIR_PIN, OUTPUT);
    pinMode(SHOULDER_EN_PIN, OUTPUT);
    digitalWrite(SHOULDER_EN_PIN, HIGH);

    pinMode(ELBOW_STEP_PIN, OUTPUT);
    pinMode(ELBOW_DIR_PIN, OUTPUT);
    pinMode(ELBOW_EN_PIN, OUTPUT);
    digitalWrite(ELBOW_EN_PIN, HIGH);

    pinMode(WRIST_STEP_PIN, OUTPUT);
    pinMode(WRIST_DIR_PIN, OUTPUT);
    pinMode(WRIST_EN_PIN, OUTPUT);
    digitalWrite(WRIST_EN_PIN, HIGH);

    pinMode(ROLL_EN_PIN, OUTPUT);
    pinMode(IN_1_PIN, OUTPUT);
    pinMode(IN_2_PIN, OUTPUT);

    // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
    // gripper.setPeriodHertz(SERVO_FREQUENCY);
    // gripper.attach(GRASP_PIN, SERVO_MINIMUM, SERVO_MAXIMUM); // set up gripper servo on pin 8

    pinMode(GRASP_PIN,OUTPUT);
    ledcAttachPin(GRASP_PIN,0);
    ledcSetup(0,330,8);
}

void initStepPinArray(){
    stepPinArray[BASE_ID] = BASE_STEP_PIN;
    stepPinArray[SHOULDER_ID] = SHOULDER_STEP_PIN;
    stepPinArray[ELBOW_ID] = ELBOW_STEP_PIN;
    stepPinArray[WRIST_ID] = BASE_STEP_PIN;
}

// === TASK === //

void controlTask(void *pvParameters) {
    (void)pvParameters;

    myPID.SetMode(AUTOMATIC);
    initMotors();

    /* Make the task execute at a specified frequency */
    const TickType_t xFrequency = configTICK_RATE_HZ / CONTROL_TASK_FREQUENCY;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("Set up controlTask");

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // PERFORM PID
        updateControl();

        // WRITE OUTPUTS TO MOTORS
        writeToMotors();

    }
}