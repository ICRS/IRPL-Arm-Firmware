#include "arm.h"

// Helpers
// Angle manipulation functions.
// Degree to radians.
float d2r(int deg){
    return static_cast<float>(deg)/180.0 * PI;
}

// Radians to degrees.
float r2d(float rad){
    return static_cast<float>(rad)/PI * 180.0;
}

float calculateNewAngleWrist(float angleWrist, float angleShoulder, float angleElbow){
        float fourBarTheta2 = d2r(angleWrist + 90);
        float fourBarDiagonal = sq(fourBarL1) + sq(fourBarL2) - 2*fourBarL1*fourBarL2*cos(fourBarTheta2);
        float fourBarCosTheta4 = (sq(fourBarL3) + sq(fourBarL4) - fourBarDiagonal) / (2*fourBarL3*fourBarL4);
        float fourBarSinTheta4 = sqrt(1 - sq(fourBarCosTheta4));
        float fourBarTheta4 = atan2(fourBarSinTheta4, fourBarCosTheta4);
        float fourBarTheta3 = d2r(180) - asin(fourBarL2*sin(fourBarTheta2)/sqrt(fourBarDiagonal)) - asin(fourBarL4*sin(fourBarTheta4)/sqrt(fourBarDiagonal));
        float fourBarAlpha = d2r(angleShoulder+angleElbow);

        float fourBarTheta2Prime = d2r(angleElbow) + fourBarTheta3 + fourBarBeta;
        float fourBarAlphaPrime = fourBarAlpha - d2r(angleElbow);
        float fourBarDiagonalPrime = sq(fourBarL5) + sq(fourBarL6) - 2*fourBarL5*fourBarL6*cos(fourBarTheta2Prime);
        float fourBarCosTheta4Prime = (sq(fourBarL7) + sq(fourBarL8) - fourBarDiagonalPrime) / (2*fourBarL7*fourBarL8);
        float fourBarSinTheta4Prime = sqrt(1 - sq(fourBarCosTheta4Prime));
        float fourBarTheta4Prime = atan2(fourBarSinTheta4Prime, fourBarCosTheta4Prime);
        float fourBarTheta3Prime = d2r(180) - asin(fourBarL6*sin(fourBarTheta2Prime)/sqrt(fourBarDiagonalPrime)) - asin(fourBarL8*sin(fourBarTheta4Prime)/sqrt(fourBarDiagonalPrime));

        float fourBarInputAngle = (r2d(fourBarTheta3Prime) - angleShoulder);
        return fourBarInputAngle;
}

void setMotors(float angleShoulder, float angleElbow, float angleWrist, float angleBase) {
    
    // Calculate new wrist angle. This is always necessary since the wrist angle depends on the angle of all other joints.
    float angleWristMotor = calculateNewAngleWrist(angleWrist, angleShoulder, currentAngleElbow) - calculateNewAngleWrist(currentAngleWrist, currentAngleShoulder, angleElbow);

    // Convert all motor angles to number of steps to be sent to motors.
    // Shoulder and base only change angle when their motors are told to. 
    uint32_t stepsShoulder = abs(angleShoulder - currentAngleShoulder) * gearRatioShoulder * static_cast<float>(steps) / 360.0;
    uint32_t stepsBase = abs(angleBase - currentAngleBase) * gearRatioBase * static_cast<float>(steps) / 360.0;
    // Elbow changes angle when its motor moves, but also when the elbow motor moves due to the physical implementation of power transmission.
    uint32_t stepsElbow = abs(angleElbow - currentAngleElbow + angleShoulder - currentAngleShoulder) * gearRatioElbow * static_cast<float>(steps) / 360.0;
    // Wrist angle has been dealt with above.
    uint32_t stepsWrist = abs(angleWristMotor) * gearRatioWrist * static_cast<float>(steps) / 360.0;
    
    // Ensure all motors rotate in correct direction. From fully extend forward, moving any link "up" should be a negative angle, "down" should be positive.
    digitalWrite(SHOULDER_DIR_PIN, angleShoulder > currentAngleShoulder ? LOW : HIGH);
    digitalWrite(ELBOW_DIR_PIN, angleElbow + angleShoulder > currentAngleElbow + currentAngleShoulder ? HIGH : LOW);
    digitalWrite(WRIST_DIR_PIN, angleWristMotor>0 ? LOW : HIGH);
    digitalWrite(BASE_DIR_PIN, angleBase > currentAngleBase ? HIGH : LOW);

    // Find out which motor has the most steps to take.
    uint32_t maxStep = max(max(stepsShoulder, stepsElbow), max(stepsWrist, stepsBase));

    // In order to move all motors simultaneously, a Bresenham algorithm is used. Each motor has an "error" between current angle and desired angle.
    // Each motor takes a steps when its error gets too big.
    // This way all motors rotate simultaneously for the same duration.
    // Initialise error here.
    int32_t errShoulder = stepsShoulder - maxStep / 2;
    int32_t errElbow = stepsElbow - maxStep / 2;
    int32_t errWrist = stepsWrist - maxStep / 2;
    int32_t errBase = stepsBase - maxStep / 2;

    // Send each motor a square pulse when the error gets too high.
    for (uint32_t i = 0; i < maxStep; i++) {

        if (errShoulder >= 0) {
            digitalWrite(SHOULDER_STEP_PIN, HIGH);
            delayMicroseconds(period);
            digitalWrite(SHOULDER_STEP_PIN, LOW);
            delayMicroseconds(period);
            errShoulder -= maxStep;
        }
        errShoulder += stepsShoulder;

        if (errElbow >= 0) {
            digitalWrite(ELBOW_STEP_PIN, HIGH);
            delayMicroseconds(period);
            digitalWrite(ELBOW_STEP_PIN, LOW);
            delayMicroseconds(period);
            errElbow -= maxStep;
        }
        errElbow += stepsElbow;

        if (errWrist >= 0) {
            digitalWrite(WRIST_STEP_PIN, HIGH);
            delayMicroseconds(period);
            digitalWrite(WRIST_STEP_PIN, LOW);
            delayMicroseconds(period);
            errWrist -= maxStep;
        }
        errWrist += stepsWrist;

        if (errBase >= 0) {
            digitalWrite(BASE_STEP_PIN, HIGH);
            delayMicroseconds(period);
            digitalWrite(BASE_STEP_PIN, LOW);
            delayMicroseconds(period);
            errBase -= maxStep;
        }
        errBase += stepsBase;
    }

    // Update the current angles after all movements are completed. In reality this should be done via encoders.
    currentAngleShoulder = angleShoulder;
    currentAngleElbow = angleElbow;
    currentAngleWrist = angleWrist;
    currentAngleBase = angleBase;
}

void rollWrist(float time){
    // Set inputs of H-Bridge adequately to direciton of rotation.
    if (time>0){
        analogWrite(ROLL_EN_PIN, 200);
        digitalWrite(IN_1_PIN, HIGH);
        digitalWrite(IN_2_PIN, LOW);
        delay(time);
    } else {
        analogWrite(ROLL_EN_PIN, 200);
        digitalWrite(IN_1_PIN, LOW);
        digitalWrite(IN_2_PIN, HIGH);
        delay(-time);
    }
    // Stop motor
    analogWrite(ROLL_EN_PIN, 0);
    digitalWrite(IN_1_PIN, LOW);
    digitalWrite(IN_2_PIN, LOW);
}

// IMPORTANT \\
// WHEN YOU START THE ARM UP IT DOESNT KNOW WHERE IT IS
// CALL THIS FUNCTION BEFORE YOU MOVE ANY JOINTS
// BEFORE FLASHING THE CODE, SET THESE VALUES TO THE APPROXIMATE CURRENT ANGLES OF EACH JOINT
void initialiseAngles(){
    currentAngleBase = 0;
    currentAngleShoulder = -100;
    currentAngleElbow = 160;
    currentAngleWrist = -45;
}

void moveLinear(int up, float distance){
    // Forward kinematics.
    float currentPositionX = upperArmLength*cos(d2r(currentAngleShoulder)) + foreArmLength*cos(d2r(currentAngleShoulder + currentAngleElbow)) + gripperLength*cos(d2r(currentAngleShoulder + currentAngleElbow + currentAngleWrist));
    float currentPositionY = -upperArmLength*sin(d2r(currentAngleShoulder)) - foreArmLength*sin(d2r(currentAngleShoulder + currentAngleElbow)) - gripperLength*sin(d2r(currentAngleShoulder + currentAngleElbow + currentAngleWrist));
    
    // Add distance to be moved to adequate coordinate.
    if (up){
        currentPositionY += distance;
    } else {
        currentPositionX += distance;
    }

    // phi is the absolute angle of the end effector.
    float phi = d2r(currentAngleShoulder+currentAngleElbow+currentAngleWrist);

    // These are parameters for the inverse kinematics. Trust they work.
    float X_ = currentPositionX - gripperLength*cos(phi);
    float Y_ = currentPositionY + gripperLength*sin(phi);
    float A = -2*X_*upperArmLength;
    float B = 2*Y_*upperArmLength;
    float C = sq(foreArmLength) - sq(upperArmLength) - sq(X_) - sq(Y_);
    float ksi = atan2(B, A);

    // Inverse kinematics to determine new motor angles for desired position
    float newAngleShoulder = ksi + acos(C/sqrt(sq(A)+sq(B)));
    float newAngleElbow = atan2((-Y_-upperArmLength*sin(newAngleShoulder))/foreArmLength, (X_-upperArmLength*cos(newAngleShoulder))/foreArmLength) - newAngleShoulder;
    float newAngleWrist = phi - newAngleShoulder - newAngleElbow;

    //Set motors to desired positions.
    setMotors(quadrant(r2d(newAngleShoulder)), quadrant(r2d(newAngleElbow)), quadrant(r2d(newAngleWrist)), currentAngleBase);
}

void rotateWrist(float angle){
    setMotors(quadrant(currentAngleShoulder), quadrant(currentAngleElbow), quadrant(angle), currentAngleBase);
}

void rotateBase(float angle){
    setMotors(quadrant(currentAngleShoulder), quadrant(currentAngleElbow), quadrant(currentAngleWrist), angle);
}

void operateGripper(float time){
    gripper.attach(8); // set up gripper servo on pin 8

    // Write forwards or backwards to servo.
    // If both negative and positive time operate the gripper in the same direction, try changing the value under else to 2500.
    if (time>0){
    gripper.writeMicroseconds(500);
    } else {
    gripper.writeMicroseconds(1500);
    }
    
    delay(2*abs(time));
    gripper.detach();
}