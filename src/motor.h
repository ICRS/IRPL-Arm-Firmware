#include "arduino.h"
#include "Servo.h"
volatile float motor_0_val = 0;
volatile float motor_1_val = 0;
volatile float motor_2_val = 0;
volatile float motor_3_val = 0;
volatile float motor_4_val = 0;

// Set all pin numbers for various motors.
#define stepShoulder 10
#define dirShoulder  11
#define opShoulder 12
#define enShoulder   13

#define stepElbow    4
#define dirElbow     5
#define opElbow   6
#define enElbow      7

#define stepWrist    30
#define dirWrist     29
#define enWrist      28

#define stepBase     45
#define dirBase      47
#define opBase       44
#define enBase       46

#define enRoll       9
#define in1          4
#define in2          5

#define PI 3.14

unsigned long lastTimeShoulder = 0;
unsigned long lastTimeElbow = 0;
unsigned long lastTimeWrist = 0;
unsigned long lastTimeBase = 0;




// Current angle of each motor. When closed loop feedback is implemented, these are the values that the encoders should feed into.
float currentAngleBase;
float currentAngleShoulder;
float currentAngleElbow;
float currentAngleWrist;

// Create Servo object.
Servo gripper;

String input;
int spaceIndex;
String argString;
int argument;

// Angle manipulation functions.
// Degree to radians.
float d2r(int deg){
    return static_cast<float>(deg)/180.0 * PI;
}

// Radians to degrees.
float r2d(float rad){
    return static_cast<float>(rad)/PI * 180.0;
}

// Ensure correct quadrant so that the arm rotates in the right direction.
float quadrant(float angle){
  if (angle > 180) {
    return (angle - 360);
  } else if (angle < -180) {
    return (angle + 360);
  } else {
    return angle;
  }
}

class Arm {
private:

    // Period of square wave to steppers.
    float period = 500;

    // Steps per revolution of steppers. This can be adjusted on the drivers.
    const float steps = 400;

    // Physical parameters of arm.
    const float gearRatioShoulder = 100.0;
    const float gearRatioElbow = 50.0;
    const float gearRatioWrist = 91.8;
    const float gearRatioBase = 102.0;
    const float upperArmLength = 325;
    const float foreArmLength = 330;
    const float gripperLength = 195;

    // Physical parameters of two four bar linkages.
    const float fourBarL1 = 330;
    const float fourBarL2 = 110;
    const float fourBarL3 = 142.147;
    const float fourBarL4 = 330;
    const float fourBarL5 = 325;
    const float fourBarL6 = 142.147;
    const float fourBarL7 = 200;
    const float fourBarL8 = 320;

    // Angle of the central joint which joins the two four bar linkages.
    const float fourBarBeta = d2r(-101.42);

public:
    Arm (float pr){
        period = pr;
    }

    // Function to return the angle the wrist base gear should move to to attain a given wrist angle. 
    // The kinematics are complex so just trust they work.
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

  // Function to set all motors to desired angles.
  void setMotors(float angleShoulder, float angleElbow, float angleWrist, float angleBase) {
    float newAngleShoulder = currentAngleShoulder + angleShoulder;
    float newAngleElbow = currentAngleElbow + angleElbow;
    float newAngleWrist = currentAngleWrist + angleWrist;
    float newAngleBase = currentAngleBase + angleBase;
    
    // Calculate new wrist angle. This is always necessary since the wrist angle depends on the angle of all other joints.
    float angleWristMotor = calculateNewAngleWrist(angleWrist, angleShoulder, currentAngleElbow) - calculateNewAngleWrist(currentAngleWrist, currentAngleShoulder, angleElbow);

    // Convert all motor angles to number of steps to be sent to motors.
    // Shoulder and base only change angle when their motors are told to. 
    uint32_t stepsShoulder = abs(newAngleShoulder - currentAngleShoulder) * gearRatioShoulder * static_cast<float>(steps) / 360.0;
    uint32_t stepsBase = abs(newAngleBase - currentAngleBase) * gearRatioBase * static_cast<float>(steps) / 360.0;
    // Elbow changes angle when its motor moves, but also when the elbow motor moves due to the physical implementation of power transmission.
    uint32_t stepsElbow = abs(newAngleElbow - currentAngleElbow + newAngleShoulder - currentAngleShoulder) * gearRatioElbow * static_cast<float>(steps) / 360.0;
    // Wrist angle has been dealt with above.
    uint32_t stepsWrist = abs(angleWristMotor) * gearRatioWrist * static_cast<float>(steps) / 360.0;
    
    // Ensure all motors rotate in correct direction. From fully extend forward, moving any link "up" should be a negative angle, "down" should be positive.
    // Ensure all motors rotate in correct direction. From fully extend forward, moving any link "up" should be a negative angle, "down" should be positive.
    digitalWrite(dirShoulder, newAngleShoulder > currentAngleShoulder ? LOW : HIGH);
    digitalWrite(dirElbow, newAngleElbow + newAngleShoulder > currentAngleElbow + currentAngleShoulder ? HIGH : LOW);
    digitalWrite(dirWrist, angleWristMotor>0 ? LOW : HIGH);
    digitalWrite(dirBase, newAngleBase > currentAngleBase ? HIGH : LOW);

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

    unsigned long currentMicros = micros();

        if (errShoulder >= 0) {
          if (currentMicros - lastTimeShoulder >= period){
            Serial.print("hi");
            digitalWrite(stepShoulder, HIGH);
            delayMicroseconds(2);
            digitalWrite(stepShoulder, LOW);
            delayMicroseconds(2);
            lastTimeShoulder = currentMicros;
          }
          errShoulder -= 1;
        }
        //Serial.print("no");
        errShoulder += stepsShoulder;

        if (errElbow >= 0) {
          if (currentMicros - lastTimeElbow >= period){
            digitalWrite(stepElbow, HIGH);
            delayMicroseconds(2);
            digitalWrite(stepElbow, LOW);
            delayMicroseconds(2);
            errElbow -= 1;
            lastTimeElbow = currentMicros;
          }
        }
        errElbow += stepsElbow;

        if (errWrist >= 0) {
          if (currentMicros - lastTimeWrist >= period){
            digitalWrite(stepWrist, HIGH);
            delayMicroseconds(2);
            digitalWrite(stepWrist, LOW);
            delayMicroseconds(2);
            errWrist -= 1;
            lastTimeWrist = currentMicros;
          }
        }
        errWrist += stepsWrist;

        if (errBase >= 0) {
          if (currentMicros - lastTimeBase >= period){
            digitalWrite(stepBase, HIGH);
            delayMicroseconds(2);
            digitalWrite(stepBase, LOW);
            delayMicroseconds(2);
            errBase -= 1;
            lastTimeBase = currentMicros;
          }
        }
        errBase += stepsBase;

    // Send each motor a square pulse when the error gets too high.
    // for (uint32_t i = 0; i < maxStep; i++) {
    //     Serial.println(String(errShoulder));
    //     if (errShoulder >= 0) {
    //       if (currentMicros - lastTimeShoulder >= period){
    //         Serial.print("hi");
    //         digitalWrite(stepShoulder, HIGH);
    //         delayMicroseconds(50);
    //         digitalWrite(stepShoulder, LOW);
    //         delayMicroseconds(50);
    //         lastTimeShoulder = currentMicros;
    //       }
    //       errShoulder -= maxStep;
    //     }
    //     //Serial.print("no");
    //     errShoulder += stepsShoulder;

    //     if (errElbow >= 0) {
    //       if (currentMicros - lastTimeElbow >= period){
    //         digitalWrite(stepElbow, HIGH);
    //         delayMicroseconds(2);
    //         digitalWrite(stepElbow, LOW);
    //         delayMicroseconds(2);
    //         errElbow -= maxStep;
    //         lastTimeElbow = currentMicros;
    //       }
    //     }
    //     errElbow += stepsElbow;

    //     if (errWrist >= 0) {
    //       if (currentMicros - lastTimeWrist >= period){
    //         digitalWrite(stepWrist, HIGH);
    //         delayMicroseconds(2);
    //         digitalWrite(stepWrist, LOW);
    //         delayMicroseconds(2);
    //         errWrist -= maxStep;
    //         lastTimeWrist = currentMicros;
    //       }
    //     }
    //     errWrist += stepsWrist;

    //     if (errBase >= 0) {
    //       if (currentMicros - lastTimeBase >= period){
    //         digitalWrite(stepBase, HIGH);
    //         delayMicroseconds(2);
    //         digitalWrite(stepBase, LOW);
    //         delayMicroseconds(2);
    //         errBase -= maxStep;
    //         lastTimeBase = currentMicros;
    //       }
    //     }
    //     errBase += stepsBase;
    // }

    // Update the current angles after all movements are completed. In reality this should be done via encoders.
    currentAngleShoulder += angleShoulder;
    currentAngleElbow += angleElbow;
    currentAngleWrist += angleWrist;
    currentAngleBase += angleBase;
}

  // The wrist "roll" (rotation around its axis) is controlled by a brushed DC motor.
  // Time is the time in ms to rotate by. Start with a small time and see how it goes. 500 is a good starting point.
  void rollWrist(float time){
      // Set inputs of H-Bridge adequately to direciton of rotation.
      if (time>0){
        analogWrite(enRoll, 200);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        delay(time);
      } else {
        analogWrite(enRoll, 200);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        delay(-time);
      }
      // Stop motor
      analogWrite(enRoll, 0);
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
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

    // Function to move the arm forwards or upwards by a given distance in mm.
    // Inputs are "up" which takes 1 for upwards motion and 0 for forwards motion, and "distance" to be moved.
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

    // Mainly for debugging. Don't call this. Use setMotors directly if you are just moving one joint.
    void rotateWrist(float angle){
      setMotors(quadrant(currentAngleShoulder), quadrant(currentAngleElbow), quadrant(angle), currentAngleBase);
    }

    // Mainly for debugging. Don't call this. Use setMotors directly if you are just moving one joint.
    void rotateBase(float angle){
      setMotors(quadrant(currentAngleShoulder), quadrant(currentAngleElbow), quadrant(currentAngleWrist), angle);
    }

    // Gripper is controlled by a crippled Servo. 
    // The servo can't read its current angle so it just goes forward and backwards when told to go to limit angles.
    // Input is time (+) to open the gripper, or negative time to close (I think, if not then swap + and -).
    // e.g. operateGripper(-300) will close the gripper for 300 ms.
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
};

