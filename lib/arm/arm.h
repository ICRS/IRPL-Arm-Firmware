#ifndef ARM_H
#define ARM_H

#include "config.h"

#include <ESP32Servo.h>

// Create Servo object.
Servo gripper;

String input;
int spaceIndex;
String argString;
int argument;

class Arm {
private:

    // Period of square wave to steppers.
    float period = DEFAULT_PERIOD;

    // Steps per revolution of steppers. This can be adjusted on the drivers.
    const float steps = DEFAULT_STEPS;

    // Physical parameters of arm.
    const float gearRatioShoulder = GEAR_RATIO_SHOULDER;
    const float gearRatioElbow = GEAR_RATIO_ELBOW;
    const float gearRatioWrist = GEAR_RATIO_WRIST;
    const float gearRatioBase = GEAR_RATIO_BASE;
    const float upperArmLength = UPPER_ARM_LENGTH;
    const float foreArmLength = FORE_ARM_LENGTH;
    const float gripperLength = GRIPPER_LENGTH;

    // Physical parameters of two four bar linkages.
    const float fourBarL1 = FOUR_BAR_L1;
    const float fourBarL2 = FOUR_BAR_L2;
    const float fourBarL3 = FOUR_BAR_L3;
    const float fourBarL4 = FOUR_BAR_L4;
    const float fourBarL5 = FOUR_BAR_L5;
    const float fourBarL6 = FOUR_BAR_L6;
    const float fourBarL7 = FOUR_BAR_L7;
    const float fourBarL8 = FOUR_BAR_L8;

    // Angle of the central joint which joins the two four bar linkages.
    const float fourBarBeta = d2r(FOUR_BAR_BETA_DEG);

public:
    Arm (float pr){
        period = pr;
    }

    // Function to return the angle the wrist base gear should move to to attain a given wrist angle. 
    // The kinematics are complex so just trust they work.
    float calculateNewAngleWrist(float angleWrist, float angleShoulder, float angleElbow);

    // Function to set all motors to desired angles.
    void setMotors(float angleShoulder, float angleElbow, float angleWrist, float angleBase);

    // The wrist "roll" (rotation around its axis) is controlled by a brushed DC motor.
    // Time is the time in ms to rotate by. Start with a small time and see how it goes. 500 is a good starting point.
    void rollWrist(float time);

    // Function to move the arm forwards or upwards by a given distance in mm.
    // Inputs are "up" which takes 1 for upwards motion and 0 for forwards motion, and "distance" to be moved.
    void moveLinear(int up, float distance);

    // Mainly for debugging. Don't call this. Use setMotors directly if you are just moving one joint.
    void rotateWrist(float angle);

    // Mainly for debugging. Don't call this. Use setMotors directly if you are just moving one joint.
    void rotateBase(float angle);

    // Gripper is controlled by a crippled Servo. 
    // The servo can't read its current angle so it just goes forward and backwards when told to go to limit angles.
    // Input is time (+) to open the gripper, or negative time to close (I think, if not then swap + and -).
    // e.g. operateGripper(-300) will close the gripper for 300 ms.
    void operateGripper(float time);
};

#endif