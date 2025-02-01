// === INCLUDE === //

#include "tasks.h"
#include "config.h"

#include "arm.h"

// === GLOBAL VARIABLES === //

// Task handles
TaskHandle_t armTaskHandle = nullptr;

// Current angle of each motor. When closed loop feedback is implemented, these are the values that the encoders should feed into.
float currentAngleBase;
float currentAngleShoulder;
float currentAngleElbow;
float currentAngleWrist;

Arm armInstance = Arm(500);

// === ROUTINES === //

// Setup.
void setupArm(){
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

    pinMode(BASE_STEP_PIN, OUTPUT);
    pinMode(BASE_DIR_PIN, OUTPUT);
    pinMode(BASE_EN_PIN, OUTPUT);
    digitalWrite(BASE_EN_PIN, HIGH);

    pinMode(ROLL_EN_PIN, OUTPUT);
    pinMode(IN_1_PIN, OUTPUT);
    pinMode(IN_2_PIN, OUTPUT);

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

// === TASK === //

void armTask(void *pvParameters){
    (void)pvParameters;

    /* Make the task execute at a specified frequency */
    const TickType_t xFrequency = configTICK_RATE_HZ / ARM_TASK_FREQUENCY;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    setupArm();

    Serial.println("Set up armTask");

    for (;;)
    {
        if (Serial.available()){
            Serial.print("Enter command\n");
            Serial.print(">> \n");
            input = Serial.readStringUntil('\n');
            input.trim();
            spaceIndex = input.indexOf(' ');
            String command = input.substring(0, spaceIndex);
            if (spaceIndex != -1){
                argString = input.substring(spaceIndex + 1);
                argument = argString.toInt();
            } else {
                argument = 0;
            }
            Serial.print(command);
            Serial.print("\n");
            Serial.print(argument);
            // Move end effector up by "argument" in mm.
            if (command == "up") {
                armInstance.moveLinear(1, argument);
            
            // Move arm forward by "argument" in mm.
            } else if (command == "fwd") {
                armInstance.moveLinear(0, argument);
            
            // Initialise (calibrate) angles. Call this before moving any angles. Make sure to update the values in function as explained previously.
            } else if (command == "cal"){
                armInstance.initialiseAngles();
            
            // Open/close gripper for "argument" in ms, positive time for open, negative for close (could be other way round).
            } else if (command == "grp"){
                armInstance.operateGripper(argument);

            // Rotate shoulder BY angle (not TO angle). "sh 5" will rotate the shoulder 5 degrees forward.
            } else if (command == "sh"){
            armInstance.setMotors(currentAngleShoulder + argument, currentAngleElbow, currentAngleWrist, currentAngleBase);
            
            // Rotate elbow by angle.
            } else if (command == "e"){
            armInstance.setMotors(currentAngleShoulder, currentAngleElbow  + argument, currentAngleWrist, currentAngleBase);

            // Rotate wrist by angle.
            } else if (command == "w"){
            armInstance.setMotors(currentAngleShoulder, currentAngleElbow, currentAngleWrist + argument, currentAngleBase);

            // Rotate base by angle.
            } else if (command == "ba"){
            armInstance.setMotors(currentAngleShoulder, currentAngleElbow, currentAngleWrist , currentAngleBase+ argument);
            
            // Roll wrist by time. Positive time for one direciton, negative for other. Values of 500-1000 are decent.
            // In the future this should be controlled by angle fed back by an encoder.
            } else if (command == "ro"){
            armInstance.rollWrist(argument);
            }
        }
    }
}