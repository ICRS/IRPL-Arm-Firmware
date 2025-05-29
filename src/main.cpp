#include "motor.h"
#include "samsCerial.h"
Arm arm = Arm(500);

void setup(){

        pinMode(stepShoulder, OUTPUT);
    pinMode(dirShoulder, OUTPUT);
    pinMode(enShoulder, OUTPUT);
    digitalWrite(enShoulder, HIGH);
    pinMode(opShoulder, OUTPUT);
    digitalWrite(opShoulder, HIGH);

    pinMode(stepElbow, OUTPUT);
    pinMode(dirElbow, OUTPUT);
    pinMode(enElbow, OUTPUT);
    digitalWrite(enElbow, HIGH);
    pinMode(opElbow, OUTPUT);
    digitalWrite(opElbow, HIGH);
    

    pinMode(stepWrist, OUTPUT);
    pinMode(dirWrist, OUTPUT);
    pinMode(enWrist, OUTPUT);
    digitalWrite(enWrist, HIGH);

    pinMode(stepBase, OUTPUT);
    pinMode(dirBase, OUTPUT);
    pinMode(enBase, OUTPUT);
    digitalWrite(enBase, HIGH);

    pinMode(enRoll, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    arm.initialiseAngles();
    Serial.begin(115200);
}

void loop(){
    //delayMicroseconds(20);
    handleSerial();
    arm.setMotors(motor_0_val, motor_1_val, motor_2_val, motor_3_val);
    Serial.println("0: "+ String(motor_0_val));
    Serial.println("1: "+ String(motor_1_val));
    Serial.println("2: "+ String(motor_2_val));
    Serial.println("3: "+ String(motor_3_val));
    Serial.println("4: "+ String(motor_4_val));
    Serial.println("0: "+ String(currentAngleShoulder));
    Serial.println("1: "+ String(currentAngleElbow));
    Serial.println("2: "+ String(currentAngleBase));
    Serial.println("3: "+ String(currentAngleWrist));

}