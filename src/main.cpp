#include <Arduino.h>

// // Include the library
// #include <ESP32Servo.h>

// // Create the servo object
// Servo myservo;

// // Setup section to run once
// void setup() {
//   myservo.attach(5); // attach the servo to our servo object
//   myservo.write(90); // stop the motor
// }

// // Loop to keep the motor turning!
// void loop() {
//   myservo.write(45); // rotate the motor counter-clockwise
//   delay(5000); // keep rotating for 5 seconds (5000 milliseconds)

//   myservo.write(90); // stop the motor
//   delay(5000); // stay stopped

//   myservo.write(135); // rotate the motor clockwise
//   delay(5000); // keep rotating 
// }


// defines pins
#define stepPin 8
#define dirPin 7
#define en 6
 
void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(en, OUTPUT);
  digitalWrite(en, HIGH);
}
void loop() {
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 1000; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(2000);    // by changing this time delay between the steps we can change the rotation speed
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(2000); 
  }
  delay(1000); // One second delay

  digitalWrite(dirPin,LOW); //Changes the rotations direction
  //digitalWrite(dirPin,LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 1000; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(1000);
  }
  delay(1000);
}

// #define ENA 41
// #define IN1 40
// #define IN2 39

// void setup(){
//   Serial.begin(9600);  
//   pinMode(ENA, OUTPUT);
//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);  
// }

// void loop(){
//   digitalWrite(ENA, HIGH);
//   digitalWrite(IN1, 150);
//   digitalWrite(IN2, LOW);
//   delay(5000);

//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, 150);
//   delay(5000);

// }