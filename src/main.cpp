#define _TIMERINTERRUPT_LOGLEVEL_ 0

#define USE_TIMER_1 true

#if (defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) ||                   \
     defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) || defined(ARDUINO_AVR_ETHERNET) ||                        \
     defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT) || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO) ||                            \
     defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) ||    \
     defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
     defined(ARDUINO_AVR_PROTRINKET3FTDI))
#define USE_TIMER_2 true
#warning Using Timer1
#else
#define USE_TIMER_3 true
#warning Using Timer3
#endif

#include "Arduino.h"
#include "TimerInterrupt.h"
#include "samsCerial.h"

#define stepShoulder 10
#define dirShoulder 11
#define opShoulder 12
#define enShoulder 13

#define stepElbow 4
#define dirElbow 5
#define opElbow 6
#define enElbow 7

#define stepBase 45
#define dirBase 47
#define opBase 44
#define enBase 46

int errShoulder = 0;
int errElbow = 0;
int errBase = 0;

int motorStepsShoulder = 0;
int motorStepsElbow = 0;
int motorStepsBase = 0;
unsigned int interval = 5;
int volatile targetSteps = 0;
int error;
int shoulderdir = 0;
int elbowdir = 0;
int basedir = 0;
bool shoulderEnable = true;
bool elbowEnable = true;
bool baseEnable = true;

void stepHandle()
{
  if (shoulderEnable)
  {
    static bool shoulderToggle = false;
    digitalWrite(stepShoulder, shoulderToggle);
    shoulderToggle = !shoulderToggle;
    if (shoulderToggle)
    {
      motorStepsShoulder = motorStepsShoulder + 1 * shoulderdir;
    }
  }
  if (elbowEnable)
  {
    static bool elbowToggle = false;
    digitalWrite(stepShoulder, elbowToggle);
    elbowToggle = !elbowToggle;
    if (elbowToggle)
    {
      motorStepsShoulder = motorStepsShoulder + 1 * elbowdir;
    }
  }
  if (baseEnable)
  {
    static bool baseToggle = false;
    digitalWrite(stepShoulder, baseToggle);
    baseToggle = !baseToggle;
    if (baseToggle)
    {
      motorStepsShoulder = motorStepsShoulder + 1 * basedir;
    }
  }
}
// void StepHandleElbow()
// {
//     static bool toggle = false;
//     digitalWrite(stepElbow, toggle);
//     toggle = !toggle;
//     if(toggle){motorStepsElbow=motorStepsElbow+1*dir;}

// }

// void StepHandleBase()
// {
//     static bool toggle = false;
//     digitalWrite(stepBase, toggle);
//     toggle = !toggle;
//     if(toggle){motorStepsBase=motorStepsBase+1*dir;}

// }

#include "arduino.h"
// #include "Servo.h"
volatile float motor_0_val = 0;
volatile float motor_1_val = 0;
volatile float motor_2_val = 0;
volatile float motor_3_val = 0;
volatile float motor_4_val = 0;

// Set all pin numbers for various motors.

#define PI 3.14

unsigned long lastTimeShoulder = 0;
unsigned long lastTimeElbow = 0;
unsigned long lastTimeWrist = 0;
unsigned long lastTimeBase = 0;

// Current angle of each motor. When closed loop feedback is implemented, these are the values that the encoders should feed into.
float currentAngleBase = 0;
float currentAngleShoulder = 0;
float currentAngleElbow = 0;
float currentAngleWrist = 0;

// Create Servo object.
// Servo gripper;

String input;
int spaceIndex;
String argString;
int argument;



class Arm
{
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



public:
  Arm(float pr)
  {
    period = pr;
  }

  // Function to set all motors to desired angles.
  void setMotors(float angleShoulder, float angleElbow, float angleWrist, float angleBase)
  {
    float newAngleShoulder = currentAngleShoulder + angleShoulder;
    float newAngleElbow = currentAngleElbow + angleElbow;
    float newAngleBase = currentAngleBase + angleBase;

    // Convert all motor angles to number of steps to be sent to motors.
    // Shoulder and base only change angle when their motors are told to.
    uint32_t stepsShoulder = abs(newAngleShoulder - currentAngleShoulder) * gearRatioShoulder * static_cast<float>(steps) / 360.0;
    uint32_t stepsBase = abs(newAngleBase - currentAngleBase) * gearRatioBase * static_cast<float>(steps) / 360.0;
    // Elbow changes angle when its motor moves, but also when the elbow motor moves due to the physical implementation of power transmission.
    uint32_t stepsElbow = abs(newAngleElbow - currentAngleElbow + newAngleShoulder - currentAngleShoulder) * gearRatioElbow * static_cast<float>(steps) / 360.0;
    // Wrist angle has been dealt with above.

    // Ensure all motors rotate in correct direction. From fully extend forward, moving any link "up" should be a negative angle, "down" should be positive.
    // Ensure all motors rotate in correct direction. From fully extend forward, moving any link "up" should be a negative angle, "down" should be positive.
    digitalWrite(dirShoulder, newAngleShoulder > currentAngleShoulder ? LOW : HIGH);
    digitalWrite(dirElbow, newAngleElbow + newAngleShoulder > currentAngleElbow + currentAngleShoulder ? HIGH : LOW);
    digitalWrite(dirBase, newAngleBase > currentAngleBase ? HIGH : LOW);

    // Find out which motor has the most steps to take.

    // In order to move all motors simultaneously, a Bresenham algorithm is used. Each motor has an "error" between current angle and desired angle.
    // Each motor takes a steps when its error gets too big.
    // This way all motors rotate simultaneously for the same duration.
    // Initialise error here.

    errShoulder = stepsShoulder - motorStepsShoulder;
    errElbow = stepsElbow - motorStepsElbow;
    errBase = stepsBase - motorStepsBase;
    Serial.println("1");
    if (errShoulder > 0)
    {
      shoulderdir = -1;
      shoulderEnable = true;
      // digitalWrite(dirShoulder,HIGH);
      //  ITimer1.enableTimer();
    }
    else if (errShoulder < 0)
    {
      shoulderdir = 1;
      shoulderEnable = true;

      digitalWrite(dirShoulder, LOW);
      // ITimer1.enableTimer();
    }
    else
    {

      shoulderEnable = false;
      // ITimer1.disableTimer();
    }
    Serial.println("2");

    if (errElbow > 0)
    {
      elbowdir = 1;
      elbowEnable = true;
      digitalWrite(dirElbow, HIGH);
      // ITimer1.enableTimer();
    }
    else if (errElbow < 0)
    {
      elbowdir = -1;
      elbowEnable = true;
      digitalWrite(dirElbow, LOW);
      // ITimer1.enableTimer();
    }
    else
    {
      elbowEnable = false;
      // ITimer1.disableTimer();
    }
    Serial.println("3");

    if (errBase > 0)
    {
      basedir = 1;
      baseEnable = true;
      digitalWrite(dirElbow, HIGH);
      // ITimer3.enableTimer();
    }
    else if (errBase < 0)
    {
      basedir = -1;
      baseEnable = true;
      digitalWrite(dirBase, LOW);
      // ITimer3.enableTimer();
    }
    else
    {
      baseEnable = false;
      // ITimer3.disableTimer();
    }
    Serial.println("4");

    // Update the current angles after all movements are completed. In reality this should be done via encoders.
    if (errShoulder == 0 && errBase == 0 && errElbow == 0)
    {
      currentAngleShoulder += angleShoulder;
      currentAngleElbow += angleElbow;
      currentAngleWrist += angleWrist;
      currentAngleBase += angleBase;
    }
  }

void initialiseAngles()
{
  currentAngleBase = 0;
  currentAngleShoulder = -100;
  currentAngleElbow = 160;
  currentAngleWrist = -45;
}

};
// IMPORTANT \\
    // WHEN YOU START THE ARM UP IT DOESNT KNOW WHERE IT IS
// CALL THIS FUNCTION BEFORE YOU MOVE ANY JOINTS
// BEFORE FLASHING THE CODE, SET THESE VALUES TO THE APPROXIMATE CURRENT ANGLES OF EACH JOINT

// Function to move the arm forwards or upwards by a given distance in mm.
// Inputs are "up" which takes 1 for upwards motion and 0 for forwards motion, and "distance" to be moved.

Arm arm = Arm(500);

void setup()
{
  if (ITimer1.attachInterruptInterval(5, stepHandle))
  {
    Serial.print(F("Starting  ITimer1 OK, millis() = "));
    Serial.println(millis());
  }
  ITimer1.init();

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

  pinMode(stepBase, OUTPUT);
  pinMode(dirBase, OUTPUT);
  pinMode(enBase, OUTPUT);
  digitalWrite(enBase, HIGH);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Arm Board started!");
  arm.initialiseAngles();
}

void loop()
{
  delayMicroseconds(1000);
  arm.setMotors(motor_0_val, motor_1_val, motor_2_val, motor_3_val);
  handleSerial();
  // Serial.println("0: "+ String(motor_0_val));
  // Serial.println("1: "+ String(motor_1_val));
  // Serial.println("2: "+ String(motor_2_val));
  // Serial.println("3: "+ String(motor_3_val));
  // Serial.println("4: "+ String(motor_4_val));
  // Serial.println("0: "+ String(currentAngleShoulder));
  // Serial.println("1: "+ String(currentAngleElbow));
  // Serial.println("2: "+ String(currentAngleBase));
  // Serial.println("3: "+ String(currentAngleWrist));
}