#include <Arduino.h>

void setup(){
Serial.begin(9600);    
}

void loop(){
Serial.println("Happy Meal!");
delay(1000);

}