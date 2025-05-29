#define BAUD_RATE 115200
#include "Arduino.h"

extern volatile float motor_0_val;
extern volatile float motor_1_val;
extern volatile float motor_2_val;
extern volatile float motor_3_val;
extern volatile float motor_4_val;
// Takes a single message like "<MOTOR:20>" and outputs the correct message 
void parseMessage(String input){
    String key;
    String value;
    if (input.startsWith("<") && input.endsWith(">")) {
        input = input.substring(1, input.length() - 1);
    } else {
        Serial.println("Not a samCereal message") ;
    }

    int colonIndex = input.indexOf(':');
    
    if (colonIndex!=-1){
        key = input.substring(0,colonIndex);
        value = input.substring(colonIndex+1);
        value.toLowerCase();
        key.toUpperCase();
    } else {
        Serial.println("No key-value pair"); // No key-value pair  
    }

    // Big if else to sort message types (change to switch case with enums at some point)
    if (key=="PING"){
        String ping_out = "<PONG:"+ String((value=="true")) +'>';
        Serial.println(ping_out);
    }else if (key=="MOTOR_0"){ //Shoulder
        motor_0_val = value.toInt();
        String motor_out = "<MOTOR_0:"+ String(motor_0_val) +'>';
        Serial.println(motor_out);
    }else if (key=="MOTOR_1"){ //Elbow
        motor_1_val = value.toInt();
        String motor_out = "<MOTOR_1:"+ String(motor_1_val) +'>';
        Serial.println(motor_out);
    }else if (key=="MOTOR_2"){ //Wrist (UP/DOWN)
        
        motor_2_val = value.toInt();
        String motor_out = "<MOTOR_2:"+ String(motor_2_val) +'>';
        Serial.println(motor_out);
    }else if (key=="MOTOR_3"){ //Base
        motor_3_val = value.toInt();
        String motor_out = "<MOTOR_3:"+ String(motor_3_val) +'>';
        Serial.println(motor_out);
    }else if (key=="MOTOR_4"){ //Wrist (Twist)
        motor_4_val = value.toInt();
        String motor_out = "<MOTOR_4:"+ String(motor_4_val) +'>';
        Serial.println(motor_out);
    }
    else{
        Serial.println("Unknown error");
    }

}

void handleSerial(){
    String incoming;

        if(Serial.available()>0){
            incoming = Serial.readStringUntil('\n');
            parseMessage(incoming);
        }
}

