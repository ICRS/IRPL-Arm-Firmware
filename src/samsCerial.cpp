#include <task.h>
#define BAUD_RATE 115200
TaskHandle_t samsCerealTaskHandle = nullptr;

// Takes a single message like "<MOTOR:20>" and outputs the correct message 
Message parseMessage(String input){
    Message output;
    String key;
    String value;
    if (input.startsWith("<") && input.endsWith(">")) {
        input = input.substring(1, input.length() - 1);
    } else {
        output.type = MessageType::ERROR;
        output.errorCode = 0; // Not a samsCereal message 
        return output;  
    }

    int colonIndex = input.indexOf(':');
    
    if (colonIndex!=-1){
        key = input.substring(0,colonIndex);
        value = input.substring(colonIndex+1);
        value.toLowerCase();
        key.toUpperCase();
    } else {
        output.type = MessageType::ERROR;
        output.errorCode = 1; // No key-value pair
        return output;  
    }

    // Big if else to sort message types (change to switch case with enums at some point)
    if (key=="PING"){
        output.type = MessageType::PING_IN;
        output.pingValue = (value=="true");
        String ping_out = "<PONG:"+ String(output.pingValue) +'>';
        Serial.println(ping_out);
    }else if (key=="MOTOR_0"){ //Shoulder
        output.type = MessageType::MOTOR_0;
        output.motorValue = value.toInt();
        motor_0_val = output.motorValue;
        String motor_out = "<MOTOR_0:"+ String(motor_0_val) +'>';
        Serial.println(motor_out);
    }else if (key=="MOTOR_1"){ //Elbow
        output.type = MessageType::MOTOR_1;
        output.motorValue = value.toInt();
        motor_1_val = output.motorValue;
        String motor_out = "<MOTOR_1:"+ String(motor_1_val) +'>';
        Serial.println(motor_out);
    }else if (key=="MOTOR_2"){ //Wrist (UP/DOWN)
        output.type = MessageType::MOTOR_2;
        output.motorValue = value.toInt();
        motor_2_val = output.motorValue;
        String motor_out = "<MOTOR_2:"+ String(motor_2_val) +'>';
        Serial.println(motor_out);
    }else if (key=="MOTOR_3"){ //Base
        output.type = MessageType::MOTOR_3;
        output.motorValue = value.toInt();
        motor_3_val = output.motorValue;
        String motor_out = "<MOTOR_3:"+ String(motor_3_val) +'>';
        Serial.println(motor_out);
    }else if (key=="MOTOR_4"){ //Wrist (Twist)
        output.type = MessageType::MOTOR_4;
        output.motorValue = value.toInt();
        motor_4_val = output.motorValue;
        String motor_out = "<MOTOR_4:"+ String(motor_4_val) +'>';
        Serial.println(motor_out);
    }
    else{
        output.type = MessageType::ERROR;
        output.errorCode = 3; // Unknown key
    }

    return output;
}



void samsCerealTask(void * parameter){

    Serial.begin(BAUD_RATE);
    Message incomingMessage;

    for(;;){
        String incoming;
        if(Serial.available()>0){
            incoming = Serial.readStringUntil('\n');
            incomingMessage = parseMessage(incoming);
            
            if(incomingMessage.type==MessageType::ERROR){
                Serial.print("Failed to parse message with code: ");
                Serial.println(incomingMessage.errorCode);
            }
        }
        vTaskDelay((1000/SAMS_CEREAL_FREQ) / portTICK_PERIOD_MS);
    }
}
