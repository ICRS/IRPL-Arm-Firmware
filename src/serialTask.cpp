// === INCLUDE === //

#include "tasks.h"
#include "config.h"

#include <queue>

// === GLOBAL VARIABLES === //

// Task handles
TaskHandle_t serialTaskHandle = nullptr;

std::queue<Message> incomingMessages;

// === EXTERNALS === //

extern std::array<float, N_ENCODERS> desiredAngleArray;
extern volatile uint16_t ph_adc_reading;

// === FUNCTIONS === //

// Takes a single message string e.g. "<DESIRED_ANGLE:2,90>" and formats to a Message object
Message parseMessage(String input){
    Message output;
    String key;
    String value1;
    String value2;
    if (input.startsWith("<") && input.endsWith(">")) {
        input = input.substring(1, input.length() - 1);
    } else {
        output.type = MessageType::ERROR;
        output.errorCode = SerialErrorCode::NOT_SERIAL_MSG;
        return output;
    }

    int colonIndex = input.indexOf(':');
    int commaIndex = input.indexOf(','); // If found, message has multiple values
    
    if (colonIndex!=-1){
        key = input.substring(0,colonIndex);
        key.toUpperCase();
        if (commaIndex!=-1){
            value1 = input.substring(colonIndex+1,commaIndex);
            value2 = input.substring(commaIndex+1);
        }
        else{
            value1 = input.substring(colonIndex+1);
        }
        value1.toLowerCase();
        value2.toLowerCase();
    } else {
        output.type = MessageType::ERROR;
        output.errorCode = SerialErrorCode::NO_KEY_VAL_PAIR;
        return output;  
    }

    // Sort message types and add to processing queue
    if (key == "PING"){
        output.type = MessageType::PING;
        output.pingValue = value1.toInt();
    }
    else if (key == "DES_VAL"){
        output.type = MessageType::DES_VAL;
        output.motorID = value1.toInt();
        output.motorValue = value2.toFloat();
    }
    else if (key == "CUR_ANG"){
        output.type = MessageType::CUR_ANG;
        output.motorID = value1.toInt();
    }
    else if (key == "CUR_POS"){
        output.type = MessageType::CUR_POS;
        output.motorID = value1.toInt();
    }
    else if (key == "PH_REQUEST"){
        output.type = MessageType::PH_PROBE;
    }
    else{
        output.type = MessageType::ERROR;
        output.errorCode = SerialErrorCode::UNKNOWN_KEY;
    }

    return output;
}

void executeCommand(Message message){
    String returnString = "";
    switch (message.type){
        case MessageType::PING:
            returnString = "<PONG:50>"; /* 50 is the arm device ID */
            break;
        case MessageType::DES_VAL:
            if (motorCommand(message.motorID, message.motorValue) == -1){
                returnString = "<ERROR_CODE:" + String(SerialErrorCode::UNKNOWN_MOTOR) + ">";
            }
            else {
                returnString = "Changing motor "+ String(message.motorID) +" to value "+ String(message.motorValue); //TODO: comment out after testing
            }
            break;
        case MessageType::CUR_ANG:
        case MessageType::CUR_POS:
            {   
                float returnVal = motorStatus(message.motorID, (message.type==MessageType::CUR_ANG));
                if (returnVal == float(JOINT_ERROR)){
                    returnString = "<ERROR_CODE:" + String(SerialErrorCode::UNKNOWN_MOTOR) + ">";
                } 
                else{
                    String code = (message.type==MessageType::CUR_ANG)? "<CUR_ANG:" : "<CUR_POS:";
                    returnString = code + String(returnVal) + ">";
                }
                break;
            } //DO NOT REMOVE THESE BRACKETS OR THIS SWITCH CASE WILL BREAK
        case MessageType::ERROR:
            returnString = "<ERROR_CODE:" + String(message.errorCode) + ">";
            break;
        case MessageType::PH_PROBE:
            returnString = "<PH_PROBE:" + String(ph_adc_reading) + ">";
            break;
        default:
            returnString = "<ERROR_CODE:" + String(SerialErrorCode::UNKNOWN_EXECUTION) + ">";
    }
    if (returnString != ""){
        Serial.println(returnString);
    }
}

// === TASK === //

void serialTask(void *pvParameters) {
    (void)pvParameters;

    /* Make the task execute at a specified frequency */
    const TickType_t xFrequency = configTICK_RATE_HZ / SERIAL_TASK_FREQUENCY;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("Set up serialTask");

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        String incoming;
        Message incomingMessage;
        
        // Check for new messages via serial
        while(Serial.available()>0){
            incoming = Serial.readStringUntil('\n');
            incomingMessage = parseMessage(incoming);
            incomingMessages.push(incomingMessage);
        }

        // Process received messages
        while(!incomingMessages.empty()){
            executeCommand(incomingMessages.front());
            incomingMessages.pop();
        }
    }
}
