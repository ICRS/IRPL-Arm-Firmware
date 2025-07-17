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
    else if (key == "DES_ANG"){
        output.type = MessageType::DES_ANG;
        output.motorID = value1.toInt();
        output.angleValue = value2.toFloat();
    }
    else if (key == "DES_POS"){
        output.type = MessageType::DES_POS;
        output.motorID = value1.toInt();
        output.positionValue = value2.toInt();
    }
    else if (key == "CUR_ANG"){
        output.type = MessageType::CUR_ANG;
        output.motorID = value1.toInt();
    }
    else if (key == "CUR_POS"){
        output.type = MessageType::CUR_POS;
        output.motorID = value1.toInt();
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
            returnString = "<PONG:"+ String(message.pingValue) + ">";
            break;
        case MessageType::DES_ANG:
            motorCommand(message.motorID, message.angleValue, true);
            returnString = "Moving motor "+ String(message.motorID) +" to angle "+ String(message.angleValue); //TODO: comment out after testing
            break;
        case MessageType::DES_POS:
            motorCommand(message.motorID, message.positionValue, false);
            returnString = "Moving motor "+ String(message.motorID) +" to position "+ String(message.positionValue); //TODO: comment out after testing
            break;
        case MessageType::CUR_ANG:
            returnString = "<CUR_ANG:"+ String(100) +","+ String(90) + ">"; //TODO: add data once encoder task implemented
            break;
        case MessageType::CUR_POS:
            returnString = "<CUR_POS:"+ String(101) +","+ String(1024) + ">"; //TODO: add data once encoder task implemented
            break;
        case MessageType::ERROR:
            returnString = "<ERROR_CODE:" + String(message.errorCode) + ">";
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
