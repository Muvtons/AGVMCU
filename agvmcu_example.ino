#include "AGVMCU.h"

const int MAX_CMD_LEN = 200;
char serialBuffer[MAX_CMD_LEN];
int bufferIndex = 0;

void setup() {
    agvmcu.begin();
}

void loop() {
    agvmcu.update();
    
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            serialBuffer[bufferIndex] = '\0'; 
            agvmcu.processCommand(serialBuffer); 
            bufferIndex = 0; 
        } 
        else if (c != '\r') {
            if (bufferIndex < MAX_CMD_LEN - 1) {
                serialBuffer[bufferIndex++] = c;
            } else {
                bufferIndex = 0; 
                Serial.println("⚠️ Cmd too long");
            }
        }
    }
}
