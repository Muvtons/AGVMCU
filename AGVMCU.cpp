#include "AGVMCU.h"

// Global instance
AGVMCU agvmcu;

AGVMCU::AGVMCU() 
    : distanceThreshold(10.0), distanceBelowThreshold(false),
      totalSteps(0), currentStepIndex(0), currentX(-1), currentY(-1), 
      currentDir('E'), currentState(STATE_IDLE),
      lastAbortPress(0), lastStartPress(0), lastStopPress(0),
      distanceBelowThresholdStart(0) {
}

void AGVMCU::begin(long baudRate) {
    Serial.begin(baudRate);
    
    // Initialize motor control pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Initialize button pins with internal pull-up resistors
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(ABORT_BUTTON_PIN, INPUT_PULLUP);

    // Initialize all pins to safe state
    stopMotors();
    
    Serial.println("✅ AGVMCU initialized - Ready for commands");
}

void AGVMCU::update() {
    // HIGH PRIORITY: Handle physical buttons every cycle
    handleButtons();
    
    // MEDIUM PRIORITY: Handle state-specific continuous actions
    switch(currentState) {
        case STATE_MOVING:
            // Movement is handled in navigateToNextStep() - time based
            break;
        case STATE_OBSTACLE_RECOVERY:
            // Waiting for QR scan after obstacle
            break;
        case STATE_ABORTED:
            // Waiting for QR scan after abort
            break;
        default:
            // Other states don't need continuous processing
            break;
    }
}

void AGVMCU::processCommand(String command) {
    Serial.print("[AGVMCU] Processing: ");
    Serial.println(command);

    // HIGHEST PRIORITY: Immediate command processing
    if(command == "ABORT") {
        handleAbort();
        return;
    }

    if(command == "STOP") {
        handleStop();
        return;
    }

    if(command == "START") {
        handleStart();
        return;
    }

    // HIGH PRIORITY: Distance sensor data
    if(command.startsWith("DISTANCE:")) {
        String distanceStr = command.substring(9);
        float currentDistance = distanceStr.toFloat();
        
        if(currentDistance < distanceThreshold) {
            if(!distanceBelowThreshold) {
                distanceBelowThreshold = true;
                distanceBelowThresholdStart = millis();
                
                if(currentState == STATE_MOVING) {
                    stopMotors();
                    handleObstacle();
                }
            } else if(millis() - distanceBelowThresholdStart >= 10000) {
                handleObstacle();
            }
        } else {
            distanceBelowThreshold = false;
        }
        return;
    }

    // MEDIUM PRIORITY: QR code data
    if(command.startsWith("QR:")) {
        String coordData = command.substring(3);
        int firstComma = coordData.indexOf(',');
        int secondComma = coordData.indexOf(',', firstComma + 1);
        
        if(secondComma != -1) {
            int qrX = coordData.substring(0, firstComma).toInt();
            int qrY = coordData.substring(firstComma + 1, secondComma).toInt();
            float qrAngle = coordData.substring(secondComma + 1).toFloat();

            currentX = qrX;
            currentY = qrY;
            correctDirectionUsingQRAngle(qrAngle);

            switch(currentState) {
                case STATE_WAITING_FOR_QR:
                    currentState = STATE_MOVING;
                    navigateToNextStep();
                    break;
                    
                case STATE_WAITING_QR_CONFIRMATION: {
                    Step targetStep = path[currentStepIndex];
                    if(isAtPosition(targetStep.x, targetStep.y)) {
                        currentStepIndex++;
                        publishCurrentPosition();
                        
                        if(currentStepIndex >= totalSteps) {
                            Serial.println("DESTINATION REACHED");
                            currentState = STATE_GOAL_REACHED;
                        } else {
                            currentState = STATE_MOVING;
                            navigateToNextStep();
                        }
                    } else {
                        Serial.println("NAV_FAILED: Wrong QR position");
                        rotateAngle(180);
                        currentState = STATE_STOPPED;
                    }
                    break;
                }
                
                case STATE_OBSTACLE_RECOVERY:
                    Serial.print("currentpos after blocked : ");
                    Serial.print(currentX); Serial.print(","); Serial.println(currentY);
                    Serial.print("blocked qr : ");
                    Serial.print(currentX); Serial.print(","); Serial.println(currentY);
                    currentState = STATE_IDLE;
                    totalSteps = 0;
                    break;
                    
                case STATE_ABORTED:
                    Serial.print("currentpos after abort : ");
                    Serial.print(currentX); Serial.print(","); Serial.println(currentY);
                    currentState = STATE_IDLE;
                    totalSteps = 0;
                    break;
            }
        }
        return;
    }

    // LOWEST PRIORITY: Path data (processed last)
    if(command.startsWith("(")) {
        parsePath(command);
        if(totalSteps > 0) {
            currentStepIndex = 0;

            if(currentX == -1 || currentY == -1) {
                currentState = STATE_WAITING_FOR_QR;
                Serial.println("Waiting for initial QR...");
            } else {
                currentState = STATE_MOVING;
                navigateToNextStep();
            }
        }
        return;
    }
}

// Motor Control Functions (same as before)
void AGVMCU::moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
}

void AGVMCU::moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
}

void AGVMCU::stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void AGVMCU::rotateAngle(float degrees) {
    if(degrees > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
    
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);

    unsigned long rotateTime = (unsigned long)(abs(degrees) * 8.5);
    delay(rotateTime);
    stopMotors();
    delay(500);
}

void AGVMCU::rotateToDirection(char from, char to) {
    int angleDiff = 0;
    if((from == 'E' && to == 'N') || (from == 'N' && to == 'W') || 
       (from == 'W' && to == 'S') || (from == 'S' && to == 'E')) {
        angleDiff = 90;
    } else if((from == 'E' && to == 'S') || (from == 'S' && to == 'W') || 
              (from == 'W' && to == 'N') || (from == 'N' && to == 'E')) {
        angleDiff = -90;
    } else if(from != to) {
        angleDiff = 180;
    }
    
    if(angleDiff != 0) {
        rotateAngle(angleDiff);
        currentDir = to;
    }
}

void AGVMCU::correctDirectionUsingQRAngle(float qrAngle) {
    float targetAngle = 0.0;
    switch(currentDir) {
        case 'E': targetAngle = 0.0; break;
        case 'N': targetAngle = 90.0; break;
        case 'W': targetAngle = 180.0; break;
        case 'S': targetAngle = -90.0; break;
    }
    
    float angleError = targetAngle - qrAngle;
    while(angleError > 180) angleError -= 360;
    while(angleError < -180) angleError += 360;
    
    if(abs(angleError) > 5.0) {
        rotateAngle(angleError);
    }
}

void AGVMCU::navigateToNextStep() {
    if(currentStepIndex >= totalSteps) {
        Serial.println("DESTINATION REACHED");
        currentState = STATE_GOAL_REACHED;
        return;
    }

    Step targetStep = path[currentStepIndex];

    // Check if direction needs to be changed
    if(currentDir != targetStep.dir) {
        rotateToDirection(currentDir, targetStep.dir);
    }

    currentState = STATE_MOVING;
    
    // Move one cell (2 seconds)
    moveForward();
    delay(2000);
    stopMotors();
    
    currentState = STATE_WAITING_QR_CONFIRMATION;
}

bool AGVMCU::isAtPosition(int x, int y) {
    return (currentX == x && currentY == y);
}

void AGVMCU::parsePath(String raw) {
    totalSteps = 0;
    raw.trim();
    int i = 0;
    while(i < raw.length() && totalSteps < 50) {
        if(raw[i] == '(') {
            int commaIndex = raw.indexOf(',', i);
            int endIndex = raw.indexOf(')', commaIndex);
            if(commaIndex == -1 || endIndex == -1) break;
            int x = raw.substring(i + 1, commaIndex).toInt();
            int y = raw.substring(commaIndex + 1, endIndex).toInt();
            char dir = 'E';
            if(endIndex + 1 < raw.length()) {
                dir = raw[endIndex + 1];
            }
            if(dir == 'N' || dir == 'S' || dir == 'E' || dir == 'W') {
                path[totalSteps++] = {x, y, dir};
            }
            i = endIndex + 2;
        } else {
            i++;
        }
    }
}

void AGVMCU::publishCurrentPosition() {
    Serial.print("CURRENT_POS:");
    Serial.print(currentX); Serial.print(","); Serial.print(currentY);
    Serial.print(","); Serial.println(currentDir);
}

void AGVMCU::handleAbort() {
    stopMotors();
    currentState = STATE_ABORTED;
    
    // Turnaround and move forward
    rotateAngle(180);
    moveForward();
    delay(2000);
    stopMotors();
    
    Serial.println("ABORT: Waiting for QR scan...");
}

void AGVMCU::handleStop() {
    stopMotors();
    currentState = STATE_STOPPED;
}

void AGVMCU::handleStart() {
    if(currentState == STATE_STOPPED && totalSteps > 0) {
        currentState = STATE_MOVING;
        navigateToNextStep();
    }
}

void AGVMCU::handleObstacle() {
    stopMotors();
    currentState = STATE_OBSTACLE_RECOVERY;
    
    // Turnaround and move forward
    rotateAngle(180);
    moveForward();
    delay(2000);
    stopMotors();
    
    Serial.println("OBSTACLE: Waiting for QR scan...");
}

void AGVMCU::handleButtons() {
    unsigned long now = millis();

    // HIGH PRIORITY: Button checks with debouncing
    if(digitalRead(STOP_BUTTON_PIN) == LOW && (now - lastStopPress > 500)) {
        lastStopPress = now;
        handleStop();
        Serial.println("[Button] STOP pressed");
    }

    if(digitalRead(START_BUTTON_PIN) == LOW && (now - lastStartPress > 500)) {
        lastStartPress = now;
        handleStart();
        Serial.println("[Button] START pressed");
    }

    if(digitalRead(ABORT_BUTTON_PIN) == LOW && (now - lastAbortPress > 500)) {
        lastAbortPress = now;
        handleAbort();
        Serial.println("[Button] ABORT pressed");
    }
}
