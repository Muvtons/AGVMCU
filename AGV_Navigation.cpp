#include "AGV_Navigation.h"

AGV_Navigation AGV;

void AGV_Navigation::begin() {
    Serial.begin(115200);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(startButtonPin, INPUT_PULLUP);
    pinMode(stopButtonPin, INPUT_PULLUP);
    pinMode(abortButtonPin, INPUT_PULLUP);
    pinMode(defaultButtonPin, INPUT_PULLUP);

    stopMotors();
    Serial.println("AGV System Ready (ROS2 Integrated - Enhanced Distance Timeout).");
}

void AGV_Navigation::run() {
    handleButtons();
    handleSerialInput();

    if (abortActive && waitingForQRAfterAbort) {
        moveForward();
        delay(100);
        return;
    }

    if (distanceTimeoutActive && waitingForQRAfterAbort) {
        moveForward();
        delay(100);
        return;
    }

    if (awaitingPositionAfterNewPath) return;
    if (movementCompleted) return;
    if (!isStarted || isStopped || goalReached || currentStep >= totalSteps || currentX == -1) return;

    Step target = steps[currentStep];

    if (currentX == target.x && currentY == target.y) {
        if (currentDir != target.dir) {
            stopMotors();
            rotateToDirection(currentDir, target.dir);
            currentDir = target.dir;
        }

        if (currentStep == totalSteps - 1) {
            Serial.print("✅ Goal reached: ");
            Serial.print(currentX); Serial.print(","); Serial.println(currentY);
            goalReached = true;
            stopMotors();
            
            currentX = -1; currentY = -1;
            totalSteps = 0; currentStep = 0;
            goalReached = false;
            isStarted = false;
            return;
        }

        Step nextTarget = steps[currentStep + 1];
        Serial.print("Step "); Serial.print(currentStep);
        Serial.print(": Moving from ("); Serial.print(currentX);
        Serial.print(","); Serial.print(currentY);
        Serial.print(") toward ("); Serial.print(nextTarget.x);
        Serial.print(","); Serial.print(nextTarget.y);
        Serial.print(") facing "); Serial.println(target.dir);

        moveOneCell();
        movementCompleted = true;
    }
}

void AGV_Navigation::publishCurrentPosition() {
    Serial.print("CURRENT_POS:");
    Serial.print(currentX); Serial.print(","); Serial.print(currentY);
    Serial.print(","); Serial.println(currentDir);
}

void AGV_Navigation::triggerRecoveryProcedure(const char* source) {
    Serial.print("🛑 Recovery triggered by: ");
    Serial.println(source);

    stopMotors();
    Serial.println("⛔ Motors stopped.");

    Serial.println("🔄 Turning around 180°...");
    rotateAngle(180);
    Serial.println("✅ Turnaround complete.");

    Serial.println("🚶 Moving forward until next QR is detected...");
    waitingForQRAfterAbort = true;

    isStarted = false;
    isStopped = true;
    awaitingPositionAfterNewPath = false;
    movementCompleted = false;
    distanceBelowThreshold = false;
    goalReached = false;
    totalSteps = 0;
    currentStep = 0;
}

void AGV_Navigation::handleDistanceTimeout() {
    Serial.println("🛑 DISTANCE TIMEOUT - Triggering recovery procedure!");
    distanceTimeoutActive = true;
    triggerRecoveryProcedure("Distance Timeout");
    
    Serial.print("DISTANCE_TIMEOUT_POS:");
    Serial.print(currentX); Serial.print(","); Serial.println(currentY);
}

void AGV_Navigation::checkDistanceCondition(float currentDistance) {
    unsigned long currentTime = millis();
    if (currentDistance < DISTANCE_THRESHOLD) {
        if (!distanceBelowThreshold) {
            distanceBelowThreshold = true;
            distanceBelowThresholdStart = currentTime;
            Serial.print("📏 Distance below threshold: ");
            Serial.print(currentDistance); Serial.println("m - Starting timer...");
        } else if (currentTime - distanceBelowThresholdStart >= DISTANCE_TIMEOUT_MS) {
            handleDistanceTimeout();
        }
    } else if (distanceBelowThreshold) {
        distanceBelowThreshold = false;
        Serial.print("✅ Distance back to normal: ");
        Serial.print(currentDistance); Serial.println("m");
    }
}

void AGV_Navigation::moveOneCell() {
    Serial.println("🚗 Moving one cell forward...");
    moveForward();
    delay(2000);
    stopMotors();
    Serial.println("🛑 Movement complete, waiting for QR confirmation...");
}

void AGV_Navigation::rotateAngle(float degrees) {
    Serial.print("🔄 Rotating "); Serial.print(degrees); Serial.println(" degrees...");
    if (degrees > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENA, 200);
        analogWrite(ENB, 200);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, 200);
        analogWrite(ENB, 200);
    }

    unsigned long rotateTime = 0;
    if (abs(degrees) >= 170) rotateTime = 1600;
    else if (abs(degrees) >= 80) rotateTime = 800;
    else rotateTime = (unsigned long)(abs(degrees) * 8.5);
    delay(rotateTime);
    stopMotors();
    Serial.println("✅ Rotation complete.");
}

void AGV_Navigation::correctDirectionUsingQRAngle(float qrAngle) {
    float targetAngle = 0.0;
    switch(currentDir) {
        case 'E': targetAngle = 0.0; break;
        case 'N': targetAngle = 90.0; break;
        case 'W': targetAngle = 180.0; break;
        case 'S': targetAngle = -90.0; break;
    }
    
    float angleError = targetAngle - qrAngle;
    while (angleError > 180) angleError -= 360;
    while (angleError < -180) angleError += 360;
    
    Serial.print("🎯 QR Reference - Current: "); Serial.print(qrAngle);
    Serial.print("°, Target: "); Serial.print(targetAngle);
    Serial.print("°, Error: "); Serial.print(angleError); Serial.println("°");
    
    if (abs(angleError) > 2.0) {
        Serial.print("🔧 Correcting by: "); Serial.print(angleError); Serial.println("°");
        rotateAngle(angleError);
    } else {
        Serial.println("✅ Perfectly aligned with target direction");
    }
}

void AGV_Navigation::rotateToDirection(char from, char to) {
    int angleDiff = 0;
    if ((from == 'E' && to == 'N') || (from == 'N' && to == 'W') || 
        (from == 'W' && to == 'S') || (from == 'S' && to == 'E')) {
        angleDiff = 90;
    } else if ((from == 'E' && to == 'S') || (from == 'S' && to == 'W') || 
               (from == 'W' && to == 'N') || (from == 'N' && to == 'E')) {
        angleDiff = -90;
    } else if (from != to) {
        angleDiff = 180;
    }
    if (angleDiff != 0) {
        rotateAngle(angleDiff);
    }
}

void AGV_Navigation::handleSerialInput() {
    if (Serial.available()) {
        inputBuffer = Serial.readStringUntil('\n');
        inputBuffer.trim();

        if (inputBuffer == "ABORT") {
            handleAbort();
            return;
        }

        if (inputBuffer.startsWith("DISTANCE:")) {
            String distanceStr = inputBuffer.substring(9);
            float currentDistance = distanceStr.toFloat();
            checkDistanceCondition(currentDistance);
            return;
        }

        if (inputBuffer.startsWith("(")) {
            parsePath(inputBuffer);
            if (totalSteps > 0) {
                goalReached = false;
                isStopped = false;
                currentStep = 0;
                movementCompleted = false;

                if (currentX == -1 || currentY == -1) {
                    awaitingPositionAfterNewPath = true;
                    isStarted = false;
                    Serial.print("✅ New path loaded. Steps: ");
                    Serial.println(totalSteps);
                    Serial.println("⏳ Awaiting initial QR position...");
                } else {
                    awaitingPositionAfterNewPath = false;
                    isStarted = true;
                    Serial.print("✅ New path loaded and position known (");
                    Serial.print(currentX); Serial.print(","); Serial.print(currentY);
                    Serial.println("). Starting immediately.");
                }

                Serial.print("📋 Received Path: ");
                for (int i = 0; i < totalSteps; i++) {
                    Serial.print("("); Serial.print(steps[i].x); Serial.print(",");
                    Serial.print(steps[i].y); Serial.print(")"); Serial.print(steps[i].dir);
                    if (i < totalSteps - 1) Serial.print("->");
                }
                Serial.println();
            } else {
                Serial.println("❌ Path parsing failed.");
            }
            return;
        }

        if (inputBuffer.startsWith("QR:")) {
            String coordData = inputBuffer.substring(3);
            int firstComma = coordData.indexOf(',');
            int secondComma = coordData.indexOf(',', firstComma + 1);
            
            if (secondComma != -1) {
                int qrX = coordData.substring(0, firstComma).toInt();
                int qrY = coordData.substring(firstComma + 1, secondComma).toInt();
                float qrAngle = coordData.substring(secondComma + 1).toFloat();

                currentX = qrX;
                currentY = qrY;
                lastValidX = qrX;
                lastValidY = qrY;
                
                Serial.print("📍 QR Position: "); 
                Serial.print(currentX); Serial.print(","); Serial.print(currentY);
                Serial.print(" Angle: "); Serial.println(qrAngle);

                correctDirectionUsingQRAngle(qrAngle);

                if (abortActive && waitingForQRAfterAbort) {
                    Serial.println("✅ QR detected after abort recovery!");
                    stopMotors();
                    abortActive = false;
                    waitingForQRAfterAbort = false;
                    Serial.println("🟢 Abort recovery complete. Awaiting new path...");
                    return;
                }

                if (distanceTimeoutActive && waitingForQRAfterAbort) {
                    Serial.println("✅ QR detected after distance timeout recovery!");
                    stopMotors();
                    distanceTimeoutActive = false;
                    waitingForQRAfterAbort = false;
                    Serial.println("🟢 Distance timeout recovery complete. Awaiting new path...");
                    return;
                }

                if (awaitingPositionAfterNewPath && totalSteps > 0) {
                    awaitingPositionAfterNewPath = false;
                    isStarted = true;
                    Serial.println("✅ Initial QR received. Starting navigation.");
                }

                if (movementCompleted && currentStep < totalSteps - 1) {
                    Step nextTarget = steps[currentStep + 1];
                    if (currentX == nextTarget.x && currentY == nextTarget.y) {
                        Serial.println("✅ QR confirmed arrival at next step.");
                        currentStep++;
                        movementCompleted = false;
                        publishCurrentPosition();
                    } else {
                        Serial.print("⚠ QR ("); Serial.print(currentX);
                        Serial.print(","); Serial.print(currentY);
                        Serial.print(") does not match expected (");
                        Serial.print(nextTarget.x); Serial.print(","); Serial.print(nextTarget.y);
                        Serial.println("). Ignoring.");
                    }
                }

                lastPositionInput = inputBuffer;
            }
            return;
        }

        if (inputBuffer.length() > 0) {
            Serial.print("❓ Unknown command: ");
            Serial.println(inputBuffer);
        }
    }
}

void AGV_Navigation::parsePath(String raw) {
    totalSteps = 0;
    raw.trim();
    int i = 0;
    while (i < raw.length() && totalSteps < maxSteps) {
        if (raw[i] == '(') {
            int commaIndex = raw.indexOf(',', i);
            int endIndex = raw.indexOf(')', commaIndex);
            if (commaIndex == -1 || endIndex == -1) break;
            int x = raw.substring(i + 1, commaIndex).toInt();
            int y = raw.substring(commaIndex + 1, endIndex).toInt();
            char dir = 'E';
            if (endIndex + 1 < raw.length()) {
                dir = raw[endIndex + 1];
            }
            if (dir == 'N' || dir == 'S' || dir == 'E' || dir == 'W') {
                steps[totalSteps++] = {x, y, dir};
            }
            i = endIndex + 2;
        } else {
            i++;
        }
    }
}

void AGV_Navigation::handleAbort() {
    Serial.println("🛑 ABORT triggered!");
    abortActive = true;
    triggerRecoveryProcedure("Manual Abort");
}

void AGV_Navigation::handleButtons() {
    static unsigned long lastAbortPress = 0;
    unsigned long now = millis();

    if (digitalRead(stopButtonPin) == LOW && !isStopped) {
        isStopped = true;
        stopMotors();
        Serial.println("⏹️ STOP pressed.");
        delay(300);
    }

    if (digitalRead(startButtonPin) == LOW && !isStarted && !awaitingPositionAfterNewPath) {
        isStarted = true;
        isStopped = false;
        Serial.println("▶️ START pressed.");
        delay(300);
    }

    if (digitalRead(abortButtonPin) == LOW) {
        if (now - lastAbortPress > 500) {
            lastAbortPress = now;
            handleAbort();
        }
    }

    if (digitalRead(defaultButtonPin) == LOW) {
        Serial.println("🟩 DEFAULT button pressed.");
        delay(300);

        String defaultPath = "(1,1)E(2,1)E(2,2)N(3,2)E";
        parsePath(defaultPath);

        if (totalSteps > 0) {
            awaitingPositionAfterNewPath = (currentX == -1);
            isStarted = !awaitingPositionAfterNewPath;
            currentStep = 0;
            movementCompleted = false;
            goalReached = false;
            Serial.println("✅ Default path loaded.");
            if (isStarted) Serial.println("▶️ Starting default path immediately.");
            else Serial.println("⏳ Waiting for initial QR before starting.");
        } else {
            Serial.println("❌ Failed to load default path.");
        }
    }
}

void AGV_Navigation::moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
}

void AGV_Navigation::stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}
