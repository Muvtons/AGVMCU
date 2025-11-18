#include "AGVMCU.h"

// Global instance
AGVMCU agvmcu;

AGVMCU::AGVMCU() 
    : distanceThreshold(10.0), distanceTimeoutMs(10000), 
      distanceBelowThreshold(false), isInObstacleRecovery(false),
      totalSteps(0), currentStepIndex(0), currentX(-1), currentY(-1), 
      currentDir('E'), currentState(STATE_IDLE),
      lastSafeX(-1), lastSafeY(-1), lastSafeDir('E'),
      blockedTargetX(-1), blockedTargetY(-1),
      originalDestinationX(-1), originalDestinationY(-1),
      lastAbortPress(0), lastStartPress(0), lastStopPress(0),
      distanceBelowThresholdStart(0),
      moveStartTime(0), isMoving(false), rotateStartTime(0), 
      isRotating(false), targetRotationTime(0), qrWaitStartTime(0) {
}

void AGVMCU::begin(long baudRate) {
    Serial.begin(baudRate);
    
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(ABORT_BUTTON_PIN, INPUT_PULLUP);

    stopMotors();
    
    Serial.println("🚀 AGV System Ready - AGVMCU Library");
    Serial.println("📋 Commands: START, STOP, ABORT, QR:x,y,angle, DISTANCE:value, Path:(x,y)dir");
    Serial.println("📍 Current State: IDLE");
}

void AGVMCU::update() {
    handleButtons();
    updateMovementStateMachine();
}

void AGVMCU::updateMovementStateMachine() {
    unsigned long currentTime = millis();
    
    // Handle non-blocking forward movement
    if (isMoving && currentTime - moveStartTime >= 2000) {
        stopMotors();
        isMoving = false;
        
        // Update position based on direction (dead reckoning)
        updatePositionAfterMove();
        
        Serial.println("⏹️ Movement completed");
        
        // Transition to waiting for QR confirmation
        if (currentState == STATE_MOVING) {
            Serial.print("📍 Movement complete. Awaiting QR verification at (");
            Serial.print(path[currentStepIndex].x); Serial.print(","); 
            Serial.print(path[currentStepIndex].y); Serial.println(")");
            
            currentState = STATE_WAITING_QR_CONFIRMATION;
            qrWaitStartTime = currentTime;
        }
    }
    
    // Handle non-blocking rotation
    if (isRotating && currentTime - rotateStartTime >= targetRotationTime) {
        stopMotors();
        isRotating = false;
        delay(500); // Small stabilization delay after rotation
        Serial.println("✅ Rotation completed");
    }
    
    // Handle QR wait timeout
    if (currentState == STATE_WAITING_QR_CONFIRMATION && 
        currentTime - qrWaitStartTime >= 10000) { // 10 second timeout
        
        Serial.println("⚠️ QR scan timeout - Using dead reckoning position");
        Step targetStep = path[currentStepIndex];
        
        // Trust our dead reckoning
        if (isAtPosition(targetStep.x, targetStep.y)) {
            Serial.println("✅ Position confirmed by dead reckoning");
            currentStepIndex++;
            
            if (currentStepIndex >= totalSteps) {
                Serial.println("🎉 FINAL GOAL REACHED!");
                currentState = STATE_GOAL_REACHED;
                publishCurrentPosition();
            } else {
                Serial.println("🎯 Proceeding to next step...");
                currentState = STATE_MOVING;
                navigateToNextStep();
            }
        } else {
            Serial.println("❌ Position mismatch - Stopping for safety");
            currentState = STATE_STOPPED;
            stopMotors();
        }
    }
}

void AGVMCU::updatePositionAfterMove() {
    Serial.print("📍 Updating position from (");
    Serial.print(currentX); Serial.print(","); Serial.print(currentY);
    Serial.print(") facing "); Serial.print(currentDir);
    
    switch(currentDir) {
        case 'E': currentX++; break;
        case 'W': currentX--; break;
        case 'N': currentY++; break;
        case 'S': currentY--; break;
    }
    
    Serial.print(" to (");
    Serial.print(currentX); Serial.print(","); Serial.print(currentY);
    Serial.println(") [Dead Reckoning]");
}

void AGVMCU::processCommand(const char* cmd) {
    inputBuffer = String(cmd);
    inputBuffer.trim();

    Serial.print("📥 CORE0 AGVMCU RECEIVED: "); Serial.println(inputBuffer);

    // Handle control commands
    if (inputBuffer == "ABORT") {
        Serial.println("📋 Command: ABORT");
        handleAbort();
        return;
    }
    if (inputBuffer == "STOP") {
        Serial.println("📋 Command: STOP");
        handleStop();
        return;
    }
    if (inputBuffer == "START") {
        Serial.println("📋 Command: START");
        handleStart();
        return;
    }

    // Handle distance sensor
    if (inputBuffer.startsWith("DISTANCE:")) {
        String distanceStr = inputBuffer.substring(9);
        float currentDistance = distanceStr.toFloat();
        Serial.print("🔍 Distance sensor: "); Serial.print(currentDistance); Serial.println("m");
        checkDistanceCondition(currentDistance);
        return;
    }

    // === PATH LOADING ===
    if (inputBuffer.startsWith("(")) {
        Serial.println("🗺️  Loading NEW navigation path...");
        
        // Reset all navigation state
        totalSteps = 0;
        currentStepIndex = 0;
        isInObstacleRecovery = false;
        
        // Load the path
        parsePath(inputBuffer);
        
        if (totalSteps > 0) {
            Serial.println("✅ Path loaded successfully.");
            Serial.print("📍 Required START position: (");
            Serial.print(path[0].x); Serial.print(","); Serial.print(path[0].y);
            Serial.print(") facing "); Serial.println(path[0].dir);
            
            if (currentX == -1 || currentY == -1) {
                Serial.println("⚠️  Current position unknown. Please scan QR at start position.");
            } else {
                Serial.print("📍 Current position: (");
                Serial.print(currentX); Serial.print(","); Serial.print(currentY);
                Serial.print(")"); Serial.println(currentDir);
                if (currentX == path[0].x && currentY == path[0].y) {
                    Serial.println("✅ Robot is already at start position.");
                } else {
                    Serial.println("⚠️  Robot is NOT at start position. Move to start and scan QR.");
                }
            }
            
            // Wait for QR verification of start position
            currentState = STATE_WAITING_FOR_QR;
        } else {
            Serial.println("❌ Failed to load valid path");
        }
        return;
    }

    // === QR CODE PROCESSING ===
    if (inputBuffer.startsWith("QR:")) {
        String coordData = inputBuffer.substring(3);
        int firstComma = coordData.indexOf(',');
        int secondComma = coordData.indexOf(',', firstComma + 1);
        
        if (secondComma == -1) {
            Serial.println("❌ Invalid QR format");
            return;
        }
        
        int qrX = coordData.substring(0, firstComma).toInt();
        int qrY = coordData.substring(firstComma + 1, secondComma).toInt();
        float qrAngle = coordData.substring(secondComma + 1).toFloat();

        Serial.print("📷 QR SCAN: Position ("); 
        Serial.print(qrX); Serial.print(","); Serial.print(qrY);
        Serial.print("), Angle: "); Serial.print(qrAngle); Serial.println("°");

        // Update current position from QR (this is ground truth)
        currentX = qrX;
        currentY = qrY;
        correctDirectionUsingQRAngle(qrAngle);

        // Handle QR based on current state
        switch(currentState) {
            case STATE_WAITING_FOR_QR: {
                // This is the INITIAL scan to verify start position
                if (totalSteps == 0) {
                    Serial.println("❌ No path loaded yet. Path must be loaded before navigation.");
                    return;
                }
                
                Serial.print("🔍 Verifying START position: Expected (");
                Serial.print(path[0].x); Serial.print(","); Serial.print(path[0].y);
                Serial.print("), Got ("); Serial.print(currentX); Serial.print(",");
                Serial.print(currentY); Serial.println(")");
                
                if (isAtPosition(path[0].x, path[0].y)) {
                    Serial.println("✅ START position verified!");
                    currentStepIndex = 1; // Ready to move to first real step
                    
                    currentState = STATE_MOVING;
                    Serial.println("🚀 Starting navigation...");
                    navigateToNextStep();
                } else {
                    Serial.println("❌ NOT at start position! Move robot to (");
                    Serial.print(path[0].x); Serial.print(","); Serial.print(path[0].y); Serial.println(")");
                    // Stay in STATE_WAITING_FOR_QR
                }
                break;
            }
            
            case STATE_WAITING_QR_CONFIRMATION: {
                // This scan verifies we reached our target step
                if (currentStepIndex >= totalSteps) {
                    Serial.println("❌ Navigation error: Step index out of bounds");
                    currentState = STATE_STOPPED;
                    return;
                }
                
                Step targetStep = path[currentStepIndex];
                
                Serial.print("🔍 Verifying step "); Serial.print(currentStepIndex + 1);
                Serial.print(": Expected ("); Serial.print(targetStep.x); Serial.print(",");
                Serial.print(targetStep.y); Serial.print("), Got ("); Serial.print(currentX);
                Serial.print(","); Serial.print(currentY); Serial.println(")");
                
                if (isAtPosition(targetStep.x, targetStep.y)) {
                    Serial.println("✅ Step verified by QR!");
                    
                    // Move to next step
                    currentStepIndex++;
                    
                    if (currentStepIndex >= totalSteps) {
                        Serial.println("🎉 FINAL GOAL REACHED!");
                        currentState = STATE_GOAL_REACHED;
                        publishCurrentPosition();
                    } else {
                        Serial.println("🎯 Proceeding to next step...");
                        currentState = STATE_MOVING;
                        navigateToNextStep();
                    }
                } else {
                    Serial.println("❌ QR MISMATCH!");
                    Serial.println("⚠️  Navigation failed - System stopped");
                    currentState = STATE_STOPPED;
                    stopMotors();
                }
                break;
            }
            
            case STATE_OBSTACLE_AVOIDANCE: {
                Serial.println("✅ QR found during obstacle recovery!");
                if (originalDestinationX != -1 && originalDestinationY != -1) {
                    publishRerouteCommand(currentX, currentY, originalDestinationX, originalDestinationY);
                }
                currentState = STATE_IDLE;
                isInObstacleRecovery = false;
                Serial.println("🔄 Recovery complete - Awaiting new path");
                break;
            }
            
            default: {
                // QR scanned in unexpected state - just update position
                Serial.println("ℹ️  Position updated from QR");
                publishCurrentPosition();
                break;
            }
        }
        return;
    }
    
    Serial.println("❓ Unknown command");
}

void AGVMCU::navigateToNextStep() {
    if (currentStepIndex >= totalSteps) {
        Serial.println("🎉 GOAL REACHED!");
        currentState = STATE_GOAL_REACHED;
        return;
    }

    Step targetStep = path[currentStepIndex];
    
    Serial.print("🎯 MOVING to step "); Serial.print(currentStepIndex + 1);
    Serial.print(" of "); Serial.print(totalSteps);
    Serial.print(": ("); Serial.print(targetStep.x); Serial.print(",");
    Serial.print(targetStep.y); Serial.print(") facing "); Serial.println(targetStep.dir);

    // Check and adjust direction
    if (currentDir != targetStep.dir) {
        Serial.println("🔄 Adjusting direction...");
        rotateToDirection(currentDir, targetStep.dir);
    }

    // Execute non-blocking movement
    Serial.println("⏩ Starting forward movement...");
    currentState = STATE_MOVING;
    startMoveForward();
}

// Non-blocking motor control functions
void AGVMCU::startMoveForward() {
    Serial.print("⏩ MOVING FORWARD from ("); 
    Serial.print(currentX); Serial.print(","); Serial.print(currentY);
    Serial.print(") facing "); Serial.print(currentDir);
    Serial.println(" for 2 seconds (1 grid cell)");
    
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    
    moveStartTime = millis();
    isMoving = true;
}

void AGVMCU::startMoveBackward() {
    Serial.print("⏪ MOVING BACKWARD from ("); 
    Serial.print(currentX); Serial.print(","); Serial.print(currentY);
    Serial.print(") facing "); Serial.print(currentDir);
    Serial.println(" for 2 seconds (1 grid cell)");
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    
    moveStartTime = millis();
    isMoving = true;
}

void AGVMCU::moveBackward() {
    // Blocking version for obstacle recovery
    Serial.print("⏪ MOVING BACKWARD from ("); 
    Serial.print(currentX); Serial.print(","); Serial.print(currentY);
    Serial.print(") facing "); Serial.print(currentDir);
    Serial.println(" for 2 seconds (1 grid cell)");
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    
    delay(2000);
    stopMotors();
    
    // Update position for backward movement
    switch(currentDir) {
        case 'E': currentX--; break;
        case 'W': currentX++; break;
        case 'N': currentY--; break;
        case 'S': currentY++; break;
    }
    
    Serial.println("⏹️  Backward movement completed");
    Serial.print("📍 New position: (");
    Serial.print(currentX); Serial.print(","); Serial.print(currentY); Serial.println(")");
}

void AGVMCU::stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    
    // Clear movement flags
    isMoving = false;
    isRotating = false;
}

void AGVMCU::rotateAngle(float degrees) {
    Serial.print("🔄 ROTATING "); 
    Serial.print(degrees); Serial.print("° from direction ");
    Serial.print(currentDir);
    
    if (degrees > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        Serial.print(" (RIGHT turn)");
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        Serial.print(" (LEFT turn)");
    }
    
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);

    unsigned long rotateTime = 0;
    if (abs(degrees) >= 170) rotateTime = 1600;
    else if (abs(degrees) >= 80) rotateTime = 800;
    else rotateTime = (unsigned long)(abs(degrees) * 8.5);
    
    Serial.print(" for "); Serial.print(rotateTime); Serial.println("ms");
    
    delay(rotateTime); // Keep blocking for rotation (quick operation)
    stopMotors();
    delay(500);
    
    Serial.println("✅ Rotation completed");
}

void AGVMCU::rotateToDirection(char from, char to) {
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
        Serial.print("🔄 CHANGING DIRECTION: ");
        Serial.print(from); Serial.print(" -> "); Serial.print(to);
        Serial.print(" ("); Serial.print(angleDiff); Serial.println("°)");
        
        rotateAngle(angleDiff);
        currentDir = to;
        
        Serial.print("📍 Now facing: "); Serial.println(currentDir);
    } else {
        Serial.print("✅ Already facing "); Serial.println(to);
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
    while (angleError > 180) angleError -= 360;
    while (angleError < -180) angleError += 360;
    
    Serial.print("🔍 DIRECTION CORRECTION: Target="); Serial.print(targetAngle);
    Serial.print("°, QR="); Serial.print(qrAngle); Serial.print("°, Error=");
    Serial.print(angleError); Serial.println("°");
    
    if (abs(angleError) > 5.0) {
        Serial.print("🔧 Correcting by "); Serial.print(angleError); Serial.println("°");
        rotateAngle(angleError);
    } else {
        Serial.println("✅ Direction is accurate");
    }
}

bool AGVMCU::isAtPosition(int x, int y) {
    return (currentX == x && currentY == y);
}

void AGVMCU::parsePath(String raw) {
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
                path[totalSteps++] = {x, y, dir};
            }
            i = endIndex + 2;
        } else {
            i++;
        }
    }
    
    Serial.print("🗺️  PATH LOADED: ");
    Serial.print(totalSteps); Serial.println(" steps");
    for (int j = 0; j < totalSteps; j++) {
        Serial.print("  Step "); Serial.print(j + 1); Serial.print(": (");
        Serial.print(path[j].x); Serial.print(","); Serial.print(path[j].y);
        Serial.print(") -> "); Serial.println(path[j].dir);
    }
}

void AGVMCU::publishCurrentPosition() {
    Serial.print("📍 CURRENT POSITION: (");
    Serial.print(currentX); Serial.print(","); Serial.print(currentY);
    Serial.print(") facing "); Serial.println(currentDir);
    
    Serial.print("CURRENT_POS:");
    Serial.print(currentX); Serial.print(","); Serial.print(currentY);
    Serial.print(","); Serial.println(currentDir);
}

void AGVMCU::publishRerouteCommand(int src_x, int src_y, int dst_x, int dst_y) {
    Serial.println("🔄 REQUESTING REROUTE:");
    Serial.print("  FROM: ("); Serial.print(src_x); Serial.print(",");
    Serial.print(src_y); Serial.println(")");
    Serial.print("  TO: ("); Serial.print(dst_x); Serial.print(",");
    Serial.print(dst_y); Serial.println(")");
    
    Serial.print("REROUTE:SRC:(");
    Serial.print(src_x); Serial.print(","); Serial.print(src_y);
    Serial.print("):DST:(");
    Serial.print(dst_x); Serial.print(","); Serial.print(dst_y);
    Serial.println("):ONCE");
}

void AGVMCU::handleAbort() {
    Serial.println("🛑 ABORT COMMAND - EMERGENCY STOP!");
    stopMotors();
    currentState = STATE_ABORTED;
    isInObstacleRecovery = false;
    publishCurrentPosition();
    totalSteps = 0;
    currentStepIndex = 0;
    Serial.println("⏹️  System in ABORTED state");
}

void AGVMCU::handleStop() {
    Serial.println("⏸️  STOP COMMAND - PAUSING NAVIGATION");
    stopMotors();
    
    // If we were moving, update state appropriately
    if (currentState == STATE_MOVING) {
        if (isMoving) {
            Serial.println("⚠️  Stopped during movement - position may be uncertain");
        }
    }
    
    currentState = STATE_STOPPED;
    Serial.println("⏹️  System in STOPPED state");
}

void AGVMCU::handleStart() {
    Serial.println("▶️  START COMMAND RECEIVED");
    if (currentState == STATE_STOPPED || currentState == STATE_ABORTED) {
        if (totalSteps > 0 && currentX != -1 && currentY != -1) {
            // Determine where to resume
            if (currentStepIndex >= totalSteps) {
                Serial.println("❌ Navigation already completed");
                return;
            }
            
            Serial.println("✅ Resuming navigation...");
            currentState = STATE_MOVING;
            navigateToNextStep();
        } else {
            Serial.println("❌ Cannot start: No path loaded or position unknown");
            Serial.println("ℹ️  Load path first, scan QR at start position");
        }
    } else {
        Serial.println("⚠️  Already running or invalid state for START");
    }
}

void AGVMCU::handleButtons() {
    unsigned long now = millis();

    if (digitalRead(STOP_BUTTON_PIN) == LOW) {
        if (now - lastStopPress > 500) {
            lastStopPress = now;
            Serial.println("⏸️  Button: STOP PRESSED");
            handleStop();
        }
    }

    if (digitalRead(START_BUTTON_PIN) == LOW) {
        if (now - lastStartPress > 500) {
            lastStartPress = now;
            Serial.println("▶️  Button: START PRESSED");
            handleStart();
        }
    }

    if (digitalRead(ABORT_BUTTON_PIN) == LOW) {
        if (now - lastAbortPress > 500) {
            lastAbortPress = now;
            Serial.println("🛑 Button: ABORT PRESSED");
            handleAbort();
        }
    }
}

void AGVMCU::checkDistanceCondition(float currentDistance) {
    unsigned long currentTime = millis();
    
    Serial.print("🔍 Distance check: "); Serial.print(currentDistance); 
    Serial.print("m (threshold: "); Serial.print(distanceThreshold); Serial.println("m)");
    
    if (currentDistance < distanceThreshold) {
        if (!distanceBelowThreshold) {
            distanceBelowThreshold = true;
            distanceBelowThresholdStart = currentTime;
            
            Serial.print("⚠️  OBSTACLE DETECTED: "); 
            Serial.print(currentDistance); Serial.println("m - Emergency stop!");
            
            if (currentState == STATE_MOVING) {
                stopMotors();
                currentState = STATE_STOPPED;
                Serial.println("🛑 EMERGENCY STOP - Obstacle ahead!");
            }
        } 
        else if (currentTime - distanceBelowThresholdStart >= distanceTimeoutMs) {
            Serial.print("⏰ OBSTACLE TIMEOUT: Still detected after ");
            Serial.print(distanceTimeoutMs/1000); Serial.println(" seconds");
            handleObstacleTimeout();
        }
    } 
    else if (distanceBelowThreshold) {
        distanceBelowThreshold = false;
        Serial.print("✅ Path clear: "); 
        Serial.print(currentDistance); Serial.println("m");
    }
}

void AGVMCU::handleObstacleTimeout() {
    Serial.println("🚨 PERMANENT OBSTACLE - Starting recovery procedure...");
    
    if (currentX != -1 && currentY != -1) {
        lastSafeX = currentX;
        lastSafeY = currentY;
        lastSafeDir = currentDir;
        Serial.print("📍 Safe position stored: (");
        Serial.print(lastSafeX); Serial.print(","); Serial.print(lastSafeY);
        Serial.print(") facing "); Serial.println(lastSafeDir);
    }
    
    if (currentStepIndex < totalSteps) {
        blockedTargetX = path[currentStepIndex].x;
        blockedTargetY = path[currentStepIndex].y;
        
        if (totalSteps > 0) {
            originalDestinationX = path[totalSteps - 1].x;
            originalDestinationY = path[totalSteps - 1].y;
        }
        
        Serial.print("🎯 Blocked target: ("); 
        Serial.print(blockedTargetX); Serial.print(","); Serial.print(blockedTargetY); 
        Serial.println(")");
        Serial.print("🏁 Original destination: ("); 
        Serial.print(originalDestinationX); Serial.print(","); Serial.print(originalDestinationY); 
        Serial.println(")");
    }
    
    startObstacleRecovery();
}

void AGVMCU::startObstacleRecovery() {
    Serial.println("🔄 OBSTACLE RECOVERY INITIATED:");
    Serial.println("  1. Turn around 180°");
    Serial.println("  2. Move back one cell");
    Serial.println("  3. Request new path");
    
    currentState = STATE_OBSTACLE_AVOIDANCE;
    isInObstacleRecovery = true;
    
    // Turn 180 degrees
    Serial.println("🔄 Turning 180° away from obstacle...");
    rotateAngle(180);
    
    // Update direction for backward movement
    switch(currentDir) {
        case 'E': currentDir = 'W'; break;
        case 'W': currentDir = 'E'; break;
        case 'N': currentDir = 'S'; break;
        case 'S': currentDir = 'N'; break;
    }
    
    Serial.print("🔄 Now facing: "); Serial.println(currentDir);
    
    // Move back one cell to safe position (blocking OK for recovery)
    Serial.println("⏪ Moving backward to safe position...");
    moveBackward();
    
    // Request reroute from ROS2
    if (currentX != -1 && currentY != -1 && originalDestinationX != -1 && originalDestinationY != -1) {
        Serial.println("📡 Publishing re-route request to ROS2...");
        publishRerouteCommand(currentX, currentY, originalDestinationX, originalDestinationY);
    }
    
    Serial.println("🔄 Obstacle recovery complete - Awaiting new path");
    currentState = STATE_IDLE; // Wait for new path
}
