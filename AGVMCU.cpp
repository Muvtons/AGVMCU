#include "AGVMCU.h"

// ============================================================================
// STATE MACHINE DEFINITIONS
// ============================================================================
enum AGVState {
    STATE_IDLE,                 // No path loaded, waiting
    STATE_WAITING_QR,          // At waypoint, need QR verification
    STATE_ADJUSTING_ORIENTATION, // Rotating to correct direction
    STATE_MOVING,              // Moving to next waypoint
    STATE_STOPPED,             // Paused by STOP command
    STATE_BLOCKED,             // Obstacle detected
    STATE_QR_MISMATCH,         // QR didn't match expected
    STATE_GOAL_REACHED         // Destination reached
};

// Global instance
AGVMCU agvmcu;

AGVMCU::AGVMCU() 
    : currentState(STATE_IDLE),
      currentStepIndex(0),
      targetStepIndex(0),
      totalSteps(0),
      isMoving(false),
      isRotating(false),
      moveStartTime(0),
      rotateStartTime(0),
      moveDurationMs(2000),
      lastQRCodeStep(-1),
      blockedStepIndex(-1) {
    // Initialize all positions to "unknown"
    currentX = currentY = expectedX = expectedY = -1;
    currentDir = expectedDir = 'E';
}

void AGVMCU::begin(long baudRate, bool initSerial) {
    if (initSerial) {
        Serial.begin(baudRate);
    }
    
    // Motor pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Button pins with pull-ups
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(ABORT_BUTTON_PIN, INPUT_PULLUP);

    stopMotors();
    
    Serial.println("\n🤖 AGV Navigation System Ready");
    Serial.println("📋 Commands: START, STOP, ABORT, QR:x,y,angle, DISTANCE:value");
    Serial.println("📍 Path Format: (x,y)dir,dist|(x,y)dir,dist|...");
    Serial.println("   Example: (0,0)E,100|(1,0)E,100|(2,0)N,50");
    Serial.println("📍 Current State: IDLE\n");
}

// ============================================================================
// MAIN UPDATE LOOP - NON-BLOCKING STATE MACHINE
// ============================================================================
void AGVMCU::update() {
    handleButtons();
    updateMovementStateMachine();
}

void AGVMCU::updateMovementStateMachine() {
    unsigned long currentTime = millis();
    
    // Handle movement completion
    if (isMoving && currentTime - moveStartTime >= moveDurationMs) {
        stopMotors();
        isMoving = false;
        
        Serial.println("⏹️ Movement completed - waiting for QR verification");
        currentState = STATE_WAITING_QR;
    }
    
    // Handle rotation completion
    if (isRotating && currentTime - rotateStartTime >= targetRotationTime) {
        stopMotors();
        isRotating = false;
        
        Serial.print("✅ Orientation adjusted - now facing ");
        Serial.println(currentDir);
        
        // After orientation, start moving
        if (currentState == STATE_ADJUSTING_ORIENTATION) {
            startMoveToNextWaypoint();
        }
    }
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================
void AGVMCU::processCommand(const char* cmd) {
    String command = String(cmd);
    command.trim();
    
    Serial.print("📥 RECEIVED: ");
    Serial.println(command);

    // ========================================
    // CONTROL COMMANDS
    // ========================================
    if (command == "ABORT") {
        handleAbort();
        return;
    }
    
    if (command == "STOP") {
        handleStop();
        return;
    }
    
    if (command == "START") {
        handleStart();
        return;
    }

    // ========================================
    // OBSTACLE DETECTION
    // ========================================
    if (command.startsWith("DISTANCE:")) {
        float distance = command.substring(9).toFloat();
        handleObstacleDetection(distance);
        return;
    }

    // ========================================
    // QR CODE PROCESSING
    // ========================================
    if (command.startsWith("QR:")) {
        processQRCode(command.substring(3));
        return;
    }

    // ========================================
    // PATH LOADING
    // ========================================
    if (command.startsWith("(")) {
        loadPath(command);
        return;
    }
    
    Serial.println("❓ Unknown command");
}

// ============================================================================
// QR CODE PROCESSING WITH VERIFICATION
// ============================================================================
void AGVMCU::processQRCode(String qrData) {
    // Parse QR: x,y,angle
    int firstComma = qrData.indexOf(',');
    int secondComma = qrData.indexOf(',', firstComma + 1);
    
    if (secondComma == -1) {
        Serial.println("❌ Invalid QR format - expected: QR:x,y,angle");
        return;
    }
    
    int qrX = qrData.substring(0, firstComma).toInt();
    int qrY = qrData.substring(firstComma + 1, secondComma).toInt();
    float qrAngle = qrData.substring(secondComma + 1).toFloat();
    
    Serial.print("\n📷 QR SCAN: Position(");
    Serial.print(qrX); Serial.print(","); Serial.print(qrY);
    Serial.print(") Angle:"); Serial.print(qrAngle); Serial.println("°");
    
    // Update current position (ground truth)
    currentX = qrX;
    currentY = qrY;
    
    // Find this QR in the loaded path
    int stepIndex = findStepIndex(qrX, qrY);
    
    // CASE 1: No path loaded - just update position
    if (totalSteps == 0) {
        Serial.println("ℹ️ Position updated. Load path to start navigation.");
        currentState = STATE_IDLE;
        publishCurrentPosition();
        return;
    }
    
    // CASE 2: QR found in path
    if (stepIndex != -1) {
        handleQRMatch(stepIndex, qrAngle);
        return;
    }
    
    // CASE 3: QR NOT in path - CRITICAL ERROR
    handleQRMismatch();
}

void AGVMCU::handleQRMatch(int stepIndex, float qrAngle) {
    lastQRCodeStep = stepIndex;
    currentStepIndex = stepIndex;
    
    Serial.print("✅ QR MATCHED: Step ");
    Serial.print(stepIndex + 1);
    Serial.print(" of ");
    Serial.println(totalSteps);
    
    // Update expected position from path
    expectedX = path[stepIndex].x;
    expectedY = path[stepIndex].y;
    expectedDir = path[stepIndex].dir;
    
    // Correct direction using QR angle
    correctDirectionUsingQRAngle(qrAngle);
    
    // Check if at final destination
    if (stepIndex >= totalSteps - 1) {
        Serial.println("\n🎉 DESTINATION REACHED - Path cleared!");
        clearPath();
        currentState = STATE_GOAL_REACHED;
        publishCurrentPosition();
        return;
    }
    
    // Set target to next waypoint
    targetStepIndex = stepIndex + 1;
    Serial.print("🎯 Next waypoint: Step ");
    Serial.println(targetStepIndex + 1);
    
    // Resume movement if stopped or just started
    if (currentState == STATE_WAITING_QR || currentState == STATE_STOPPED) {
        currentState = STATE_STOPPED; // Ready to resume
        Serial.println("▶️ Ready to continue - send START command");
    }
}

void AGVMCU::handleQRMismatch() {
    Serial.println("❌ CRITICAL: QR POSITION NOT IN LOADED PATH!");
    Serial.println("🔄 Executing 180° turn and stopping...");
    
    // Store the mismatch location for debugging
    lastQRCodeStep = -1;
    
    // Rotate 180 degrees (blocking for safety)
    rotateAngle(180);
    
    // Update direction after rotation
    switch(currentDir) {
        case 'E': currentDir = 'W'; break;
        case 'W': currentDir = 'E'; break;
        case 'N': currentDir = 'S'; break;
        case 'S': currentDir = 'N'; break;
    }
    
    currentState = STATE_QR_MISMATCH;
    stopMotors();
    
    Serial.println("⏹️ SYSTEM STOPPED - Load new path or QR");
    publishCurrentPosition();
}

// ============================================================================
// PATH LOADING AND VALIDATION
// ============================================================================
void AGVMCU::loadPath(String rawPath) {
    Serial.println("\n🗺️ LOADING NAVIGATION PATH...");
    
    parsePath(rawPath);
    
    if (totalSteps < 2) {
        Serial.println("❌ Invalid path: Need at least 2 waypoints");
        clearPath();
        return;
    }
    
    Serial.println("✅ Path loaded successfully");
    Serial.print("📍 SOURCE: Step 1 (");
    Serial.print(path[0].x); Serial.print(","); Serial.print(path[0].y);
    Serial.print(") facing "); Serial.println(path[0].dir);
    
    Serial.print("🎯 DESTINATION: Step ");
    Serial.print(totalSteps); Serial.print(" (");
    Serial.print(path[totalSteps-1].x); Serial.print(","); 
    Serial.print(path[totalSteps-1].y);
    Serial.print(") facing "); Serial.println(path[totalSteps-1].dir);
    
    // Reset navigation state
    currentStepIndex = 0;
    targetStepIndex = 0;
    currentState = STATE_WAITING_QR;
    
    Serial.println("\n⚠️  QR VERIFICATION REQUIRED before movement");
    Serial.println("📷 Scan QR at current position to proceed");
}

void AGVMCU::parsePath(String raw) {
    totalSteps = 0;
    raw.trim();
    
    int i = 0;
    while (i < raw.length() && totalSteps < maxSteps) {
        if (raw[i] == '(') {
            // Parse (x,y)
            int comma = raw.indexOf(',', i);
            int closeParen = raw.indexOf(')', comma);
            int dirComma = raw.indexOf(',', closeParen);
            int pipe = raw.indexOf('|', closeParen);
            
            if (comma == -1 || closeParen == -1) break;
            
            int x = raw.substring(i + 1, comma).toInt();
            int y = raw.substring(comma + 1, closeParen).toInt();
            
            // Parse direction and distance
            char dir = raw[closeParen + 1];
            int distance = 100; // default 100cm
            
            if (dirComma != -1 && (pipe == -1 || dirComma < pipe)) {
                distance = raw.substring(dirComma + 1).toInt();
            }
            
            if ((dir == 'N' || dir == 'S' || dir == 'E' || dir == 'W') && distance > 0) {
                path[totalSteps++] = {x, y, dir, distance};
            }
            
            i = (pipe == -1) ? raw.length() : pipe + 1;
        } else {
            i++;
        }
    }
    
    Serial.print("🗺️ Path contains ");
    Serial.print(totalSteps);
    Serial.println(" waypoints");
    
    for (int j = 0; j < totalSteps; j++) {
        Serial.print("  Step "); Serial.print(j + 1); Serial.print(": (");
        Serial.print(path[j].x); Serial.print(","); Serial.print(path[j].y);
        Serial.print(") → DIR:"); Serial.print(path[j].dir);
        Serial.print(" DIST:"); Serial.print(path[j].distanceCm);
        Serial.println("cm");
    }
}

void AGVMCU::clearPath() {
    totalSteps = 0;
    currentStepIndex = 0;
    targetStepIndex = 0;
    lastQRCodeStep = -1;
    blockedStepIndex = -1;
    Serial.println("🗑️ Path cleared from memory");
}

// ============================================================================
// NAVIGATION EXECUTION
// ============================================================================
void AGVMCU::handleStart() {
    Serial.println("\n▶️ START COMMAND RECEIVED");
    
    if (currentState == STATE_QR_MISMATCH) {
        Serial.println("❌ Cannot start: QR mismatch - load new path");
        return;
    }
    
    if (currentState == STATE_BLOCKED) {
        Serial.println("❌ Cannot start: Obstacle blocked - load new path");
        return;
    }
    
    if (currentState == STATE_IDLE) {
        if (totalSteps == 0) {
            Serial.println("❌ Cannot start: No path loaded");
        } else {
            Serial.println("⚠️  QR verification required first");
        }
        return;
    }
    
    if (currentState == STATE_STOPPED || currentState == STATE_WAITING_QR) {
        if (lastQRCodeStep == -1) {
            Serial.println("⚠️  QR verification required before movement");
            currentState = STATE_WAITING_QR;
            return;
        }
        
        if (targetStepIndex >= totalSteps) {
            Serial.println("✅ Already at destination");
            return;
        }
        
        Serial.println("🚀 Resuming navigation...");
        navigateToStep(targetStepIndex);
    } else {
        Serial.println("⚠️  Already running or invalid state");
    }
}

void AGVMCU::navigateToStep(int stepIndex) {
    if (stepIndex >= totalSteps) {
        Serial.println("🎉 DESTINATION REACHED!");
        clearPath();
        currentState = STATE_GOAL_REACHED;
        return;
    }
    
    Step target = path[stepIndex];
    
    Serial.print("\n🎯 NAVIGATING to Step ");
    Serial.print(stepIndex + 1);
    Serial.print(": (");
    Serial.print(target.x); Serial.print(","); Serial.print(target.y);
    Serial.print(") facing "); Serial.println(target.dir);
    
    // Calculate required direction
    char neededDir = calculateDirectionToTarget(currentX, currentY, target.x, target.y);
    
    Serial.print("📍 Current: ("); Serial.print(currentX); Serial.print(",");
    Serial.print(currentY); Serial.print(") facing "); Serial.println(currentDir);
    Serial.print("🧭 Need to face: "); Serial.println(neededDir);
    
    // Set movement parameters
    moveDurationMs = calculateMoveDuration(path[stepIndex].distanceCm);
    
    // Adjust orientation if needed
    if (currentDir != neededDir) {
        Serial.println("🔄 Adjusting orientation...");
        currentState = STATE_ADJUSTING_ORIENTATION;
        startRotateToDirection(neededDir);
    } else {
        Serial.println("✅ Orientation correct");
        startMoveToNextWaypoint();
    }
}

void AGVMCU::startMoveToNextWaypoint() {
    Step target = path[targetStepIndex];
    
    Serial.print("\n⏩ MOVING to (");
    Serial.print(target.x); Serial.print(","); Serial.print(target.y);
    Serial.print(") - Distance: "); Serial.print(target.distanceCm);
    Serial.println("cm");
    
    // Configure motors for forward movement
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    
    moveStartTime = millis();
    isMoving = true;
    currentState = STATE_MOVING;
}

void AGVMCU::startRotateToDirection(char targetDir) {
    int angleDiff = calculateAngleDifference(currentDir, targetDir);
    
    if (angleDiff == 0) {
        Serial.println("✅ Already facing correct direction");
        startMoveToNextWaypoint();
        return;
    }
    
    Serial.print("🔄 Rotating "); Serial.print(abs(angleDiff));
    Serial.print("° "); Serial.println(angleDiff > 0 ? "RIGHT" : "LEFT");
    
    // Configure motors for rotation
    if (angleDiff > 0) {
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
    
    // Calculate rotation time (blocking for now, but short)
    targetRotationTime = calculateRotationTime(abs(angleDiff));
    rotateStartTime = millis();
    isRotating = true;
    
    currentDir = targetDir; // Update immediately
}

// ============================================================================
// OBSTACLE HANDLING
// ============================================================================
void AGVMCU::handleObstacleDetection(float distance) {
    if (distance >= distanceThreshold) {
        return; // Path is clear
    }
    
    if (currentState != STATE_MOVING) {
        return; // Only care about obstacles during movement
    }
    
    Serial.println("\n🚨 OBSTACLE DETECTED!");
    Serial.print("📏 Distance: "); Serial.print(distance); 
    Serial.println("m (< threshold)");
    
    // Emergency stop
    stopMotors();
    
    // Record where we got blocked
    blockedStepIndex = targetStepIndex;
    
    Serial.print("🚧 BLOCKED at Step "); Serial.print(blockedStepIndex + 1);
    Serial.print(": Attempting to move to (");
    Serial.print(path[blockedStepIndex].x); Serial.print(",");
    Serial.print(path[blockedStepIndex].y); Serial.println(")");
    
    // Rotate 180 degrees (blocking, but emergency)
    Serial.println("🔄 Executing 180° turn...");
    rotateAngle(180);
    
    // Update direction after 180° turn
    switch(currentDir) {
        case 'E': currentDir = 'W'; break;
        case 'W': currentDir = 'E'; break;
        case 'N': currentDir = 'S'; break;
        case 'S': currentDir = 'N'; break;
    }
    
    // Return to last verified QR position
    if (lastQRCodeStep >= 0 && lastQRCodeStep < totalSteps) {
        currentX = path[lastQRCodeStep].x;
        currentY = path[lastQRCodeStep].y;
        currentDir = path[lastQRCodeStep].dir;
        
        Serial.print("📍 Returned to last verified position: Step ");
        Serial.print(lastQRCodeStep + 1); Serial.print(" (");
        Serial.print(currentX); Serial.print(","); Serial.print(currentY);
        Serial.print(") facing "); Serial.println(currentDir);
    }
    
    currentState = STATE_BLOCKED;
    publishCurrentPosition();
    
    Serial.println("\n⏹️ SYSTEM BLOCKED - Load new path to continue");
}

// ============================================================================
// CONTROL COMMANDS
// ============================================================================
void AGVMCU::handleAbort() {
    Serial.println("\n🛑 ABORT COMMAND - EMERGENCY STOP!");
    
    stopMotors();
    clearPath();
    currentState = STATE_IDLE;
    lastQRCodeStep = -1;
    blockedStepIndex = -1;
    
    Serial.println("⏹️ System ABORTED - All data cleared");
    publishCurrentPosition();
}

void AGVMCU::handleStop() {
    Serial.println("\n⏸️ STOP COMMAND - PAUSING");
    
    stopMotors();
    
    // Only pause if currently moving or adjusting
    if (currentState == STATE_MOVING || currentState == STATE_ADJUSTING_ORIENTATION) {
        currentState = STATE_STOPPED;
        Serial.println("⏹️ Motors stopped - Navigation PAUSED");
        Serial.println("▶️ Send START to resume from this position");
    } else {
        Serial.println("⚠️  System not in movement state");
    }
    
    publishCurrentPosition();
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================
void AGVMCU::stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    
    isMoving = false;
    isRotating = false;
}

char AGVMCU::calculateDirectionToTarget(int fromX, int fromY, int toX, int toY) {
    int dx = toX - fromX;
    int dy = toY - fromY;
    
    if (dx > 0) return 'E';
    if (dx < 0) return 'W';
    if (dy > 0) return 'N';
    if (dy < 0) return 'S';
    
    return currentDir; // Already at target
}

int AGVMCU::calculateAngleDifference(char from, char to) {
    // Returns + for right turn, - for left turn
    if (from == to) return 0;
    
    // 90 degree right turns
    if ((from == 'E' && to == 'S') || 
        (from == 'S' && to == 'W') || 
        (from == 'W' && to == 'N') || 
        (from == 'N' && to == 'E')) {
        return -90; // Left turn
    }
    
    // 90 degree left turns
    if ((from == 'E' && to == 'N') || 
        (from == 'N' && to == 'W') || 
        (from == 'W' && to == 'S') || 
        (from == 'S' && to == 'E')) {
        return 90; // Right turn
    }
    
    return 180; // Opposite direction
}

unsigned long AGVMCU::calculateRotationTime(float degrees) {
    if (degrees >= 170) return 1600;
    if (degrees >= 80) return 800;
    return (unsigned long)(degrees * 8.5);
}

unsigned long AGVMCU::calculateMoveDuration(float distanceCm) {
    // Assuming constant speed of 50cm/s
    // Adjust this factor based on your robot's actual speed
    const float speedCmPerSec = 50.0;
    return (unsigned long)((distanceCm / speedCmPerSec) * 1000.0);
}

void AGVMCU::rotateAngle(float degrees) {
    // Blocking rotation for emergency cases only
    if (degrees == 0) return;
    
    Serial.print("🔄 Rotating "); Serial.print(abs(degrees));
    Serial.println("°");
    
    if (degrees > 0) {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    } else {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    }
    
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);
    
    delay(calculateRotationTime(abs(degrees)));
    stopMotors();
    delay(500);
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
    
    if (abs(angleError) > 5.0) {
        Serial.print("🔧 Correcting direction by "); Serial.print(angleError);
        Serial.println("°");
        rotateAngle(angleError);
    } else {
        Serial.println("✅ Direction accurate (error < 5°)");
    }
}

int AGVMCU::findStepIndex(int x, int y) {
    for (int i = 0; i < totalSteps; i++) {
        if (path[i].x == x && path[i].y == y) {
            return i;
        }
    }
    return -1;
}

void AGVMCU::publishCurrentPosition() {
    Serial.print("\n📍 CURRENT POSITION: (");
    Serial.print(currentX); Serial.print(","); Serial.print(currentY);
    Serial.print(") facing "); Serial.print(currentDir);
    Serial.print(" | State: "); Serial.println(getStateName());
    
    Serial.print("CURRENT_POS:"); Serial.print(currentX);
    Serial.print(","); Serial.print(currentY);
    Serial.print(","); Serial.println(currentDir);
}

const char* AGVMCU::getStateName() {
    switch(currentState) {
        case STATE_IDLE: return "IDLE";
        case STATE_WAITING_QR: return "WAITING_QR";
        case STATE_ADJUSTING_ORIENTATION: return "ADJUSTING_ORIENTATION";
        case STATE_MOVING: return "MOVING";
        case STATE_STOPPED: return "STOPPED";
        case STATE_BLOCKED: return "BLOCKED";
        case STATE_QR_MISMATCH: return "QR_MISMATCH";
        case STATE_GOAL_REACHED: return "GOAL_REACHED";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================
void AGVMCU::handleButtons() {
    unsigned long now = millis();
    
    // STOP button
    if (digitalRead(STOP_BUTTON_PIN) == LOW) {
        if (now - lastStopPress > 500) {
            lastStopPress = now;
            Serial.println("\n⏸️ Button: STOP PRESSED");
            handleStop();
        }
    }
    
    // START button
    if (digitalRead(START_BUTTON_PIN) == LOW) {
        if (now - lastStartPress > 500) {
            lastStartPress = now;
            Serial.println("\n▶️ Button: START PRESSED");
            handleStart();
        }
    }
    
    // ABORT button
    if (digitalRead(ABORT_BUTTON_PIN) == LOW) {
        if (now - lastAbortPress > 500) {
            lastAbortPress = now;
            Serial.println("\n🛑 Button: ABORT PRESSED");
            handleAbort();
        }
    }
}
