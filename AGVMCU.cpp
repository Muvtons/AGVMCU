#include "AGVMCU.h"
#include <string.h>

AGVMCU agvmcu;

volatile long encL = 0;
volatile long encR = 0;
void IRAM_ATTR isrLeft()  { encL++; }
void IRAM_ATTR isrRight() { encR++; }

AGVMCU::AGVMCU() 
    : distanceThreshold(10.0), totalSteps(0), currentStepIndex(0), 
      currentX(-1), currentY(-1), currentDir('E'), targetDirPending('E'),
      currentState(STATE_IDLE), obstacleTimerStart(0), 
      pulsesTraveledBeforeStop(0), lastButtonPressTime(0) {}

void AGVMCU::begin() {
    // 1. Start Serial
    Serial.begin(115200);
    delay(100); // Short delay to let power stabilize

    // 2. Config Outputs (Motor Driver)
    pinMode(DIR_L, OUTPUT); digitalWrite(DIR_L, LOW);
    pinMode(DIR_R, OUTPUT); digitalWrite(DIR_R, LOW);
    pinMode(BRK_L, OUTPUT); digitalWrite(BRK_L, HIGH); // Force Brakes ON
    pinMode(BRK_R, OUTPUT); digitalWrite(BRK_R, HIGH); 
    pinMode(PWM_L, OUTPUT); analogWrite(PWM_L, 0);     // Ensure 0 Speed
    pinMode(PWM_R, OUTPUT); analogWrite(PWM_R, 0);

    // 3. Config Inputs (Encoders)
    pinMode(SPD_L, INPUT_PULLUP);
    pinMode(SPD_R, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SPD_L), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(SPD_R), isrRight, RISING);

    // 4. Config Buttons
    pinMode(START_BTN, INPUT_PULLUP);
    pinMode(STOP_BTN, INPUT_PULLUP);
    pinMode(ABORT_BTN, INPUT_PULLUP);

    Serial.println("✅ AGVMCU INITIALIZED (S3 SAFE MODE)");
}

void AGVMCU::update() {
    // Button Debounce
    if (millis() - lastButtonPressTime > 100) { 
        if (digitalRead(ABORT_BTN) == LOW) { lastButtonPressTime = millis(); handleAbort(); }
        else if (digitalRead(STOP_BTN) == LOW) { lastButtonPressTime = millis(); handleStop(); }
        else if (digitalRead(START_BTN) == LOW) { lastButtonPressTime = millis(); handleStart(); }
    }

    // Obstacle Timer
    if (currentState == STATE_OBSTACLE_WAITING) {
        if (millis() - obstacleTimerStart > 10000) { 
            Serial.println("⚠️ Obstacle Timeout! Retreating.");
            initiateRetreat();
        }
        return; 
    }

    // PID Loop
    if (currentState == STATE_MOVING || currentState == STATE_TURNING || currentState == STATE_RETURNING) {
        runPIDCycle();
    }
}

// --- SIMPLIFIED PWM LOGIC (S3 COMPATIBLE) ---
// We use Arduino's analogWrite which handles LEDC timers automatically on S3
void AGVMCU::setPWM(float left, float right) {
    // Constrain 0-100%
    left = constrain(left, 0, 100); 
    right = constrain(right, 0, 100);
    
    // Map 0-100 to 0-255 (Standard 8-bit resolution)
    int dutyL = (int)(left * 2.55);
    int dutyR = (int)(right * 2.55);

    analogWrite(PWM_L, dutyL);
    analogWrite(PWM_R, dutyR);
}

// --- REMAINING LOGIC (UNCHANGED FROM PREVIOUS CORRECT VERSION) ---
void AGVMCU::processCommand(char* cmd) {
    while (*cmd == ' ' || *cmd == '\n' || *cmd == '\r') cmd++;

    // ABORT UNLOCK
    if (currentState == STATE_ABORT_WAITING_POS) {
        if (strncmp(cmd, "POS:", 4) == 0) {
            currentState = STATE_IDLE; 
            Serial.println("✅ Abort Cleared.");
        } else return;
    }

    // DISTANCE
    if (strncmp(cmd, "DISTANCE:", 9) == 0) {
        float dist = atof(cmd + 9);
        if (dist < distanceThreshold && currentState == STATE_MOVING) {
            currentState = STATE_OBSTACLE_WAITING;
            motorBrake();
            noInterrupts(); pulsesTraveledBeforeStop = (abs(encL) + abs(encR)) / 2; interrupts();
            obstacleTimerStart = millis();
            Serial.print("⛔ Obstacle! Pulses: "); Serial.println(pulsesTraveledBeforeStop);
        } else if (dist >= distanceThreshold && currentState == STATE_OBSTACLE_WAITING) {
            Serial.println("✅ Obstacle Cleared.");
            currentState = STATE_MOVING; 
        }
        return;
    }

    // MANHATTAN PATH
    if (strncmp(cmd, "Manhattan", 9) == 0) {
        totalSteps = 0;
        char* ptr = strchr(cmd, '(');
        while (ptr != NULL) {
            if(totalSteps >= 50) break;
            int x = atoi(ptr + 1);
            ptr = strchr(ptr, ','); if (!ptr) break;
            int y = atoi(ptr + 1);
            ptr = strchr(ptr, ')'); if (!ptr) break;
            char dir = *(ptr + 1);
            path[totalSteps].x = x; path[totalSteps].y = y; path[totalSteps].dir = dir;
            totalSteps++;
            ptr = strchr(ptr + 1, '(');
        }
        Serial.print("Path Received. Steps: "); Serial.println(totalSteps);
    }

    // POS UPDATE
    if (strncmp(cmd, "POS:", 4) == 0) {
        char* ptr = cmd + 4;
        currentX = atoi(ptr);
        ptr = strchr(ptr, ','); if (!ptr) return;
        currentY = atoi(ptr + 1);
        ptr = strchr(ptr + 1, ','); if (!ptr) return;
        float angle = atof(ptr + 1);
        
        if(angle > 315 || angle <= 45) currentDir = 'E';
        else if(angle > 45 && angle <= 135) currentDir = 'N';
        else if(angle > 135 && angle <= 225) currentDir = 'W';
        else currentDir = 'S';
        
        if(currentState == STATE_WAITING_QR) handleStart();
    }
}

void AGVMCU::runPIDCycle() {
    if (millis() - lastPIDTime < PID_INTERVAL_MS) return;
    long l, r;
    noInterrupts(); l = abs(encL); r = abs(encR); interrupts();
    long avg = (l + r) / 2;

    if (avg >= targetPulses) {
        motorBrake();
        if (currentState == STATE_TURNING) {
            currentDir = targetDirPending; 
            Serial.print("Turn Done. Facing: "); Serial.println(currentDir);
            currentState = nextStateAfterTurn; 
            if (currentState == STATE_RETURNING) {
                targetPulses = pulsesTraveledBeforeStop; 
                slowdownStartPulses = targetPulses * MOVE_SLOWDOWN_START;
                resetEncoders(); motorForward();
                prevError=0; integral=0; lastPIDTime=millis(); slowdownPhase=false;
                setPWM(BASE_SPEED, BASE_SPEED);
            } else if (currentState == STATE_ABORT_WAITING_POS) {
                Serial.println("Task Aborted/Waiting Angle");
            }
        } else if (currentState == STATE_MOVING) {
            currentX = path[currentStepIndex].x;
            currentY = path[currentStepIndex].y;
            currentStepIndex++;
            currentState = STATE_WAITING_QR;
            Serial.println("Node Reached. Waiting QR...");
        } else if (currentState == STATE_RETURNING) {
            Serial.println("Returned to node. Idle.");
            currentState = STATE_IDLE;
        }
        return;
    }

    float currentSpeedTarget = (currentState == STATE_TURNING) ? TURN_SPEED : BASE_SPEED;
    if (avg >= slowdownStartPulses) slowdownPhase = true;
    if (slowdownPhase) {
        float progress = (float)(avg - slowdownStartPulses) / (targetPulses - slowdownStartPulses);
        progress = constrain(progress, 0.0, 1.0);
        float minSpd = FINAL_SPEED;
        if(currentState == STATE_TURNING) 
             currentSpeedTarget -= (currentSpeedTarget - minSpd) * (progress * progress);
        else currentSpeedTarget -= (currentSpeedTarget - minSpd) * progress;
    }

    long error = l - r;
    float dt = (millis() - lastPIDTime) / 1000.0;
    integral = constrain(integral + error * dt, -100, 100); 
    float output = constrain(KP * error + KI * integral + KD * ((error - prevError) / dt), -MAX_CORRECTION, MAX_CORRECTION);

    float spdL = currentSpeedTarget - output;
    float spdR = currentSpeedTarget + output;
    if (error > 0) spdL -= abs(output); else spdR -= abs(output);
    setPWM(spdL, spdR);
    prevError = error;
    lastPIDTime = millis();
}

void AGVMCU::handleStart() {
    if (currentStepIndex >= totalSteps) return;
    Step target = path[currentStepIndex];
    if (currentDir != target.dir) {
        rotateToDirection(currentDir, target.dir);
        nextStateAfterTurn = STATE_MOVING; 
    } else {
        targetPulses = mmToPulses(SQUARE_SIZE_MM);
        slowdownStartPulses = targetPulses * MOVE_SLOWDOWN_START;
        resetEncoders(); motorForward();
        currentState = STATE_MOVING;
        prevError=0; integral=0; lastPIDTime=millis(); slowdownPhase=false;
        setPWM(BASE_SPEED, BASE_SPEED);
    }
}

void AGVMCU::initiateRetreat() {
    rotateToDirection(currentDir, getOppositeDir(currentDir));
    nextStateAfterTurn = STATE_RETURNING;
}

void AGVMCU::handleAbort() {
    motorBrake();
    if (currentState == STATE_ABORT_WAITING_POS) return;
    Serial.println("ABORTING!");
    rotateToDirection(currentDir, getOppositeDir(currentDir));
    nextStateAfterTurn = STATE_ABORT_WAITING_POS;
}

void AGVMCU::rotateToDirection(char from, char to) {
    motorBrake(); delay(100); 
    int diff = 0;
    if (from == 'E') { if(to=='N') diff=90; if(to=='W') diff=180; if(to=='S') diff=-90; }
    if (from == 'N') { if(to=='W') diff=90; if(to=='S') diff=180; if(to=='E') diff=-90; }
    if (from == 'W') { if(to=='S') diff=90; if(to=='E') diff=180; if(to=='N') diff=-90; }
    if (from == 'S') { if(to=='E') diff=90; if(to=='N') diff=180; if(to=='W') diff=-90; }
    if (diff == 0) return; 
    bool isRight = (diff < 0);
    targetPulses = turnDegreesToPulses(abs(diff));
    slowdownStartPulses = targetPulses * getTurnSlowdownStart(abs(diff));
    targetDirPending = to; 
    resetEncoders(); if(isRight) motorRight(); else motorLeft();
    currentState = STATE_TURNING;
    prevError=0; integral=0; lastPIDTime=millis(); slowdownPhase=false;
    setPWM(TURN_SPEED, TURN_SPEED);
}

void AGVMCU::handleStop() { motorBrake(); currentState = STATE_STOPPED; Serial.println("STOPPED"); }
void AGVMCU::motorBrake()   { digitalWrite(BRK_L, HIGH); digitalWrite(BRK_R, HIGH); setPWM(0,0); }
void AGVMCU::motorForward() { digitalWrite(DIR_L, HIGH); digitalWrite(DIR_R, HIGH); digitalWrite(BRK_L, LOW); digitalWrite(BRK_R, LOW); }
void AGVMCU::motorLeft()    { digitalWrite(DIR_L, LOW);  digitalWrite(DIR_R, HIGH); digitalWrite(BRK_L, LOW); digitalWrite(BRK_R, LOW); }
void AGVMCU::motorRight()   { digitalWrite(DIR_L, HIGH); digitalWrite(DIR_R, LOW);  digitalWrite(BRK_L, LOW); digitalWrite(BRK_R, LOW); }
void AGVMCU::resetEncoders() { noInterrupts(); encL=0; encR=0; interrupts(); }
char AGVMCU::getOppositeDir(char dir) { if (dir == 'N') return 'S'; if (dir == 'S') return 'N'; if (dir == 'E') return 'W'; return 'E'; }
long AGVMCU::mmToPulses(float mm) { return (long)((mm / (PI * WHEEL_DIAMETER_MM)) * PULSES_PER_REV * LINEAR_CALIBRATION); }
long AGVMCU::turnDegreesToPulses(float deg) { float arc = (PI * WHEEL_BASE_MM) * (deg / 360.0f); return (long)((arc / (PI * WHEEL_DIAMETER_MM)) * PULSES_PER_REV * TURN_CALIBRATION); }
float AGVMCU::getTurnSlowdownStart(float deg) { if (deg <= 45) return 0.85; if (deg <= 90) return 0.35; return 0.65; }
