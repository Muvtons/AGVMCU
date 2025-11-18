#ifndef AGVMCU_H
#define AGVMCU_H

#include <Arduino.h>

// Motor pins (L298N driver)
#define ENA 16    // Motor A PWM
#define IN1 17    // Motor A direction 1
#define IN2 18    // Motor A direction 2
#define ENB 19    // Motor B PWM
#define IN3 20    // Motor B direction 1
#define IN4 21    // Motor B direction 2

// Button pins (active LOW with pull-ups)
#define START_BUTTON_PIN 4
#define STOP_BUTTON_PIN 5
#define ABORT_BUTTON_PIN 6

// Navigation constants
#define maxSteps 50      // Maximum waypoints in path

// ============================================================================
// STATE MACHINE - UPDATED FOR QR-VERIFIED NAVIGATION
// ============================================================================
enum AGVState {
    STATE_IDLE,                 // No path loaded, system idle
    STATE_WAITING_QR,          // At waypoint, QR verification required
    STATE_ADJUSTING_ORIENTATION, // Rotating to correct direction
    STATE_MOVING,              // Moving to next waypoint
    STATE_STOPPED,             // Paused by STOP command
    STATE_BLOCKED,             // Obstacle detected, waiting for new path
    STATE_QR_MISMATCH,         // QR code didn't match path (critical error)
    STATE_GOAL_REACHED         // Successfully completed path
};

// ============================================================================
// PATH STEP STRUCTURE - NOW INCLUDES DISTANCE
// ============================================================================
struct Step {
    int x;           // Grid X coordinate
    int y;           // Grid Y coordinate
    char dir;        // Direction: 'N', 'S', 'E', 'W'
    int distanceCm;  // Distance to travel in centimeters
};

class AGVMCU {
public:
    AGVMCU();
    
    void begin(long baudRate = 115200, bool initSerial = true);
    void update();
    void processCommand(const char* cmd);

private:
    // =========================================================================
    // NAVIGATION STATE CORE
    // =========================================================================
    Step path[maxSteps];        // Loaded navigation path
    int totalSteps;             // Total waypoints in path
    int currentStepIndex;       // Last verified step index
    int targetStepIndex;        // Next step to move to
    int lastQRCodeStep;         // Most recent QR-confirmed step
    
    // Current and expected positions
    int currentX, currentY;     // Ground truth from QR
    int expectedX, expectedY;   // Expected from path
    char currentDir;            // Current facing direction
    char expectedDir;           // Expected direction from path
    AGVState currentState;      // Main state machine state
    
    // =========================================================================
    // OBSTACLE HANDLING
    // =========================================================================
    int blockedStepIndex;       // Where obstacle was encountered
    float distanceThreshold;    // Obstacle detection threshold (meters)
    
    // =========================================================================
    // BUTTON DEBOUNCING
    // =========================================================================
    unsigned long lastAbortPress;
    unsigned long lastStartPress;
    unsigned long lastStopPress;
    
    // =========================================================================
    // NON-BLOCKING MOVEMENT TIMING
    // =========================================================================
    unsigned long moveStartTime;
    bool isMoving;
    unsigned long moveDurationMs;   // Calculated from distance
    
    unsigned long rotateStartTime;
    bool isRotating;
    unsigned long targetRotationTime;
    
    // =========================================================================
    // PRIMARY COMMAND PROCESSORS
    // =========================================================================
    void processQRCode(String qrData);
    void handleQRMatch(int stepIndex, float qrAngle);
    void handleQRMismatch();
    void loadPath(String rawPath);
    void clearPath();
    
    // =========================================================================
    // CONTROL COMMAND HANDLERS
    // =========================================================================
    void handleStart();
    void handleStop();
    void handleAbort();
    void handleButtons();
    void handleObstacleDetection(float distance);
    
    // =========================================================================
    // NAVIGATION EXECUTION
    // =========================================================================
    void navigateToStep(int stepIndex);
    void startMoveToNextWaypoint();
    void startRotateToDirection(char targetDir);
    
    // =========================================================================
    // PATH PARSING AND VALIDATION
    // =========================================================================
    void parsePath(String raw);
    int findStepIndex(int x, int y);
    char calculateDirectionToTarget(int fromX, int fromY, int toX, int toY);
    
    // =========================================================================
    // MOTOR CONTROL
    // =========================================================================
    void stopMotors();
    void rotateAngle(float degrees);  // Blocking (emergency use only)
    
    // =========================================================================
    // UTILITY CALCULATIONS
    // =========================================================================
    int calculateAngleDifference(char from, char to);
    unsigned long calculateRotationTime(float degrees);
    unsigned long calculateMoveDuration(float distanceCm);
    void correctDirectionUsingQRAngle(float qrAngle);
    const char* getStateName();
    
    // =========================================================================
    // STATUS PUBLISHING
    // =========================================================================
    void publishCurrentPosition();
};

// Global instance
extern AGVMCU agvmcu;

#endif // AGVMCU_H
