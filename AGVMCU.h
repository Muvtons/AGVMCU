#ifndef AGVMCU_H
#define AGVMCU_H

#include <Arduino.h>

// Motor pins
#define ENA 16
#define IN1 17
#define IN2 18
#define ENB 19
#define IN3 20
#define IN4 21

// Button pins
#define START_BUTTON_PIN 4
#define STOP_BUTTON_PIN 5
#define ABORT_BUTTON_PIN 6

// Navigation constants
#define maxSteps 50

// System states
enum AGVState {
    STATE_IDLE,
    STATE_WAITING_FOR_QR,
    STATE_MOVING,
    STATE_WAITING_QR_CONFIRMATION,
    STATE_STOPPED,
    STATE_ABORTED,
    STATE_GOAL_REACHED,
    STATE_OBSTACLE_AVOIDANCE
};

// Path step structure
struct Step {
    int x;
    int y;
    char dir; // 'N', 'S', 'E', 'W'
};

class AGVMCU {
public:
    AGVMCU();
    
    // ✅ UPDATED: Added initSerial parameter
    void begin(long baudRate = 115200, bool initSerial = true);
    void update();
    void processCommand(const char* cmd);

private:
    // Navigation state
    Step path[maxSteps];
    int totalSteps;
    int currentStepIndex;
    int currentX;
    int currentY;
    char currentDir;
    AGVState currentState;
    
    // Obstacle recovery
    int lastSafeX;
    int lastSafeY;
    char lastSafeDir;
    int blockedTargetX;
    int blockedTargetY;
    int originalDestinationX;
    int originalDestinationY;
    bool isInObstacleRecovery;
    
    // Distance sensor
    float distanceThreshold;
    unsigned long distanceTimeoutMs;
    bool distanceBelowThreshold;
    unsigned long distanceBelowThresholdStart;
    
    // Button debouncing
    unsigned long lastAbortPress;
    unsigned long lastStartPress;
    unsigned long lastStopPress;
    
    // Non-blocking movement
    unsigned long moveStartTime;
    bool isMoving;
    unsigned long rotateStartTime;
    bool isRotating;
    unsigned long targetRotationTime;
    unsigned long qrWaitStartTime;
    
    // Command buffer
    String inputBuffer;
    
    // Navigation functions
    void navigateToNextStep();
    void parsePath(String raw);
    bool isAtPosition(int x, int y);
    void updatePositionAfterMove();
    void updateMovementStateMachine();
    int findCurrentStepIndex();
    char calculateDirectionToTarget(int fromX, int fromY, int toX, int toY);
    
    // Motor control
    void startMoveForward();
    void startMoveBackward();
    void moveBackward(); // Blocking version for recovery
    void stopMotors();
    void rotateAngle(float degrees);
    void rotateToDirection(char from, char to);
    void correctDirectionUsingQRAngle(float qrAngle);
    
    // Command handlers
    void handleAbort();
    void handleStop();
    void handleStart();
    void handleButtons();
    
    // Distance sensor
    void checkDistanceCondition(float currentDistance);
    void handleObstacleTimeout();
    void startObstacleRecovery();
    
    // Communication
    void publishCurrentPosition();
    void publishRerouteCommand(int src_x, int src_y, int dst_x, int dst_y);
};

extern AGVMCU agvmcu;

#endif // AGVMCU_H
