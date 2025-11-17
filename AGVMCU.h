#ifndef AGVMCU_H
#define AGVMCU_H

#include <Arduino.h>

// Pin definitions
#define ENA 10
#define IN1 3
#define IN2 4
#define ENB 11
#define IN3 5
#define IN4 6

#define START_BUTTON_PIN 7
#define STOP_BUTTON_PIN 8
#define ABORT_BUTTON_PIN 9

// Navigation states
enum State { 
    STATE_IDLE, 
    STATE_WAITING_FOR_QR, 
    STATE_MOVING, 
    STATE_WAITING_QR_CONFIRMATION, 
    STATE_GOAL_REACHED,
    STATE_ABORTED,
    STATE_STOPPED,
    STATE_OBSTACLE_AVOIDANCE
};

// Step structure
struct Step {
    int x;
    int y;
    char dir;
};

class AGVMCU {
private:
    // Navigation variables
    State currentState;
    String inputBuffer;
    String lastPositionInput;
    char currentDir;
    int currentX, currentY;
    int lastSafeX, lastSafeY;
    char lastSafeDir;
    
    // Path variables
    static const int maxSteps = 20;
    Step path[maxSteps];
    int totalSteps;
    int currentStepIndex;
    
    // Obstacle detection
    float distanceThreshold;
    unsigned long distanceBelowThresholdStart;
    const unsigned long distanceTimeoutMs;
    bool distanceBelowThreshold;
    
    // Obstacle recovery
    int blockedTargetX, blockedTargetY;
    int originalDestinationX, originalDestinationY;
    bool isInObstacleRecovery;
    
    // Button debounce
    unsigned long lastAbortPress, lastStartPress, lastStopPress;
    
    // Motor control functions
    void moveForward();
    void moveBackward();
    void stopMotors();
    void rotateAngle(float degrees);
    void rotateToDirection(char from, char to);
    void correctDirectionUsingQRAngle(float qrAngle);
    
    // Navigation functions
    void navigateToNextStep();
    bool isAtPosition(int x, int y);
    void parsePath(String raw);
    
    // Communication functions
    void publishCurrentPosition();
    void publishRerouteCommand(int src_x, int src_y, int dst_x, int dst_y);
    
    // State handling functions
    void handleAbort();
    void handleStop();
    void handleStart();
    void handleButtons();
    void handleSerialInput();
    void checkDistanceCondition(float currentDistance);
    void handleObstacleTimeout();
    void startObstacleRecovery();
    
public:
    AGVMCU();
    void begin(long baudRate = 115200);
    void update();
    void processCommand(const char* command);  // ✅ ADD THIS EXACT LINE
};

// Global instance with your preferred name
extern AGVMCU agvmcu;

#endif
