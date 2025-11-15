#ifndef AGV_NAVIGATION_H
#define AGV_NAVIGATION_H

#include <Arduino.h>

// Navigation states
enum AGVState { 
    AGV_STATE_IDLE, 
    AGV_STATE_WAITING_FOR_QR, 
    AGV_STATE_MOVING, 
    AGV_STATE_WAITING_QR_CONFIRMATION, 
    AGV_STATE_GOAL_REACHED,
    AGV_STATE_ABORTED,
    AGV_STATE_STOPPED,
    AGV_STATE_OBSTACLE_AVOIDANCE
};

// Step structure
struct AGVStep {
    int x;
    int y;
    char dir;
};

class AGVNavigation {
private:
    // Pin definitions
    int _ena, _in1, _in2, _enb, _in3, _in4;
    int _startButton, _stopButton, _abortButton;
    
    // Navigation variables
    AGVState _currentState;
    String _inputBuffer;
    char _currentDir;
    int _currentX, _currentY;
    int _lastSafeX, _lastSafeY;
    char _lastSafeDir;
    
    // Path variables
    static const int MAX_STEPS = 20;
    AGVStep _path[MAX_STEPS];
    int _totalSteps;
    int _currentStepIndex;
    
    // Obstacle detection
    float _distanceThreshold;
    unsigned long _distanceBelowThresholdStart;
    const unsigned long _distanceTimeoutMs;
    bool _distanceBelowThreshold;
    
    // Obstacle recovery
    int _blockedTargetX, _blockedTargetY;
    int _originalDestinationX, _originalDestinationY;
    bool _isInObstacleRecovery;
    
    // Button debounce
    unsigned long _lastAbortPress, _lastStartPress, _lastStopPress;
    
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
    // Constructor
    AGVNavigation(int ena = 10, int in1 = 3, int in2 = 4, 
                  int enb = 11, int in3 = 5, int in4 = 6,
                  int startBtn = 7, int stopBtn = 8, int abortBtn = 9);
    
    // Initialization
    void begin(long baudRate = 115200);
    
    // Main loop function
    void update();
    
    // Control functions
    void startNavigation();
    void stopNavigation();
    void abortNavigation();
    bool loadPath(String pathString);
    void setQRPosition(int x, int y, float angle);
    void setDistance(float distance);
    
    // State query functions
    AGVState getState();
    String getStateString();
    bool isGoalReached();
    bool isMoving();
    bool isObstacleDetected();
    
    // Position functions
    void getCurrentPosition(int& x, int& y, char& dir);
    int getCurrentX();
    int getCurrentY();
    char getCurrentDirection();
    
    // Path functions
    int getTotalSteps();
    int getCurrentStep();
    bool hasPath();
    
    // Obstacle functions
    void setDistanceThreshold(float threshold);
    float getDistanceThreshold();
};

#endif
