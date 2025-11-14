#ifndef AGV_NAVIGATION_H
#define AGV_NAVIGATION_H

#include <Arduino.h>

class AGV_Navigation {
public:
    void begin();
    void run();

private:
    // === Pin Definitions ===
    const int ENA = 10;
    const int IN1 = 3;
    const int IN2 = 4;
    const int ENB = 11;
    const int IN3 = 5;
    const int IN4 = 6;

    const int startButtonPin = 7;
    const int stopButtonPin  = 8;
    const int abortButtonPin = 9;
    const int defaultButtonPin = 12;

    // === Distance Threshold ===
    const float DISTANCE_THRESHOLD = 0.1;
    unsigned long distanceBelowThresholdStart = 0;
    const unsigned long DISTANCE_TIMEOUT_MS = 10000;
    bool distanceBelowThreshold = false;

    // === Navigation State ===
    bool isStarted = false;
    bool isStopped = false;
    bool goalReached = false;
    bool awaitingPositionAfterNewPath = false;
    bool movementCompleted = false;
    bool abortActive = false;
    bool waitingForQRAfterAbort = false;
    bool distanceTimeoutActive = false;

    String inputBuffer = "";
    String lastPositionInput = "";

    char currentDir = 'E';
    int currentX = -1;
    int currentY = -1;

    int lastValidX = -1;
    int lastValidY = -1;

    struct Step {
        int x;
        int y;
        char dir;
    };

    const int maxSteps = 20;
    Step steps[maxSteps];
    int totalSteps = 0;
    int currentStep = 0;

    // Private methods
    void handleButtons();
    void handleSerialInput();
    void handleAbort();
    void handleDistanceTimeout();
    void moveOneCell();
    void rotateAngle(float degrees);
    void correctDirectionUsingQRAngle(float qrAngle);
    void checkDistanceCondition(float currentDistance);
    void publishCurrentPosition();
    void parsePath(String raw);
    void moveForward();
    void stopMotors();
    void triggerRecoveryProcedure(const char* source);
    void rotateToDirection(char from, char to);
};

// Changed from AGV to AGVMCU
extern AGV_Navigation AGVMCU;

#endif
