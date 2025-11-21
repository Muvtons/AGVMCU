#ifndef AGVMCU_H
#define AGVMCU_H

#include <Arduino.h>
#include "driver/ledc.h"

// ================= HARDWARE SETTINGS =================
#define SQUARE_SIZE_MM      1000
#define WHEEL_DIAMETER_MM   200
#define WHEEL_BASE_MM       600
#define MOTOR_PULSES_PER_REV 24
#define GEARBOX_RATIO       30
#define PULSES_PER_REV      (MOTOR_PULSES_PER_REV * GEARBOX_RATIO)

// PID & SPEED
#define BASE_SPEED          65
#define TURN_SPEED          50
#define FINAL_SPEED         5
#define KP 0.5
#define KI 0.02
#define KD 0.15
#define MAX_CORRECTION      15
#define PID_INTERVAL_MS     50
#define MOVE_SLOWDOWN_START 0.80

// CALIBRATION
#define LINEAR_CALIBRATION  0.895
#define TURN_CALIBRATION    0.55

// PINS
#define SPD_L   2
#define SPD_R   3
#define PWM_L   5
#define PWM_R   6
#define DIR_L   7
#define DIR_R   8
#define BRK_L   9
#define BRK_R   10
#define START_BTN 33
#define STOP_BTN  32
#define ABORT_BTN 35

// ================= STATES =================
enum State {
    STATE_IDLE,
    STATE_MOVING,             // Moving to target
    STATE_TURNING,            // Turning to target heading
    STATE_WAITING_QR,         // Stopped at node, waiting for QR confirmation
    STATE_OBSTACLE_WAITING,   // Stopped by obstacle, counting down 10s
    STATE_RETURNING,          // Obstacle timeout: Driving back exactly distance traveled
    STATE_STOPPED,            // Paused by Stop button
    STATE_ABORT_WAITING_POS   // Aborted, Turned 180, waiting for POS update
};

struct Step {
    int x;
    int y;
    char dir;
};

class AGVMCU {
private:
    // Motor & PID
    void initPWM();
    void setPWM(float left, float right);
    void motorBrake();
    void motorForward();
    void motorLeft();
    void motorRight();
    void runPIDCycle();
    void resetEncoders();
    
    // Helpers
    long mmToPulses(float mm);
    long turnDegreesToPulses(float deg);
    float getTurnSlowdownStart(float deg);
    char getOppositeDir(char dir);

    // Logic
    void rotateToDirection(char from, char to);
    void initiateRetreat(); // Sets up the precision return

    // Variables
    Step path[50];
    int totalSteps;
    int currentStepIndex;
    
    // Position
    int currentX, currentY;
    char currentDir;
    float currentAngleRaw; // Stores raw angle if needed
    
    // State
    State currentState;
    State nextStateAfterTurn; // Decides what happens after a turn finishes

    // Obstacle Logic
    float distanceThreshold;
    unsigned long obstacleTimerStart;
    long pulsesTraveledBeforeStop; // The "Memory" for precision retreat

    // PID vars
    long targetPulses;
    long slowdownStartPulses;
    float prevError, integral;
    unsigned long lastPIDTime;
    bool slowdownPhase;

public:
    AGVMCU();
    void begin();
    void update();
    void processCommand(String command);
    
    // Button Handlers
    void handleStart();
    void handleStop();
    void handleAbort();
};

extern AGVMCU agvmcu;
// Interrupts
void IRAM_ATTR isrLeft();
void IRAM_ATTR isrRight();
extern volatile long encL;
extern volatile long encR;

#endif
