#ifndef AGVMCU_H
#define AGVMCU_H

#include <Arduino.h>
#include "driver/ledc.h"

// ================= PHYSICAL CONSTANTS =================
#define SQUARE_SIZE_MM       1000
#define WHEEL_DIAMETER_MM    200
#define WHEEL_BASE_MM        600
#define MOTOR_PULSES_PER_REV 24
#define GEARBOX_RATIO        30
#define PULSES_PER_REV       (MOTOR_PULSES_PER_REV * GEARBOX_RATIO)

// ================= PID & MOTION SETTINGS =================
#define BASE_SPEED           65
#define TURN_SPEED           50
#define FINAL_SPEED          5
#define KP                   0.5
#define KI                   0.02
#define KD                   0.15
#define MAX_CORRECTION       15
#define PID_INTERVAL_MS      50
#define MOVE_SLOWDOWN_START  0.80

// ================= CALIBRATION =================
#define LINEAR_CALIBRATION   0.895
#define TURN_CALIBRATION     0.55

// ================= PIN DEFINITIONS =================
#define SPD_L     2
#define SPD_R     3
#define PWM_L     5
#define PWM_R     6
#define DIR_L     7
#define DIR_R     8
#define BRK_L     9
#define BRK_R     10

// Buttons
#define START_BTN 33
#define STOP_BTN  32
#define ABORT_BTN 35

// ================= STATES =================
enum State {
    STATE_IDLE,
    STATE_MOVING,             // Normal movement to next node
    STATE_TURNING,            // Rotating to align
    STATE_WAITING_QR,         // Arrived at node, waiting for QR confirmation
    STATE_OBSTACLE_WAITING,   // Obstacle detected, counting down 10s
    STATE_RETURNING,          // Timeout reached, driving back to previous node
    STATE_STOPPED,            // Paused via STOP button
    STATE_ABORT_WAITING_POS   // Aborted, turned 180, waiting for POS update
};

struct Step {
    int x;
    int y;
    char dir;
};

class AGVMCU {
private:
    // --- LOW LEVEL MOTOR CONTROL ---
    void initPWM();
    void setPWM(float left, float right);
    void motorBrake();
    void motorForward();
    void motorLeft();
    void motorRight();
    void resetEncoders();

    // --- CORE LOGIC ---
    void runPIDCycle();
    void rotateToDirection(char from, char to);
    void initiateRetreat();
    char getOppositeDir(char dir);
    
    // --- MATH HELPERS ---
    long mmToPulses(float mm);
    long turnDegreesToPulses(float deg);
    float getTurnSlowdownStart(float deg);

    // --- VARIABLES ---
    // Pathing
    Step path[50];
    int totalSteps;
    int currentStepIndex;
    
    // Positioning
    int currentX, currentY;
    char currentDir;
    char targetDirPending; // CRITICAL: Stores direction we are turning TOWARD, but haven't reached yet
    
    // State Machine
    State currentState;
    State nextStateAfterTurn; // Tells the PID loop what to do after a turn finishes

    // Obstacle Memory
    float distanceThreshold;
    unsigned long obstacleTimerStart;
    long pulsesTraveledBeforeStop; // Saves exact distance traveled to allow precision retreat

    // Safety
    unsigned long lastButtonPressTime; // For debouncing

    // PID Variables
    long targetPulses;
    long slowdownStartPulses;
    float prevError;
    float integral;
    unsigned long lastPIDTime;
    bool slowdownPhase;

public:
    AGVMCU();
    void begin();
    void update(); // The main loop
    
    // CHANGED: Accepts char pointer now (Zero Allocation)
    void processCommand(char* command);
    
    // Handlers
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
