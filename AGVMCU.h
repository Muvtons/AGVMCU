#ifndef AGVMCU_H
#define AGVMCU_H

#include <Arduino.h>

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
#define LINEAR_CALIBRATION   0.895
#define TURN_CALIBRATION     0.55

// ================= PIN DEFINITIONS (ESP32-S3 SAFE) =================
// DO NOT USE PINS 26-37 (Reserved for Flash/PSRAM on S3)
// DO NOT USE PINS 19-20 (Reserved for USB JTAG)

// Encoders (Input)
#define SPD_L     4  // Changed from 2
#define SPD_R     5  // Changed from 3

// Motor PWM (Speed)
#define PWM_L     6  // Changed from 5
#define PWM_R     7  // Changed from 6

// Motor Direction
#define DIR_L     15 // Changed from 7
#define DIR_R     16 // Changed from 8

// Motor Brakes
#define BRK_L     17 // Changed from 9
#define BRK_R     18 // Changed from 10

// Buttons (Moved to high numbers usually safe on DevKits)
// If your board lacks these pins, use 11, 12, 13
#define START_BTN 12 // Changed from 33 (PSRAM conflict)
#define STOP_BTN  13 // Changed from 32 (PSRAM conflict)
#define ABORT_BTN 14 // Changed from 35 (PSRAM conflict)

// ================= STATES =================
enum State {
    STATE_IDLE,
    STATE_MOVING, STATE_TURNING, STATE_WAITING_QR,
    STATE_OBSTACLE_WAITING, STATE_RETURNING,
    STATE_STOPPED, STATE_ABORT_WAITING_POS
};

struct Step { int x; int y; char dir; };

class AGVMCU {
private:
    // --- MOTOR CONTROL ---
    void setPWM(float left, float right);
    void motorBrake();
    void motorForward();
    void motorLeft();
    void motorRight();
    void resetEncoders();

    // --- LOGIC ---
    void runPIDCycle();
    void rotateToDirection(char from, char to);
    void initiateRetreat();
    char getOppositeDir(char dir);
    
    // --- MATH ---
    long mmToPulses(float mm);
    long turnDegreesToPulses(float deg);
    float getTurnSlowdownStart(float deg);

    // --- VARIABLES ---
    Step path[50];
    int totalSteps;
    int currentStepIndex;
    int currentX, currentY;
    char currentDir;
    char targetDirPending; 
    State currentState;
    State nextStateAfterTurn; 

    float distanceThreshold;
    unsigned long obstacleTimerStart;
    long pulsesTraveledBeforeStop; 
    unsigned long lastButtonPressTime; 

    long targetPulses;
    long slowdownStartPulses;
    float prevError, integral;
    unsigned long lastPIDTime;
    bool slowdownPhase;

public:
    AGVMCU();
    void begin();
    void update(); 
    void processCommand(char* command);
    void handleStart();
    void handleStop();
    void handleAbort();
};

extern AGVMCU agvmcu;
void IRAM_ATTR isrLeft();
void IRAM_ATTR isrRight();
extern volatile long encL;
extern volatile long encR;

#endif
