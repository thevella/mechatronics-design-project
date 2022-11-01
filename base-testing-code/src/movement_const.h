#pragma once
#include <Arduino.h>

#include "options.h"

// Constants for motor control

// Need to define each wheel, location matters
// more than usual due to having to sum vectors
#define MOTOR_FR 1
#define MOTOR_FL 4
#define MOTOR_RL 3
#define MOTOR_RR 2

// Define how the wires are hooked up, easier than
// changing the wires every time
#define MOTOR_FR_FORWARD FORWARD
#define MOTOR_FL_FORWARD BACKWARD
#define MOTOR_RL_FORWARD FORWARD
#define MOTOR_RR_FORWARD BACKWARD

// Define backwards for every motor based on how 
// forward is defined
#if MOTOR_FR_FORWARD == FORWARD
#define MOTOR_FR_BACKWARD BACKWARD
#else
#define MOTOR_FR_BACKWARD FORWARD
#endif

#if MOTOR_FL_FORWARD == FORWARD
#define MOTOR_FL_BACKWARD BACKWARD
#else
#define MOTOR_FL_BACKWARD FORWARD
#endif

#if MOTOR_RL_FORWARD == FORWARD
#define MOTOR_RL_BACKWARD BACKWARD
#else
#define MOTOR_RL_BACKWARD FORWARD
#endif

#if MOTOR_RR_FORWARD == FORWARD
#define MOTOR_RR_BACKWARD BACKWARD
#else
#define MOTOR_RR_BACKWARD FORWARD
#endif


extern int motor_stop_delay;

enum TASK_COMMANDS {
    T_FORWARD,
    T_BACKWARD,
    T_STRAFE_L,
    T_STRAFE_R,
    T_TURN_CW,
    T_TURN_CCW,
    T_GRAB_SAND
};

/**
 * @brief   Type for passing if centering left to right,
 *          or forward to backward
 * 
 */
enum CENTER_TYPE {
    CENTER_F_B,
    CENTER_L_R
};

/**
 * @brief   Type to pass robot movement, roughly
 *          Does not allow arbitrary movement
 * 
 */
enum ROBOT_DIR {
    RB_FORWARD,
    RB_BACKWARD,
    RB_RIGHT,
    RB_LEFT,
    RB_STOP,
    RB_TURN_CC,
    RB_TURN_CW
};

// Only need to export if we are tuning, otherwise keep it internal
#ifdef TUNE_PID_ROTATE
extern uint16_t max_speed;
#endif