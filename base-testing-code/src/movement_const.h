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

// Constants for calibrating the speed of the motors,
// for some reason the encoders are not reliable.
// Possibly due to number of interrupts from 4 motors
// allowing some to be missed
#define MOTOR_FR_CONSTANT ((double)1.0)
#define MOTOR_FL_CONSTANT ((double)1.00)
#define MOTOR_RL_CONSTANT ((double)1.04)
#define MOTOR_RR_CONSTANT ((double)1.06)

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

#define MM_TO_SQUARES_F 200
// #define MM_TO_SQUARES_F_CORR 2
#define MM_TO_SQUARES_F_CORR 0

#define MM_TO_SQUARES_FB_OFF 60

#define MM_TO_SQUARES_B 265
// #define MM_TO_SQUARES_B_CORR 10
#define MM_TO_SQUARES_B_CORR 0




#define MM_TO_SQUARES_L 200
// #define MM_TO_SQUARES_L_CORR 6
#define MM_TO_SQUARES_L_CORR 0

#define MM_TO_SQUARES_LR_OFF 43
// #define MM_TO_SQUARES_LR_OFF 53

#define MM_TO_SQUARES_R 250
// #define MM_TO_SQUARES_R_CORR 10
#define MM_TO_SQUARES_R_CORR 0

extern int motor_stop_delay;

enum TASK_COMMANDS {
    T_FORWARD,
    T_BACKWARD,
    T_STRAFE_L,
    T_STRAFE_R,
    T_TURN_CW,
    T_TURN_CCW,
    T_GRAB_SAND,
    T_RAMP,
    T_OBSTACLE
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

