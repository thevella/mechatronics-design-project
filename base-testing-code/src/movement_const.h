#pragma once
#include <Arduino.h>

#include "options.h"

#define MOTOR_FR 1
#define MOTOR_FL 2
#define MOTOR_RL 3
#define MOTOR_RR 4

#define MOTOR_FR_FORWARD FORWARD
#define MOTOR_FL_FORWARD BACKWARD
#define MOTOR_RL_FORWARD FORWARD
#define MOTOR_RR_FORWARD BACKWARD

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

enum CENTER_TYPE {
    CENTER_F_B,
    CENTER_L_R
};

enum ROBOT_DIR {
    RB_FORWARD,
    RB_BACKWARD,
    RB_RIGHT,
    RB_LEFT,
    RB_STOP,
    RB_TURN_CC,
    RB_TURN_CW
};

#ifdef TUNE_PID_ROTATE
extern uint16_t max_speed;
#endif