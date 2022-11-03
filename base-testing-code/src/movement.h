#pragma once
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <SPI.h>
#include <BasicLinearAlgebra.h>
#include "communication.h"
#include "sensors.h"
#include "movement_const.h"
#include <Servo.h>
#include <ArxContainer.h>

#include "scheduling.h"

extern uint16_t robot_acceleration;
extern uint16_t max_speed;

void robot_move(ROBOT_DIR, uint16_t speed = max_speed, uint16_t acceleration = robot_acceleration);

void robot_move_(ROBOT_DIR, int16_t heading_correction = 0, int16_t placement_correction = 0, uint16_t speed = max_speed);

void robot_rotation(ROBOT_DIR, uint16_t speed = max_speed, uint16_t acceleration = robot_acceleration);

void robot_rotation_by_deg (ROBOT_DIR, uint16_t, uint16_t = max_speed, uint16_t = robot_acceleration);

void manual_move(uint16_t, uint16_t);
void manual_rotate(int);

void setup_movement();

void recenter();

void grip_sand();

void forward(int , int16_t delay_offset = 0, bool center = false, bool hard = false, bool on_right = true);

void forward_sense(int , int speed = max_speed);

void backward(int time_offset = 0);

void strafe(ROBOT_DIR , int time_offset = 0);


void turn(ROBOT_DIR , int time_offset = 0);



class MovementCoroutine: public ace_routine::Coroutine {
public:
    int task_move(ROBOT_DIR dir, int squares);
    int task_strafe(ROBOT_DIR dir, int squares);
    int task_rotate(ROBOT_DIR dir, float deg);
    int task_grab_sand();
};

#define task_move_ \
    do { \
        int output = 0; \
        int output2 = 0; \
        int target = 0; \
        read_TOF_front(&output); \
        \
        bool move = false; \
        \
        if (output > 900) { \
            while (abs(output - 865) > 3) { \
                move = move || read_TOF_left(&output2); \
                if (move) { \
                    robot_move_(dir, 0, output2 - MM_TO_SQUARES_LR_OFF); \
                } \
                move = false; \
                COROUTINE_DELAY(25); \
                move = read_TOF_front(&output); \
            } \
        \
            --squares; \
        } \
        \
        \
        if (dir == RB_FORWARD) { \
            target = output - (squares * MM_TO_SQUARES_FB - (squares - 1) * MM_TO_SQUARES_FB_CORR); \
            if (target < MM_TO_SQUARES_FB_OFF) { \
                target = MM_TO_SQUARES_FB_OFF; \
            } \
        } else if (dir == RB_BACKWARD) { \
            target = output + (squares * MM_TO_SQUARES_FB + (squares - 1) * MM_TO_SQUARES_FB_CORR); \
            target = target - (target % MM_TO_SQUARES_FB) + MM_TO_SQUARES_FB_OFF; \
        } \
        \
        \
        \
        move = false; \
        \
        if (squares != 0) { \
            do { \
                move = move || read_TOF_left(&output2); \
                if (move) { \
                    robot_move_(dir, 0, output2 - MM_TO_SQUARES_LR_OFF); \
                } \
                move = false; \
                COROUTINE_DELAY(25); \
                move = read_TOF_front(&output); \
            } while (abs(output - target) > 3); \
        } \
        \
        robot_move(RB_STOP); \
    } while (false)


