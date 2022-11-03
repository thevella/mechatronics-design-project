
#include "main.h"
#include "movement.h"
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>

bool do_rotate = false;


EXTERN_COROUTINE(MovementCoroutine, navigate_maze);
EXTERN_COROUTINE(nfc_read_call);

void setup() {
    // Small delay so we can clear the wires before it starts
    delay(1000);
    // Increase resolution so less error in reading sensors
    // new range 0-4095
    analogReadResolution(12);

    // Start serial interface for debugging
    Serial.begin(9600);
    while (!Serial) {}

    nfc_setup();
    // Call setup functions
    setup_sensors();
    setup_movement();


    // int rot = get_rotation();

    // while(true) {
    //     manual_move(4096/2-1000,4096/2+1000);
    //     delay(5000);
    //     while(abs(get_rotation() + 110) > 2) {
    //         robot_rotation(RB_TURN_CW);
    //         delay(100);
    //     }
    //     robot_move(RB_STOP);

    //     manual_move(4096/2-1000,4096/2+1000);
    //     delay(500);
    //     robot_move(RB_STOP);

    //     Serial.println(get_rotation());
    // }

    // Only pull in lora if we want to use it, otherwise
    // will complain and cause slowdowns if unplugged
    #ifdef USE_LORA
    lora_setup();
    #endif

   
    ROBOT_DIR dir = RB_LEFT;
    int squares = 3;
    int output = 0;
    int target = 0;
    int output2 = 0;


    read_TOF_left(&output);

    bool move = false;
    
    if (output > 900) {
        while (abs(output - 865) > 3) {
            move = move || read_TOF_front(&output2);
            if (move) {
                robot_move_(dir, 0, output2 - MM_TO_SQUARES_FB_OFF);
            }
            move = false;
            delay(25);
            move = read_TOF_left(&output);
        }

        --squares;
    }
    

    if (dir == RB_LEFT) {
        target = output - (squares * MM_TO_SQUARES_LR - (squares - 1) * MM_TO_SQUARES_LR_CORR);
        if (target < MM_TO_SQUARES_LR_OFF) {
            target = MM_TO_SQUARES_LR_OFF;
        }
    } else if (dir == RB_RIGHT) {
        target = output + (squares * MM_TO_SQUARES_LR + (squares - 1) * MM_TO_SQUARES_LR_CORR);
        target = target - (target % MM_TO_SQUARES_LR) + MM_TO_SQUARES_LR_OFF;
    }

    
    
    move = false;

    if (squares != 0) {
        do {
            move = move || read_TOF_front(&output2);
            if (move) {
                robot_move_(dir, 0, output2 - MM_TO_SQUARES_FB_OFF);
            }
            move = false;
            delay(25);
            move = read_TOF_left(&output);
        } while (abs(output - target) > 3);
    }
    
    robot_move(RB_STOP);

    while(true){}

    ace_routine::CoroutineScheduler::setup();
    
}



void loop() {
    ace_routine::CoroutineScheduler::loop();
}









