
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

   
    ROBOT_DIR dir = RB_FORWARD;
    float deg = 90;
    float output = 0;
    float target = 0;

    output = get_rotation();

    if (dir == RB_TURN_CW) {
        target = add_degrees(output, deg);
    } else if (dir == RB_TURN_CC) {
        target = add_degrees(output, -deg);
    }

    do {
        robot_move(dir);
        delay(50);
        output = get_rotation();
    } while (deg_difference(output, target) > 3);

    robot_move(RB_STOP); 

    while(true){}

    ace_routine::CoroutineScheduler::setup();
    
}



void loop() {
    ace_routine::CoroutineScheduler::loop();
}









