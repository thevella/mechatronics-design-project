
#include "main.h"
#include "movement.h"
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>


bool do_rotate = false;


void setup() {
    analogReadResolution(12);
    Serial.begin(9600);           // set up Serial library at 9600 bps
    while (!Serial) {}

    setup_sensors();
    setup_movement();

    //lora_setup();



}

void forward(int number, bool center = false, bool hard = false) {
    const int move_delay = 2000;


    if (number >= 2) {
        for (int i = 0; i < number/2; ++i) {
            robot_move(RB_FORWARD);
            delay((move_delay+50)*2);
            robot_move(RB_STOP);
            delay(500);
            if (center) {
                if (hard) {
                    robot_move(RB_RIGHT);
                    delay(1000);
                    robot_move(RB_STOP);
                    delay(500);
                }
                recenter();
                delay(500);
            } 
        }
    }

    if (number % 2 == 1) {
        robot_move(RB_FORWARD);
        delay(move_delay);
        robot_move(RB_STOP);
        delay(500);
    }

    if (center) {
        if (hard) {
            robot_move(RB_RIGHT);
            delay(1000);
            robot_move(RB_STOP);
            delay(500);
        }
        recenter();
        delay(500);
    }
    
    

    delay(500);
}

void loop() {
    // if (joystick_com.button) {
    //     manual_rotate(joystick_com.x);
    // } else {
    //     manual_move(joystick_com.x, joystick_com.y);
    // }

    //check_message();
    //Serial.println(dist_left.raw_value());

    //delay(5);

    //Serial.println(analogRead(gpio));
    //sprintf(output, "X: %04i, Y: %04i, C: %i", analogRead(x_pin), analogRead(y_pin), do_rotate);
    //Serial.println(output);

    recenter();

    forward(1, false);

    robot_rotation(RB_TURN_CW);

    delay(1800);

    robot_move(RB_STOP);

    delay(500);

    forward(1, true, true);


    forward(2, false);

    robot_move(RB_RIGHT);

    delay(2000);

    robot_move(RB_STOP);

    delay(500);

    forward(1, false);

    robot_move(RB_LEFT);

    delay(2100);

    robot_move(RB_STOP);

    delay(500);

    forward(2, false);

    robot_rotation(RB_TURN_CW);

    delay(1800);

    robot_move(RB_STOP);

    delay(500);

    forward(1, true, true);

    forward(3, true);

    while(true) {}

}









