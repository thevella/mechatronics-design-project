
#include "main.h"
#include "movement.h"
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>

bool do_rotate = false;

void setup() {
    // Small delay so we can clear the wires before it starts
    delay(1000);
    // Increase resolution so less error in reading sensors
    // new range 0-4095
    analogReadResolution(12);

    // Start serial interface for debugging
    Serial.begin(9600);
    while (!Serial) {}


    #ifdef USE_NFC
    nfc_setup();
    #endif

    // Call setup functions
    setup_sensors();
    setup_movement();



    // Only pull in lora if we want to use it, otherwise
    // will complain and cause slowdowns if unplugged
    #ifdef USE_LORA
    lora_setup();
    #endif

    /*
     * Serial.print("\033[2J");   // clear screen
     * Serial.print("\033[0;0H"); // set cursor to 0,0
     * Serial.print("\033[10B");  // move cursor down 10 lines
     * Serial.print("\033[5A");  // move cursor up 5 lines
     * Serial.write(13); // Beginning of line
     */
    // robot_move(RB_FORWARD);
    // while(true) {
    //     if (round(millis()/1000.0) % 2 == 0 ){
    //         robot_move(RB_FORWARD);
    //     } else {
    //         robot_move(RB_BACKWARD);
    //     }
    //     Serial.print(millis());
    //     Serial.write(13);
        
    //     delay(10);
    //     // Serial.print("\033[2J");
    //     // Serial.print("\033[0;0H");
    // }



    
}



void loop() {

    navigate_maze();

    while(true){}

   
}









