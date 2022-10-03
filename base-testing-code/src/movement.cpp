#include "movement.h"


#define DIST_LEFT  0
#define DIST_RIGHT 1
#define DIST_FRONT 2
#define DIST_BACK  3

#define DIST_VAL   0
#define DIST_GPIO  1

extern const uint8_t distance_sensors[][2];
extern const uint8_t num_dist_sensors;

Adafruit_MotorShield motorshield = Adafruit_MotorShield();

BLA::ArrayMatrix<2, 1, double> xyc = {0,0};
BLA::ArrayMatrix<2, 1, double> speed_solved = {0, 0};
BLA::ArrayMatrix<2,2, double> speed = {0.70710678118655, -0.70710678118655, 0.70710678118655, 0.70710678118655};
auto speed_decomp = BLA::LUDecompose(speed);


bool down = false;

uint16_t max_speed = 4095/2;




void setup_movement() {
    Serial.println("Adafruit Motorshield v2 - DC Motor test!");

    if (!motorshield.begin(1600, &Wire1)) {         // create with the default frequency 1.6KHz
        // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    
    Serial.println("Motor Shield found.");
    
}

void manual_move(uint16_t x, uint16_t y) {
    double xc = (max_analog/(double)2.0) - x;
    double yc = (max_analog/(double)2.0) - y;

    if (abs(xc) < joystick_deadzone) {
        xc = 0;
    }

    if (abs(yc) < joystick_deadzone) {
        yc = 0;
    }

    //double xyc_abs = sqrt(pow(xc/max_analog/2, 2) + pow(yc/max_analog/2, 2));

    xyc = {xc, yc};

    speed_solved = BLA::LUSolve(speed_decomp, xyc);
    
    double speed_abs = sqrt((speed_solved(0)*speed_solved(0)) + (speed_solved(1)*speed_solved(1)));

    double speed_neg = (speed_solved(0)/speed_abs) * max_speed;
    double speed_pos = (speed_solved(1)/speed_abs) * max_speed;


    if (speed_neg > 0) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
    } else {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_BACKWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_BACKWARD);
    }

    if (speed_pos > 0) {
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_FORWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_FORWARD);
    } else {
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);
    }
    

    motorshield.getMotor(MOTOR_FL)->setSpeedFine( (uint16_t)round(abs(speed_neg*(4822/4702.0))) );
    motorshield.getMotor(MOTOR_RL)->setSpeedFine( (uint16_t)round(abs(speed_pos*(4852/4854.0))) );
    motorshield.getMotor(MOTOR_RR)->setSpeedFine( (uint16_t)round(abs(speed_neg*(7047/7199.0))) );
    motorshield.getMotor(MOTOR_FR)->setSpeedFine( (uint16_t)round(abs(speed_pos)) );
}

void manual_rotate(int x) {
    double xc = (max_analog/2) - x;

    if (abs(xc) < joystick_deadzone) {
        xc = 0;
    }
    double speed = (((xc)/(max_analog/2)) * max_speed * (3.0/4)); 

    if (speed < 0) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_BACKWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_FORWARD);
    } else {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);
    }
    

    motorshield.getMotor(MOTOR_FL)->setSpeedFine( (uint16_t)round(abs(speed)) );
    motorshield.getMotor(MOTOR_RL)->setSpeedFine( (uint16_t)round(abs(speed)) );
    motorshield.getMotor(MOTOR_RR)->setSpeedFine( (uint16_t)round(abs(speed)) );
    motorshield.getMotor(MOTOR_FR)->setSpeedFine( (uint16_t)round(abs(speed)) );
}


