#include "movement.h"
#include "sensors.h"
#include <ArxContainer.h>
#include <math.h>

#ifdef USE_PID_ROTATE
#include <PID_v1.h>
#include <pid.h>
#endif

// How long to let the motors settle to ensure robot has stopped
int motor_stop_delay = 250;

// Initialize motors and servos
Servo claw_servo;
Servo rotate_servo;

// Motor shield is I2C, so we can give it an optional address
Adafruit_MotorShield motorshield = Adafruit_MotorShield();

// Matrixs used for arbitrary strafing in any direction,
// not only on the orthagonal as most setups configure
BLA::ArrayMatrix<2, 1, double> xyc = {0,0};
BLA::ArrayMatrix<2, 1, double> speed_solved = {0, 0};
BLA::ArrayMatrix<2, 2, double> rotation = {0, 0, 0, 0};

// Matrix containing the components of the wheel vectors
// Two wheels make a vector at 45deg into the first quadrant,
// the other two make a vector at 45deg into the second quadrant
/*
   2     1
    \ | /
     \|/
------|------
      |
      |
*/
BLA::ArrayMatrix<2, 2, double> speed_u_vec = {0.70710678118655, -0.70710678118655, 0.70710678118655, 0.70710678118655};
auto speed_decomp = BLA::LUDecompose(speed_u_vec);

// Set max speed as a reduced value
// to limit drift from current setup
uint16_t max_speed = 3800;
int16_t cur_speed = 0;
uint16_t robot_acceleration = 20;

// Constants for calibrating the speed of the motors,
// for some reason the encoders are not reliable.
// Possibly due to number of interrupts from 4 motors
// allowing some to be missed
#define MOTOR_FR_CONSTANT (double)1.0
#define MOTOR_FL_CONSTANT ((double)1.00)
#define MOTOR_RL_CONSTANT ((double)1.04)
#define MOTOR_RR_CONSTANT ((double)1.06)

#ifdef USE_PID_ROTATE
// Variables for PID use
double pid_rot_input=80, pid_rot_output=50, pid_rot_setpoint=180;
double pid_rot_kp=5, pid_rot_ki=0.5, pid_rot_kd=2;

#define PID_LIM_MIN -max_speed /* Limit for PWM. */
#define PID_LIM_MAX max_speed /* Limit for PWM. */
#define DEADBAND 0.0f /* Off==0 */

// PID Object
epid_t pid_rot;

//Define Variables we'll be connecting to
float Input;
int Output = 0;
float deadband_delta;


PID turning_pid(&pid_rot_input, &pid_rot_output, &pid_rot_setpoint, pid_rot_kp, pid_rot_ki, pid_rot_kd, DIRECT);
#endif

void calibrate_center(CENTER_TYPE);

using namespace ace_routine;

std::vector<std::vector<int>> commands {{T_FORWARD, 1}};

int16_t heading = 0;

int MovementCoroutine::task_move(ROBOT_DIR dir, int squares) {
    int output = 0;
    int output2 = 0;
    int target = 0;
    read_TOF_front(&output);

    bool move = false;
    
    if (output > 900) {
        while (abs(output - 865) > 3) {
            move = move || read_TOF_left(&output2);
            if (move) {
                robot_move_(dir, 0, output2 - MM_TO_SQUARES_LR_OFF);
            }
            move = false;
            COROUTINE_DELAY(25);
            move = read_TOF_front(&output);
        }

        --squares;
    }
    

    if (dir == RB_FORWARD) {
        target = output - (squares * MM_TO_SQUARES_FB - (squares - 1) * MM_TO_SQUARES_FB_CORR);
        if (target < MM_TO_SQUARES_FB_OFF) {
            target = MM_TO_SQUARES_FB_OFF;
        }
    } else if (dir == RB_BACKWARD) {
        target = output + (squares * MM_TO_SQUARES_FB + (squares - 1) * MM_TO_SQUARES_FB_CORR);
        target = target - (target % MM_TO_SQUARES_FB) + MM_TO_SQUARES_FB_OFF;
    }

    
    
    move = false;

    if (squares != 0) {
        do {
            move = move || read_TOF_left(&output2);
            if (move) {
                robot_move_(dir, 0, output2 - MM_TO_SQUARES_LR_OFF);
            }
            move = false;
            COROUTINE_DELAY(25);
            move = read_TOF_front(&output);
        } while (abs(output - target) > 3);
    }
    
    robot_move(RB_STOP);
    return 0;
}

int MovementCoroutine::task_strafe(ROBOT_DIR dir, int squares) {
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
            COROUTINE_DELAY(25);
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
            COROUTINE_DELAY(25);
            move = read_TOF_left(&output);
        } while (abs(output - target) > 3);
    }
    
    robot_move(RB_STOP);
    return 0;
}

int MovementCoroutine::task_rotate(ROBOT_DIR dir, float deg) {
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
        COROUTINE_DELAY(50);
        output = get_rotation();
    } while (deg_difference(output, target) > 3);

    heading = target;

    robot_move(RB_STOP); 
    return 0;
}

int MovementCoroutine::task_grab_sand() {
    return 0;
}   

COROUTINE(MovementCoroutine, navigate_maze){
    COROUTINE_LOOP() {
        int output = 0;
        for (auto command: commands) {
            switch(command.at(0)) {
                case(T_FORWARD):
                    this->task_move(RB_FORWARD, command.at(1));
                    break;
                case(T_BACKWARD):
                    this->task_move(RB_BACKWARD, command.at(1));
                    break;
                case(T_STRAFE_L):
                    this->task_strafe(RB_LEFT, command.at(1));
                    break;
                case(T_STRAFE_R):
                    this->task_strafe(RB_RIGHT, command.at(1));
                    break;
                case(T_TURN_CW):
                    this->task_rotate(RB_TURN_CW, command.at(1));
                    break;
                case(T_TURN_CCW):
                    this->task_rotate(RB_TURN_CC, command.at(1));
                    break;
                case(T_GRAB_SAND):
                    this->task_grab_sand();
                    break;
            }
            COROUTINE_DELAY(50);
        }
    }
};

void robot_stop() {
    for (int i = 0; i < 4; ++i) {
        motorshield.getMotor(i)->run(RELEASE);
    }
}


/**
 * @brief Setup for all motor devices
 * 
 */
void setup_movement() {
    // Make sure that we are connected to the shield, otherwise
    // no point continueing
    Serial.println("Adafruit Motorshield v2 - DC Motor test!");

    if (!motorshield.begin(1600, &Wire1)) {
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    
    Serial.println("Motor Shield found.");

    #ifdef USE_PID_ROTATE
    #ifdef USE_GYRO
    // Set target as 90deg from current position
    pid_rot_setpoint = add_degrees(get_rotation(), (float)-90);
    pid_rot_input = get_rotation();
    #endif

    //Setup the pid 
    turning_pid.SetMode(AUTOMATIC);
    turning_pid.SetOutputLimits(PID_LIM_MIN, PID_LIM_MAX);

    // Initialize pid object
    epid_info_t pid_rot_err = epid_init(&pid_rot,
        pid_rot_input, pid_rot_input, -max_speed,
        pid_rot_kp, pid_rot_ki, pid_rot_kd);

    if (pid_rot_err != EPID_ERR_NONE) {
        Serial.print("\n\n** ERROR: epid_err != EPID_ERR_NONE **\n\n");
        while (1) { ; }
    }
    #endif

    heading = get_rotation();

    
    // Setup the center position and calibrate side sensors
    // for distance
    //calibrate_center(CENTER_L_R);
}

/**
 * @brief                 Move forward number of squares using delays
 * 
 * @param number          Number of squares to move forward
 * @param delay_offset    How much to add or subtract from default delay
 * @param center          Whether to center periodically
 * @param hard            Whether centering should be done by hitting the wall first
 * @param on_right        Whether to hard center off the left or right wall
 */
void forward(int number, int16_t delay_offset, bool center, bool hard, bool on_right) {
    const int move_delay = 2050;

    // If moving more than 2 squares, recenter every 2 squares
    if (number >= 2) {
        for (int i = 0; i < number/2; ++i) {
            // Turn motors on and wait for 2 squares,
            // with small corrective delay
            robot_move(RB_FORWARD);
            delay((move_delay+delay_offset)*2);
            robot_move(RB_STOP);
            delay(motor_stop_delay);

            // If we have been asked to center, do so
            if (center) {
                // If we want to hit the wall to correct
                // orientation, do a hard center
                // have the option of either wall if one is more
                // hazardous
                if (hard) {
                    if (on_right) {
                        robot_move(RB_RIGHT);
                    } else {
                        robot_move(RB_LEFT);
                    }
                    
                    delay(1000);
                    robot_move(RB_STOP);
                    delay(motor_stop_delay);
                }
                // Call recenter 
                recenter();
            } 
        }
    }

    // If we had an odd number, we need to finish the movement,
    // same as above
    if (number % 2 == 1) {
        robot_move(RB_FORWARD);
        delay(move_delay + delay_offset);
        robot_move(RB_STOP);
        delay(motor_stop_delay);
        if (center) {
            if (hard) {
                if (on_right) {
                    robot_move(RB_RIGHT);
                } else {
                    robot_move(RB_LEFT);
                }
                delay(1000);
                robot_move(RB_STOP);
                delay(motor_stop_delay);
            }
            recenter();
        }
    }
}

/**
 * @brief         Move forward until sensor threshold is reached
 * 
 * @param offset  Sensor offset threshold (ms)
 * @param speed   Speed at which to turn wheels 0-4095
 */
void forward_sense(int offset, int speed) {
    robot_move(RB_FORWARD, speed);

    while (dist_front.raw_value() < offset) {
        delay(10);
    }

    robot_move(RB_STOP);

    delay(motor_stop_delay);
}

/**
 * @brief              Move back into the wall to reorient 
 * 
 * @param time_offset  Amount to be added or subtracted to default delay (ms)
 */
void backward(int time_offset) {
    robot_move(RB_BACKWARD);

    delay(700 + time_offset);

    robot_move(RB_STOP);

    delay(motor_stop_delay);
}

/**
 * @brief              Strafe left or right one square
 * 
 * @param dir          Direction to strafe
 * @param time_offset  Amount to be added or subtracted to default delay (ms)
 */
void strafe(ROBOT_DIR dir, int time_offset) {
    robot_move(dir);

    delay(2300 + time_offset);

    robot_move(RB_STOP);

    delay(motor_stop_delay);
}

/**
 * @brief              Rotate 90deg CW or CCW
 * 
 * @param dir          Direction to rotate
 * @param time_offset  Amount to be added or subtracted to default delay (ms)
 */
void turn(ROBOT_DIR dir, int time_offset) {
    robot_rotation(dir);

    delay(1800 + time_offset);

    robot_move(RB_STOP);

    delay(motor_stop_delay);
}


/**
 * @brief      Turn on servos and grip sand.
 *   
 * @details    Wait until the end since the servos
 *              can draw a lot of current and can crash
 *              the arduino
 */
void grip_sand(){

    int temp = 15;
    int delay_time = 50;

    // Write to the servo before attaching, otherwise
    // moves to a default position.
    // This ensures the claw is open before lowering
    claw_servo.write(0);
    claw_servo.attach(10);
    delay(300);

    // Detach from servo since we have moved it,
    // ensure enough power for second servo
    claw_servo.detach();

    rotate_servo.write(90);
    rotate_servo.attach(9);
    delay(300);
    
    // Slowly move the arm towards a position so
    // as to control the amount of current it draws
    // Especially since the moment is so large on this servo
    temp = 63;

    if (temp < rotate_servo.read()) {
        int diff = rotate_servo.read() - temp;
        for (int i = abs(diff); i > 0; --i) {
            rotate_servo.write(rotate_servo.read() - 1);
            delay(delay_time);
        }
    } else {
        int diff = rotate_servo.read() - temp;
        for (int i = abs(diff); i > 0; --i) {
            rotate_servo.write(rotate_servo.read() + 1);
            delay(delay_time);
        }
    }

    delay(300);
    
    // Close claw
    claw_servo.write(32);
    claw_servo.attach(10);
}

/**
 * @brief Basic recentering algorithm using the light sensors
 * 
 */
void recenter() {
    // Read our calibrated distance from the wheels to
    // what the sensor is reading
    int64_t left = dist_left.read_dist_wheels();
    int64_t right = dist_right.read_dist_wheels();
    
    // If the difference is too large, probably on an
    // opening or didn't mean to center
    if (abs(left - right) > 1000) {
        return;
    }

    #ifdef DEBUG_PRINT_DIST
    char out1[50];
    sprintf(out1, "Difference: %i", dist_left.read_dist_wheels() - dist_right.read_dist_wheels());
    Serial.println(out1);
    #endif

    // While the difference is larger than 10mm, continue moving
    // Small diference have a likelyhood of being short lived
    // false readings
    while (abs(left - right) > 10) {
        left = dist_left.read_dist_wheels();
        right = dist_right.read_dist_wheels();
        Serial.println("need to move");
        // Moving at decreased speed so as to not overshoot
        // target
        if (left > right) {
            robot_move(RB_LEFT, max_speed/3);
            Serial.println("moving left");
        } else {
            robot_move(RB_RIGHT, max_speed/3);
            Serial.println("moving right");
        }
        delay(100);
    } 

    robot_move(RB_STOP);

    delay(motor_stop_delay);
}

#ifdef USE_PID_ROTATE
#ifdef USE_GYRO

void robot_move_(ROBOT_DIR direction, int16_t heading_correction, int16_t placement_correction, uint16_t speed) {
    double xc = 0;
    double yc = 0;

    int16_t head_corr[4] = {0,0,0,0};

    for (int i = 0; i < 4; ++i) {
        head_corr[i] = heading_correction;
        if (i == MOTOR_FR || i == MOTOR_RR) {
            head_corr[i] *= -1;
        }
    }

    if (direction == RB_FORWARD) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_FORWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_FORWARD);

        xc = max_speed;
    } else if (direction == RB_BACKWARD) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_BACKWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_BACKWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);

        xc = -max_speed;

        for (int i = 0; i < 4; ++i) {
            head_corr[i] *= -1;
        }
    } else if (direction == RB_STOP) {
        motorshield.getMotor(MOTOR_FL)->run(RELEASE);
        motorshield.getMotor(MOTOR_RR)->run(RELEASE);
        motorshield.getMotor(MOTOR_RL)->run(RELEASE);
        motorshield.getMotor(MOTOR_FR)->run(RELEASE);
    }

    if (direction == RB_LEFT) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_BACKWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_BACKWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_FORWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_FORWARD);

        yc = -max_speed;

        head_corr[MOTOR_FL] *= -1;
        head_corr[MOTOR_RR] *= -1;
    } else if (direction == RB_RIGHT) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);

        yc = max_speed;

        head_corr[MOTOR_FR] *= -1;
        head_corr[MOTOR_RL] *= -1;
    }


    // By taking the unit vector of the input, then solving against
    // the wheel vectors defined in the preamble, we are able to arbitrarily
    // strafe
    xyc = {xc, yc};

    double theta = -1 * placement_correction * PI/((double)180.0);

    if (direction == RB_LEFT || direction == RB_RIGHT) {
        theta *= -1;
    }

    rotation = {cos(theta), -sin(theta), sin(theta), cos(theta)};

    xyc = rotation * xyc;

    speed_solved = BLA::LUSolve(speed_decomp, xyc);
    
    double speed_mag = sqrt(pow(speed_solved(0), 2) + pow(speed_solved(1), 2));

    double speed_neg = (speed_solved(0)/speed_mag) * speed;
    double speed_pos = (speed_solved(1)/speed_mag) * speed;

    
    

    motorshield.getMotor(MOTOR_FL)->setSpeedFine( (uint16_t)round((abs(speed_neg)+head_corr[MOTOR_FL])*MOTOR_FL_CONSTANT) );
    motorshield.getMotor(MOTOR_RL)->setSpeedFine( (uint16_t)round((abs(speed_pos)+head_corr[MOTOR_RL])*MOTOR_RL_CONSTANT) );
    motorshield.getMotor(MOTOR_RR)->setSpeedFine( (uint16_t)round((abs(speed_neg)+head_corr[MOTOR_RR])*MOTOR_RR_CONSTANT) );
    motorshield.getMotor(MOTOR_FR)->setSpeedFine( (uint16_t)round((abs(speed_pos)+head_corr[MOTOR_FR])*MOTOR_FR_CONSTANT) );
}


void robot_move_old(ROBOT_DIR direction, int16_t heading_correction, int16_t placement_correction, uint16_t speed) {
    double speed_neg = speed;
    double speed_pos = speed;

    int16_t placement_corr[4] = {0,0,0,0};
    int16_t head_corr[4] = {0,0,0,0};

    for (int i = 0; i < 4; ++i) {
        placement_corr[i] = 0;
        head_corr[i] = heading_correction;
        if (i == MOTOR_FR || i == MOTOR_RR) {
            head_corr[i] *= -1;
        }
    }

    if (direction == RB_FORWARD) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_FORWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_FORWARD);
        
        if (placement_correction < 0) {
            placement_corr[MOTOR_RL] = -placement_correction;
            placement_corr[MOTOR_FR] = -placement_correction;
        } else {
            placement_corr[MOTOR_FL] = -placement_correction;
            placement_corr[MOTOR_RR] = -placement_correction;
        }
    } else if (direction == RB_BACKWARD) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_BACKWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_BACKWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);
        for (int i = 0; i < 4; ++i) {
            head_corr[i] *= -1;
            placement_corr[i] *= -1;
        }
        if (placement_correction > 0) {
            placement_corr[MOTOR_RL] = -placement_correction;
            placement_corr[MOTOR_FR] = -placement_correction;
        } else {
            placement_corr[MOTOR_FL] = -placement_correction;
            placement_corr[MOTOR_RR] = -placement_correction;
        }
    } else if (direction == RB_STOP) {
        motorshield.getMotor(MOTOR_FL)->run(RELEASE);
        motorshield.getMotor(MOTOR_RR)->run(RELEASE);
        motorshield.getMotor(MOTOR_RL)->run(RELEASE);
        motorshield.getMotor(MOTOR_FR)->run(RELEASE);
    }

    if (direction == RB_LEFT) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_BACKWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_BACKWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_FORWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_FORWARD);

        head_corr[MOTOR_FL] *= -1;
        head_corr[MOTOR_RR] *= -1;


    } else if (direction == RB_RIGHT) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);

        head_corr[MOTOR_FR] *= -1;
        head_corr[MOTOR_RL] *= -1;


    }
    
    // Set adjusted speeds and round to an integer 
    motorshield.getMotor(MOTOR_FL)->setSpeedFine( (uint16_t)round((abs(speed_neg)+head_corr[MOTOR_FL]+placement_corr[MOTOR_FL])*MOTOR_FL_CONSTANT) );
    motorshield.getMotor(MOTOR_RL)->setSpeedFine( (uint16_t)round((abs(speed_pos)+head_corr[MOTOR_RL]+placement_corr[MOTOR_RL])*MOTOR_RL_CONSTANT) );
    motorshield.getMotor(MOTOR_RR)->setSpeedFine( (uint16_t)round((abs(speed_neg)+head_corr[MOTOR_RR]+placement_corr[MOTOR_RR])*MOTOR_RR_CONSTANT) );
    motorshield.getMotor(MOTOR_FR)->setSpeedFine( (uint16_t)round((abs(speed_pos)+head_corr[MOTOR_FR]+placement_corr[MOTOR_FR])*MOTOR_FR_CONSTANT) );
}

void robot_rotation_(ROBOT_DIR direction, uint16_t speed, uint16_t acceleration) {

    if (direction == RB_TURN_CC) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_BACKWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_FORWARD);
    } else if (direction == RB_TURN_CW) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);
    } else if (direction == RB_STOP) {
        motorshield.getMotor(MOTOR_FL)->run(RELEASE);
        motorshield.getMotor(MOTOR_RL)->run(RELEASE);
        motorshield.getMotor(MOTOR_RR)->run(RELEASE);
        motorshield.getMotor(MOTOR_FR)->run(RELEASE);
    }
    

    motorshield.getMotor(MOTOR_FL)->setSpeedFine( round(speed * MOTOR_FL_CONSTANT) );
    motorshield.getMotor(MOTOR_RL)->setSpeedFine( round(speed * MOTOR_RL_CONSTANT) );
    motorshield.getMotor(MOTOR_RR)->setSpeedFine( round(speed * MOTOR_RR_CONSTANT) );
    motorshield.getMotor(MOTOR_FR)->setSpeedFine( round(speed * MOTOR_FR_CONSTANT) );
}

/**
 * @brief            Calculate next PID output
 * 
 * @param set_point  Current target value
 * @return uint16_t  PID output
 */
uint16_t pid_rot_calculate(uint16_t set_point) {
    epid_pid_calc(&pid_rot, set_point, get_rotation()); /* Calc PID terms values */

    /* Apply deadband filter to `delta[k]`. */
    deadband_delta = pid_rot.p_term + pid_rot.i_term + pid_rot.d_term;
    if ((deadband_delta != deadband_delta) || (fabsf(deadband_delta) >= DEADBAND)) {
        /* Compute new control signal output */
        epid_pid_sum(&pid_rot, PID_LIM_MIN, PID_LIM_MAX);
        pid_rot_output = (int)lroundf(pid_rot.y_out); /* float to int */
    }

    return pid_rot_output;
}
#endif
#endif


/**
 * @brief      Calibrate center by ramming the walls
 * 
 * @param dir 
 * 
 * @details    Takes readings at the two known distances:
 *              When the sensor is against the wall at almost 4cm,
 *              and when the sensor is far from the wall
 */
void calibrate_center(CENTER_TYPE dir) {
    uint16_t temp[2];

    // Will eventually center forward and backwards
    // so strafing is more possible
    if (dir == CENTER_L_R) {
        // hit the right wall and take readings
        int delay_val = 1600;
        Serial.println("centering right");
        uint16_t old_dist_val = dist_right.raw_value();
        robot_move(RB_RIGHT, max_speed/1.5);
        delay(delay_val);

        robot_move(RB_STOP);

        delay(motor_stop_delay);

        temp[0] = dist_right.raw_value();
        temp[1] = dist_left.raw_value();

        old_dist_val = dist_left.raw_value();

        // hit left wall and take readings    
        Serial.println("centering left");
        robot_move(RB_LEFT, max_speed/1.5);
        delay(delay_val);

        robot_move(RB_STOP);

        // Calibrate the two sensors
        dist_right.calibrate(temp[0], dist_right.raw_value());
        #ifdef DEBUG_PRINT_DIST
        Serial.print(dist_left.raw_value());
        Serial.print(",");
        Serial.println(temp[1]);
        delay(2000);
        #endif
        dist_left.calibrate(dist_left.raw_value(), temp[1]);
    }
}

#ifdef USE_PID_ROTATE
/**
 * @brief PID debug text, taken from library example
 * 
 */
void SerialSend()
{
    Serial.print("pid_rot_setpoint: ");
    Serial.print(pid_rot_setpoint);
    Serial.print(" ");
    Serial.print("input: ");
    Serial.print(pid_rot_input);
    Serial.print(" ");
    Serial.print("output: ");
    Serial.print(pid_rot_output);
    Serial.print(" ");

    Serial.print("pid_rot_kp: ");
    Serial.print(turning_pid.GetKp());
    Serial.print(" ");
    Serial.print("pid_rot_ki: ");
    Serial.print(turning_pid.GetKi());
    Serial.print(" ");
    Serial.print("pid_rot_kd: ");
    Serial.print(turning_pid.GetKd());
    Serial.println();
    
}
#endif

/**
 * @brief               Low level movement commands, allows for the motors to be rearranged
 *                      and all movement to be updated
 * 
 * @param direction     Direction to move the robot
 * @param speed         Speeed to spin the motors at 0-4095
 * @param acceleration  Currently unused, how fast to increase speed of motors
 * 
 * @details             Each motor has a defined forward and backward direction which can be configured
 *                      so that the wheels can be reliably turned in the correct direction
 * 
 *                      Each mechanum wheel creates a vector, by summing these vectors, the overall
 *                      movement of the robot can be determined.
 * 
 *                      When all wheels are moving forward, the horizontal component is zero,
 *                      When each side is moving opposite eachother (ie, each side has one forward rotating and one backwards)
 *                      then the robot can strafe
 */
void robot_move(ROBOT_DIR direction, uint16_t speed, uint16_t acceleration) {
    double speed_neg = speed;
    double speed_pos = speed;

    if (direction == RB_FORWARD) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
    } else if (direction == RB_BACKWARD) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_BACKWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_BACKWARD);
    } else if (direction == RB_STOP) {
        motorshield.getMotor(MOTOR_FL)->run(RELEASE);
        motorshield.getMotor(MOTOR_RR)->run(RELEASE);
        motorshield.getMotor(MOTOR_RL)->run(RELEASE);
        motorshield.getMotor(MOTOR_FR)->run(RELEASE);
    }

    if (direction == RB_FORWARD) {
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_FORWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_FORWARD);
    } else if (direction == RB_BACKWARD) {
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);
    }

    if (direction == RB_LEFT) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_BACKWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_BACKWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_FORWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_FORWARD);
    } else if (direction == RB_RIGHT) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);
    }
    
    // Set adjusted speeds and round to an integer 
    motorshield.getMotor(MOTOR_FL)->setSpeedFine( (uint16_t)round(abs(speed_neg*MOTOR_FL_CONSTANT)) );
    motorshield.getMotor(MOTOR_RL)->setSpeedFine( (uint16_t)round(abs(speed_pos*MOTOR_RL_CONSTANT)) );
    motorshield.getMotor(MOTOR_RR)->setSpeedFine( (uint16_t)round(abs(speed_neg*MOTOR_RR_CONSTANT)) );
    motorshield.getMotor(MOTOR_FR)->setSpeedFine( (uint16_t)round(abs(speed_pos*MOTOR_FR_CONSTANT)) );
}


/**
 * @brief               High level rotation, same as above but for rotation
 *                      Uses tank tread type rotation
 * 
 * @param direction     Turn CW or CCW
 * @param speed         Motor speed 0-4095
 * @param acceleration  Currently unused, how fast to change motor speed
 */
void robot_rotation(ROBOT_DIR direction, uint16_t speed, uint16_t acceleration) {

    if (direction == RB_TURN_CC) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_BACKWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_FORWARD);
    } else if (direction == RB_TURN_CW) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);
    } else if (direction == RB_STOP) {
        motorshield.getMotor(MOTOR_FL)->run(RELEASE);
        motorshield.getMotor(MOTOR_RL)->run(RELEASE);
        motorshield.getMotor(MOTOR_RR)->run(RELEASE);
        motorshield.getMotor(MOTOR_FR)->run(RELEASE);
    }
    

    motorshield.getMotor(MOTOR_FL)->setSpeedFine( round(speed * MOTOR_FL_CONSTANT) );
    motorshield.getMotor(MOTOR_RL)->setSpeedFine( round(speed * MOTOR_RL_CONSTANT) );
    motorshield.getMotor(MOTOR_RR)->setSpeedFine( round(speed * MOTOR_RR_CONSTANT) );
    motorshield.getMotor(MOTOR_FR)->setSpeedFine( round(speed * MOTOR_FR_CONSTANT) );
}

#ifdef USE_GYRO
/**
 * @brief               Use Gyro to turn a set number of degrees
 * 
 * @param direction     Turn CW or CCW
 * @param degrees       Number of degrees to turn
 * @param speed         How fast to spin motors
 * @param acceleration  Currently unused, how fast to change motor speed
 */
void robot_rotation_by_deg (ROBOT_DIR direction, uint16_t degrees, uint16_t speed, uint16_t acceleration) {
    uint16_t final_degrees = (round(get_rotation()) + degrees)%360;

    while(get_rotation()) {
        
    }

}
#endif


/**
 * @brief   Manual movement commands, allow for arbitrary strafing by varying the relative
 *          speed of the wheels.
 * 
 * @param x x component of direction vector
 * @param y y component of direction vector
 * 
 * @details While currently using a joystick, this will eventually be the basis for our 
 *          recentering while moving
 */
void manual_move(uint16_t x, uint16_t y) {
    double xc = (max_analog/(double)2.0) - x;
    double yc = (max_analog/(double)2.0) - y;

    if (abs(xc) < joystick_deadzone) {
        xc = 0;
    }

    if (abs(yc) < joystick_deadzone) {
        yc = 0;
    }


    // By taking the unit vector of the input, then solving against
    // the wheel vectors defined in the preamble, we are able to arbitrarily
    // strafe
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
    

    motorshield.getMotor(MOTOR_FL)->setSpeedFine( (uint16_t)round(abs(speed_neg*MOTOR_FL_CONSTANT)) );
    motorshield.getMotor(MOTOR_RL)->setSpeedFine( (uint16_t)round(abs(speed_pos*MOTOR_RL_CONSTANT)) );
    motorshield.getMotor(MOTOR_RR)->setSpeedFine( (uint16_t)round(abs(speed_neg*MOTOR_RR_CONSTANT)) );
    motorshield.getMotor(MOTOR_FR)->setSpeedFine( (uint16_t)round(abs(speed_pos*MOTOR_FR_CONSTANT)) );
}

/**
 * @brief   Mostly same as robot_rotate, but for manual control
 * 
 * @param x x component of input control
 */
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


