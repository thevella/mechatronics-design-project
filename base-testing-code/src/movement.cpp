#include "movement.h"
#include "sensors.h"
#include <ArxContainer.h>
#include <math.h>


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

#ifdef USE_SCHEDULING
using namespace ace_routine;
#endif

//#define REVERSE_AUTO
//#define REVERSE

#if defined(REVERSE) && defined(REVERSE_AUTO)
#warning Overriding auto reverse
#undef REVERSE_AUTO
#endif

#define DO_BOTTOM
//#define DO_TOP
#define DO_ALL

#ifdef DO_ALL
    #ifndef DO_TOP
        #define DO_TOP
    #endif

    #ifndef DO_BOTTOM
        #define DO_BOTTOM
    #endif
#endif

// Commands, Structured using defines so that individual sections
// of the map can be done individually instead of having to do
// the whole run to test
// std::vector<std::vector<int>> commands {
//     #if defined(DO_TOP)
//     {T_FORWARD, 4}, {T_TURN_CW, 90}, {T_BACKWARD, 3}, {T_TURN_CW, 90}, {T_FORWARD, 3}, {T_TURN_CW, 90}, {T_FORWARD, 2}, {T_TURN_CW, 90}
//     #endif

//     #if defined(DO_TOP) && defined(DO_BOTTOM)
//     , {T_RAMP},
//     #endif

//     #if defined(DO_BOTTOM)
//     {T_FORWARD, 1}, {T_TURN_CW, 90}, {T_FORWARD, 2}, {T_TURN_CW, 90}, {T_FORWARD, 3}, {T_TURN_CCW, 90}, {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_FORWARD, 4}, {T_TURN_CW, 90}, 
//     {T_FORWARD, 1}, {T_TURN_CCW, 90}, /**/ {T_BACKWARD, 1}, {T_TURN_CW, 90}, {T_FORWARD, 1}, 
//     {T_STRAFE_R, 1}, {T_REVERSE, 350}, {T_FORWARD, 2}, {T_TURN_CW, 90}, {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_REVERSE, 350},
//     {T_GRAB_SAND}
//     #endif

//     #if defined(REVERSE) && defined(DO_BOTTOM)
//     , {T_TURN_CCW, 90}, {T_FORWARD, 1}, {T_STRAFE_R, 1}, {T_REVERSE, 450}, {T_FORWARD, 2}, {T_STRAFE_R, 1}, {T_REVERSE, 450}, {T_FORWARD, 1},
//     {T_TURN_CW, 1}, {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_FORWARD, 4}, {T_TURN_CW, 90}, {T_FORWARD, 2}, 
//     {T_TURN_CW, 90}, {T_FORWARD, 4}, {T_TURN_CCW, 90}, {T_FORWARD, 2}, {T_TURN_CCW, 90}
//     #endif

//     #if defined(DO_TOP) && defined(DO_BOTTOM) && defined(REVERSE)
//     , {T_RAMP, true}
//     #endif

//     #if defined(REVERSE) && defined(DO_TOP)
//     #ifdef DO_BOTTOM
//     , {T_TURN_CCW, 90}
//     #else
//     , {T_TURN_CW, 90}
//     #endif
//     , {T_FORWARD, 4}, {T_TURN_CCW, 90}, {T_FORWARD, 4}, {T_TURN_CW, 90}, {T_FORWARD, 4}, {T_TURN_CW, 90}, {T_FORWARD, 4}
//     #endif

//     };

std::vector<std::vector<int>> commands {
    #if defined(DO_TOP)
    {T_FORWARD, 4}, {T_TURN_CW, 90}, {T_FORWARD, 2}, {T_TURN_CW, 90}, {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_FORWARD, 1}, {T_TURN_CW, 90}, {T_FORWARD, 1},
    {T_TURN_CCW, 90}, {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_FORWARD, 2}, {T_TURN_CW, 90}
    #endif

    #if defined(DO_TOP) && defined(DO_BOTTOM)
    , {T_RAMP},
    #endif

    #if defined(DO_BOTTOM)
    {T_FORWARD, 1}, {T_TURN_CW, 90}, {T_FORWARD, 2}, {T_TURN_CW, 90}, {T_FORWARD, 3}, {T_TURN_CCW, 90}, {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_FORWARD, 4}, {T_TURN_CW, 90}, 
    {T_FORWARD, 1}, {T_TURN_CCW, 90}, /**/ {T_BACKWARD, 1}, {T_TURN_CW, 90}, {T_FORWARD, 1}, 
    {T_STRAFE_R, 1}, {T_REVERSE, 350}, {T_FORWARD, 2}, {T_TURN_CW, 90}, {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_REVERSE, 350},
    {T_GRAB_SAND}
    #endif

    #if defined(REVERSE) && defined(DO_BOTTOM)
    , {T_TURN_CCW, 90}, {T_FORWARD, 1}, {T_STRAFE_R, 1}, {T_REVERSE, 450}, {T_FORWARD, 2}, {T_STRAFE_R, 1}, {T_REVERSE, 450}, {T_FORWARD, 1},
    {T_TURN_CW, 1}, {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_FORWARD, 4}, {T_TURN_CW, 90}, {T_FORWARD, 2}, 
    {T_TURN_CW, 90}, {T_FORWARD, 4}, {T_TURN_CCW, 90}, {T_FORWARD, 2}, {T_TURN_CCW, 90}
    #endif

    #if defined(DO_TOP) && defined(DO_BOTTOM) && defined(REVERSE)
    , {T_RAMP, true}
    #endif

    #if defined(REVERSE) && defined(DO_TOP)
    #ifdef DO_BOTTOM
    , {T_TURN_CCW, 90}
    #else
    , {T_TURN_CW, 90}
    #endif
    , {T_FORWARD, 4}, {T_TURN_CCW, 90}, {T_FORWARD, 4}, {T_TURN_CW, 90}, {T_FORWARD, 4}, {T_TURN_CW, 90}, {T_FORWARD, 4}
    #endif

    };

float heading = 0.0;

/**
 * @brief Correct heading if gyro is available
 * 
 */
void correct_heading() {
#ifdef USE_GYRO
    float output = 0;
    float target = 0;
    ROBOT_DIR dir;

    // Get the current rotation
    output = get_rotation();

    // The target is the saved heading
    target = heading;

    do {
        // Turn towards the target value
        if (deg_difference(output, heading) < 0) {
            dir = RB_TURN_CC;
        } else {
            dir = RB_TURN_CW;
        }
        robot_rotation(dir, 1500);
        
        output = get_rotation();
    } while (abs(deg_difference(output, target)) > 2);

    heading = target;

    robot_move(RB_STOP);
#else
    // If no gyro available, do nothing
    return;
#endif
}

/**
 * @brief Strafe right or left correcting position as it moves
 * 
 * @param dir     direction to move
 * @param squares number of squares to move
 * @param offset  if an offset is needed for manual correction
 */
void task_move(ROBOT_DIR dir, int squares, int offset = 0) {
    int output = 0;
    int output2 = 0;
    int target = 0;

    // Read the sensor to get initial reading
    read_TOF_front(&output);

    bool move = false;
    
    // If we are pover 900 mm away, we do not read correctly,
    // move closer to a calibrated distance
    if (output > 900 && false) {
        while (abs(output - 865) > 3) {
            // Only move if one of the sensors is returning valid data
            move = move || read_TOF_left(&output2);
            if (move) {
                robot_move(dir, 0, output2 - MM_TO_SQUARES_LR_OFF);
            } else {
                robot_move(RB_STOP);
            }
            move = false;
            delay(25);
            move = read_TOF_front(&output);
        }

        --squares;
    }
    
    // If moving towards sensor, subtract distance, otherwise add it
    if (dir == RB_FORWARD) {
        target = output - (squares * MM_TO_SQUARES_F - (squares - 1) * MM_TO_SQUARES_F_CORR) - offset;
        if (target < MM_TO_SQUARES_FB_OFF) {
            target = MM_TO_SQUARES_FB_OFF;
        }
    } else if (dir == RB_BACKWARD) {
        target = output + (squares * MM_TO_SQUARES_B + (squares - 1) * MM_TO_SQUARES_B_CORR) + offset;
    }

    
    
    move = false;
    int correction = 0;
    if (squares != 0) {
        do {
            // Only move if one of the sensors returned valid data
            move = move || read_TOF_left(&output2);
            Serial.println(millis());
            if (move) {
                // If our correction value is valid, then use it,
                // otherwise we are on an open square, so don't
                // apply recentering
                if (output2 < 200) {
                    correction = output2 - MM_TO_SQUARES_LR_OFF;
                } else {
                    correction = 0;
                }

                // Move towards the target value
                if (output - target < 0 && dir == RB_FORWARD) {
                    robot_move(RB_BACKWARD, 0, correction);
                } else if (output - target > 0 && dir == RB_BACKWARD) {
                    robot_move(RB_FORWARD, 0, correction);
                } else {
                    robot_move(dir, 0, correction);
                }
                
            } else {
                robot_move(RB_STOP);
            }
            move = false;
            delay(25);
            move = read_TOF_front(&output);
        } while (abs(output - target) > 3);
    }
    
    // Stop robot
    robot_move(RB_STOP);

    // Correct heading if gyro available
    correct_heading();
}

/**
 * @brief Strafe right or left correcting position as it moves,
 *          almost identical to task_move
 * 
 * @param dir     direction to move
 * @param squares number of squares to move
 * @param offset  if an offset is needed for manual correction
 */
void task_strafe(ROBOT_DIR dir, int squares, int offset = 0) {
    int output = 0;
    int target = 0;
    int output2 = 0;

    // Read the sensor to get initial reading
    read_TOF_left(&output);

    bool move = false;
    
    // If we are pover 900 mm away, we do not read correctly,
    // move closer to a calibrated distance
    if (output > 900) {
        while (abs(output - 865) > 3) {
            // Only move if one of the sensors is returning valid data
            move = move || read_TOF_front(&output2);
            if (move) {
                robot_move(dir, 0, output2 - MM_TO_SQUARES_FB_OFF);
            }
            move = false;
            delay(25);
            move = read_TOF_left(&output);
        }

        --squares;
    }
    
    // If moving towards sensor, subtract distance, otherwise add it
    if (dir == RB_LEFT) {
        target = output - (squares * MM_TO_SQUARES_L + (squares - 1) * MM_TO_SQUARES_L_CORR) - offset;
        // If we are less than the minimum, move to the value used for centering
        if (target < MM_TO_SQUARES_LR_OFF) {
            target = MM_TO_SQUARES_LR_OFF;
        }
    } else if (dir == RB_RIGHT) {
        target = output + (squares * MM_TO_SQUARES_R + (squares - 1) * MM_TO_SQUARES_R_CORR) + offset;
    }
    
    move = false;
    int correction = 0;
    // Only move if we are to move more than one square
    if (squares != 0) {
        do {
            // Only move if one of the sensors returned valid data
            move = move || read_TOF_front(&output2);
            if (move) {
                // If our correction value is valid, then use it,
                // otherwise we are on an open square, so don't
                // apply recentering
                if (output2 < 200) {
                    correction = output2 - MM_TO_SQUARES_LR_OFF;
                } else {
                    correction = 0;
                }

                // Move towards the target value
                if (output - target < 0 && dir == RB_LEFT) {
                    robot_move(RB_RIGHT, 0, correction);
                } else if (output - target > 0 && dir == RB_RIGHT) {
                    robot_move(RB_LEFT, 0, correction);
                } else {
                    robot_move(dir, 0, correction);
                }
            }
            move = false;
            delay(25);
            move = read_TOF_left(&output);
        } while (abs(output - target) > 3);
    }
    
    // Stop robot
    robot_move(RB_STOP);
    
    // Correct heading if gyro available
    correct_heading();
}

/**
 * @brief Turn robot using gyro if available
 * 
 * @param dir Turn clockwise of counter-clockwise
 * @param deg Number of degrees to turn, only used when using gyro
 */
void task_rotate(ROBOT_DIR dir, double deg) {

#ifdef USE_GYRO
    float output = get_rotation();

    if (dir == RB_TURN_CW) {
        heading = add_degrees(heading, deg);
    } else if (dir == RB_TURN_CC) {
        heading = add_degrees(heading, -deg);
    }

    while (abs(deg_difference(output, heading)) > 3) {
        if (deg_difference(output, heading) < 0) {
            robot_rotation(RB_TURN_CC, 4096 / 2);
        } else {
            robot_rotation(RB_TURN_CW, 4096 / 2);
        }
        
        delay(50);
        output = get_rotation();
    } 

#else
    robot_rotation(dir);

    delay(700);

    robot_move(RB_STOP);

    correct_heading();
#endif
}

/**
 * @brief Task to grab sand since the dist sensor is occluded by the
 *          gripper mechanism when open
 * 
 */
void task_grab_sand() {
    claw_servo.write(0);
    claw_servo.attach(3);

    robot_move(RB_FORWARD);
    delay(1500);
    robot_move(RB_STOP);

    delay(500);

    claw_servo.write(120);

    delay(500);

    robot_move(RB_BACKWARD);
    delay(1200);
    robot_move(RB_STOP);

    claw_servo.detach();

}   

/**
 * @brief Do the ramp as a manual function, since looking over the edge could
 *          return invalid data
 * 
 * @param reverse Whether we are traveling up the ramp
 */
void task_ramp(bool reverse = false) {
    // Reverse turing dir and correction side
    // if going up
    ROBOT_DIR turn_dir = RB_TURN_CW;
    ROBOT_DIR wall_dir = RB_LEFT;
    ROBOT_DIR fix_dir = RB_RIGHT;

    if (reverse) {
        turn_dir = RB_TURN_CC;
        wall_dir = RB_RIGHT;
        fix_dir = RB_LEFT;
    }

    robot_move(wall_dir);
    delay(200);
    robot_move(RB_STOP);

    delay(50);

    robot_move(RB_FORWARD);
    delay(2000);
    robot_move(RB_STOP);
    
    delay(50);

    task_move(RB_FORWARD, 15);

    task_rotate(turn_dir, 90);

    robot_move(RB_FORWARD);
    delay(2000);
    robot_move(RB_STOP);

    task_move(RB_FORWARD, 15);

    task_rotate(turn_dir, 90);

    task_move(RB_FORWARD, 15);

    delay(50);

    robot_move(wall_dir);
    delay(200);
    robot_move(RB_STOP);

    delay(300);

    robot_move(fix_dir);
    delay(200);
    robot_move(RB_STOP);
}

/**
 * @brief Iterate through tasks and operate
 * 
 */
void navigate_maze() {
    int offset = 0;
    // Iterate through tasks
    for (auto command: commands) {
        if (command.size() > 2) {
            offset = command.at(2);
        } else {
            offset = 0;
        }
        switch(command.at(0)) {
            case(T_FORWARD):
                task_move(RB_FORWARD, command.at(1), offset);
                break;
            case(T_BACKWARD):
                task_move(RB_BACKWARD, command.at(1), offset);
                break;
            case(T_STRAFE_L):
                task_strafe(RB_LEFT, command.at(1), offset);
                break;
            case(T_STRAFE_R):
                task_strafe(RB_RIGHT, command.at(1), offset);
                break;
            case(T_TURN_CW):
                task_rotate(RB_TURN_CW, command.at(1));
                break;
            case(T_TURN_CCW):
                task_rotate(RB_TURN_CC, command.at(1));
                break;
            case(T_GRAB_SAND):
                task_grab_sand();
                break;
            case(T_RAMP):
                if (command.size() > 1) {
                    task_ramp(command.at(1));
                } else{
                    task_ramp();
                }
                
                break;
            case(T_REVERSE):
                robot_move(RB_BACKWARD);
                delay(command.at(1));
                robot_move(RB_STOP);
                break;
            default:
                break;
        }
        delay(50);
    }

    #ifdef REVERSE_AUTO
    auto command = commands.at(commands.size() - 1);
    switch(command.at(0)) {
        case(T_FORWARD):
            task_move(RB_FORWARD, command.at(1), offset);
            break;
        case(T_BACKWARD):
            task_move(RB_BACKWARD, command.at(1), offset);
            break;
        case(T_STRAFE_L):
            task_strafe(RB_LEFT, command.at(1), offset);
            break;
        case(T_STRAFE_R):
            task_strafe(RB_RIGHT, command.at(1), offset);
            break;
        case(T_TURN_CCW):
            task_rotate(RB_TURN_CC, command.at(1));
            break;
        case(T_TURN_CW):
            task_rotate(RB_TURN_CW, command.at(1));
            break;
        case(T_GRAB_SAND):
            break;
        case(T_RAMP):
            task_ramp();
            break;
        default:
            break;
    }    

    for (int i = commands.size()-2; i >= 0; --i) {
        command = commands.at(i);
        if (command.size() > 2) {
            offset = command.at(2);
        } else {
            offset = 0;
        }
        switch(command.at(0)) {
            case(T_FORWARD):
                task_move(RB_FORWARD, command.at(1), offset);
                break;
            case(T_BACKWARD):
                task_move(RB_BACKWARD, command.at(1), offset);
                break;
            case(T_STRAFE_L):
                task_strafe(RB_LEFT, command.at(1), offset);
                break;
            case(T_STRAFE_R):
                task_strafe(RB_RIGHT, command.at(1), offset);
                break;
            case(T_TURN_CW):
                task_rotate(RB_TURN_CC, command.at(1));
                break;
            case(T_TURN_CCW):
                task_rotate(RB_TURN_CW, command.at(1));
                break;
            case(T_GRAB_SAND):
                break;
            case(T_RAMP):
                task_ramp();
                break;
            default:
                break;
        }
        delay(50);
    }
    #endif
    
}

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

    // heading = get_rotation();

#ifdef USE_IR
    // Setup the center position and calibrate side sensors
    // for distance
    calibrate_center(CENTER_L_R);
#endif
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
                #ifdef USE_IR
                recenter();
                #endif
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
            #ifdef USE_IR
            recenter();
            #endif
        }
    }
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

#ifdef USE_IR

/**
 * @brief         Move forward until sensor threshold is reached
 *
 * @param offset  Sensor offset threshold (ms)
 * @param speed   Speed at which to turn wheels 0-4095
 */
void forward_sense(int offset, int speed)
{
    robot_move(RB_FORWARD, 0, 0, speed);

    while (dist_front.raw_value() < offset)
    {
        delay(10);
    }

    robot_move(RB_STOP);

    delay(motor_stop_delay);
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
            robot_move(RB_LEFT, 0, 0, max_speed/3);
            Serial.println("moving left");
        } else {
            robot_move(RB_RIGHT, 0, 0, max_speed/3);
            Serial.println("moving right");
        }
        delay(100);
    } 

    robot_move(RB_STOP);

    delay(motor_stop_delay);
}


/**
 * @brief      Calibrate center by ramming the walls
 *
 * @param dir
 *
 * @details    Takes readings at the two known distances:
 *              When the sensor is against the wall at almost 4cm,
 *              and when the sensor is far from the wall
 */
void calibrate_center(CENTER_TYPE dir)
{
    uint16_t temp[2];

    // Will eventually center forward and backwards
    // so strafing is more possible
    if (dir == CENTER_L_R)
    {
        // hit the right wall and take readings
        int delay_val = 1600;
        Serial.println("centering right");
        uint16_t old_dist_val = dist_right.raw_value();
        robot_move(RB_RIGHT, 0, 0, max_speed / 1.5);
        delay(delay_val);

        robot_move(RB_STOP);

        delay(motor_stop_delay);

        temp[0] = dist_right.raw_value();
        temp[1] = dist_left.raw_value();

        old_dist_val = dist_left.raw_value();

        // hit left wall and take readings
        Serial.println("centering left");
        robot_move(RB_LEFT, 0, 0, max_speed / 1.5);
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
void robot_move(ROBOT_DIR direction, float heading_correction, int16_t placement_correction, uint16_t speed) {

    if (abs(heading_correction) < 3) {
        heading_correction = 0;
    }

    double xc = 0;
    double yc = 0;

    float head_corr[4] = {0,0,0,0};

    for (int i = 0; i < 4; ++i) {
        head_corr[i] = heading_correction*2;
        if (i == MOTOR_FR-1 || i == MOTOR_RR-1) {
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

        head_corr[MOTOR_FL-1] *= -1;
        head_corr[MOTOR_RR-1] *= -1;
    } else if (direction == RB_RIGHT) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);

        yc = max_speed;

        head_corr[MOTOR_FR-1] *= -1;
        head_corr[MOTOR_RL-1] *= -1;
    }


    // By taking the unit vector of the input, then solving against
    // the wheel vectors defined in the preamble, we are able to arbitrarily
    // strafe

    // We can apply a rotation vector to based on strafing factor, so we can move while strafing
    xyc = {xc, yc};

    double theta = -1 * placement_correction * PI/((double)180.0);

    if (direction == RB_LEFT || direction == RB_BACKWARD) {
        theta *= -1;
    }

    rotation = {cos(theta), -sin(theta), sin(theta), cos(theta)};

    xyc = rotation * xyc;

    speed_solved = BLA::LUSolve(speed_decomp, xyc);
    
    double speed_mag = sqrt(pow(speed_solved(0), 2) + pow(speed_solved(1), 2));

    double speed_neg = (speed_solved(0)/speed_mag) * speed;
    double speed_pos = (speed_solved(1)/speed_mag) * speed;

    // Scale speeds to the max speed, otherwise can be going 
    // slower than max
    if (speed_neg < speed_pos) {
        speed_neg *= speed / speed_pos;
        speed_pos = speed;
    } else {
        speed_neg *= speed / speed_pos;
        speed_pos = speed;
    }

    
    head_corr[MOTOR_FL-1] = 0;
    head_corr[MOTOR_RR-1] = 0;

    for (int i = 0; i < 4; ++i) {
        if (head_corr[i] > 0) {
            head_corr[i] = 0;
        }
    }

    motorshield.getMotor(MOTOR_FL)->setSpeedFine( (uint16_t)round((abs(speed_neg)+head_corr[MOTOR_FL-1])*MOTOR_FL_CONSTANT) );
    motorshield.getMotor(MOTOR_RL)->setSpeedFine( (uint16_t)round((abs(speed_pos)+head_corr[MOTOR_RL-1])*MOTOR_RL_CONSTANT) );
    motorshield.getMotor(MOTOR_RR)->setSpeedFine( (uint16_t)round((abs(speed_neg)+head_corr[MOTOR_RR-1])*MOTOR_RR_CONSTANT) );
    motorshield.getMotor(MOTOR_FR)->setSpeedFine( (uint16_t)round((abs(speed_pos)+head_corr[MOTOR_FR-1])*MOTOR_FR_CONSTANT) );
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


