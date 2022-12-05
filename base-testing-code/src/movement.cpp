#include "movement.h"
#include "sensors.h"
#include <ArxContainer.h>
#include <math.h>


// How long to let the motors settle to ensure robot has stopped
int motor_stop_delay = 250;

// Initialize motors and servos
Servo sand_servo;
Servo arm_servo;
Servo claw_servo;

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
#define REVERSE

#if defined(REVERSE) && defined(REVERSE_AUTO)
#warning Overriding auto reverse
#undef REVERSE_AUTO
#endif

//#define DO_BOTTOM
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

std::vector<std::vector<int>> commands {
    #if defined(DO_TOP)
    {T_FORWARD, 1}, {T_TURN_CCW, 90}, {T_OBSTACLE}, {T_FORWARD, 6}, {T_TURN_CW, 90+45}
    #endif


    #if defined(DO_TOP) && defined(DO_BOTTOM)
    , {T_RAMP}
    #endif

    #if defined(DO_BOTTOM)
    , {T_STRAFE_R, 1}, {T_FORWARD, 1}, {T_TURN_CW, 90}, {T_FORWARD, 6}, {T_TURN_CW, 90}, {T_GRAB_SAND}
    #endif

    #if defined(REVERSE) && defined(DO_BOTTOM)
    , {T_TURN_CW, 90}, {T_FORWARD, 6}, {T_STRAFE_L, 1}, {T_FORWARD, 1}, {T_TURN_CCW, 90}
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
    , {T_FORWARD, 6}, {T_STRAFE_L, 1}
    #endif

    };

double heading = 0.0;
double flat_inclination = 0.0;


void task_rotate(ROBOT_DIR, double);
/**
 * @brief Correct heading if gyro is available
 * 
 */
void correct_heading() {
#ifdef USE_GYRO
    task_rotate(RB_TURN_CC, 0);
#else
    // If no gyro available, do nothing
    return;
#endif
}



/**
 * @brief Move along any ordanal direction, correcting as it moves
 * 
 * @param dir          Direction to move
 * @param squares      Number of squares to move
 * @param offset       If an offset is needed for manual correction
 * @param centering    Whether to correct position left and right
 * @param fix_rotation Whether to correct heading
 * @param until_flat   Whether to move until the ground is level again
 * @param obstacle     Whether to move until an obstacle is detected in the gripper
 * @param low_limit    The lower bound of rotation correction
 * @param high_limit   The higher bound of rotation correction
 */
void task_move(ROBOT_DIR dir, int squares, int offset = 0, bool centering = true, bool fix_rotation = true, bool until_flat = false, bool obstacle = false, int low_limit = -45, int high_limit = 45) {
    // Reading in the direction of movement, Forward for forward/backward movement, and Left for left/right movement
    int dir_reading = 0;
    // Reading towards the wall based on direction of movement, alternate to dir_reading
    int wall_reading = 0;
    // What value to compare wall reading to, optimal position off wall
    int wall_compare = MM_TO_SQUARES_LR_OFF;
    // The value we are moving towards in the direction of travel
    int target = 0;

    // The two functions to get the correct readings, reduces number of if statements
    // needed in main loop
    bool (*read_dir_TOF)(int*, int*) = &read_TOF_front;
    bool (*read_wall_TOF)(int*, int*) = &read_TOF_left;

    // Set to values needed for strafing
    if (dir == RB_LEFT || dir == RB_RIGHT) {
        read_dir_TOF = &read_TOF_left;
        read_wall_TOF = &read_TOF_front;
        wall_compare = MM_TO_SQUARES_FB_OFF;
    }


    // Read the sensor to get initial reading
    read_dir_TOF(&dir_reading, nullptr);

    
    // Setup directions for moving towards the target
    // for_dir is used when value being read is lower than target
    // back_dir is used when value being read is higher than target
    ROBOT_DIR for_dir = RB_FORWARD;
    ROBOT_DIR back_dir = RB_BACKWARD;

    if (dir == RB_RIGHT || dir == RB_LEFT) {
        for_dir = RB_LEFT;
        back_dir = RB_RIGHT;
    }

    // If moving towards sensor, subtract distance, otherwise add it
    if (dir == RB_FORWARD || dir == RB_LEFT) {
        int corr = MM_TO_SQUARES_F_CORR;
        int mm_to_sq = MM_TO_SQUARES_F;
        int off = MM_TO_SQUARES_FB_OFF;

        if (dir == RB_LEFT) {
            corr = MM_TO_SQUARES_L_CORR;
            mm_to_sq = MM_TO_SQUARES_L;
            off = MM_TO_SQUARES_LR_OFF;
        }

        target = dir_reading - (squares * mm_to_sq - (squares - 1) * corr) - offset;

        // If target is too low, or special value is given, set target to
        // ideal distance from wall
        if (target < mm_to_sq || squares == -1) {
            target = off;
        }
    } else if (dir == RB_BACKWARD || dir == RB_RIGHT) {
        int corr = MM_TO_SQUARES_B_CORR;
        int mm_to_sq = MM_TO_SQUARES_B;
        int off = MM_TO_SQUARES_FB_OFF;

        if (dir == RB_RIGHT) {
            corr = MM_TO_SQUARES_R_CORR;
            mm_to_sq = MM_TO_SQUARES_R;
            off = MM_TO_SQUARES_LR_OFF;
        }

        target = dir_reading + (squares * mm_to_sq + (squares - 1) * corr) + offset;
    }

    

    
    // Stores sensor state
    bool move = false;
    // Correction for positioning from wall
    int correction = 0;

    // Speed value, so that we can slow down as we approach 
    // target value
    uint16_t speed = max_speed;

    // How much to correct based on rotation
    int rotation_corr = 0;

    // Target value when not using distance for targeting
    int alt_target = 0;

    if (until_flat) {
        alt_target = get_inclination();
    } else if (obstacle) {
        alt_target = dist_block.raw_value();
    }

    if (squares != 0) {
        do {
            // Only move if one of the sensors returned valid data
            if (centering) {
                move = move || read_wall_TOF(&wall_reading, nullptr);
            }
            
            if (move) {
                // If our correction value is valid, then use it,
                // otherwise we are on an open square, so don't
                // apply recentering
                if (wall_reading < 200 && centering) {
                    correction = (wall_reading - wall_compare)*2;

                    // Bound value, allows for limiting how much power is removed from wheels
                    if (correction > high_limit) {
                        correction = high_limit;
                    } else if (correction < low_limit) {
                        correction = low_limit;
                    } 
                } else {
                    correction = 0;
                }

                // Slow down if less than 100mm from target value, scaled depending on how close
                // On the ramp and when moving towards an obstacle, we dont get closer, so ignore
                if (abs(dir_reading - target) < 100 && !until_flat && !obstacle) {
                    speed = max_speed - ((100 - abs(dir_reading - target))*20);
                } else {
                    speed = max_speed;
                }

                
                if (fix_rotation) {
                    rotation_corr = deg_difference(get_rotation(), heading)*80;
                }

                // Move towards the target value
                if (dir_reading - target < 0) {
                    robot_move(back_dir, rotation_corr, correction, speed);
                } else {
                    robot_move(for_dir, rotation_corr, correction, speed);
                }
                
            }
            
            delay(25);
            if (until_flat) {
                move = true;
                alt_target = get_inclination();
                if (abs(alt_target - flat_inclination) < 2) {
                    break;
                }
            } else if (obstacle) {
                move = true;
                alt_target = dist_block.raw_value();
                if (alt_target > 2700) {
                    break;
                }
            } else {
                move = false;
                move = read_dir_TOF(&dir_reading, nullptr);
            }
            
        } while (abs(dir_reading - target) > 1);
    }
    
    // Stop robot
    robot_move(RB_STOP);
}

/**
 * @brief Prepare for obstacle by raising arm, then move towards it
 * 
 */
void task_obstacle() {
    // Open claw, then lower arm
    claw_servo.write(180);
    claw_servo.attach(11, 750, 2250);
    claw_servo.write(180);

    delay(100);

    arm_servo.write(10);
    arm_servo.attach(12);
    arm_servo.write(10);

    delay(600);

    // Move until obstacle detected
    task_move(RB_BACKWARD, 6, 0, false, true, false, true);

    // Close claw, then raise arm again
    delay(300);

    claw_servo.write(0);

    delay(600);

    arm_servo.write(180);

    delay(300);

    // Rotate so that we are facing forwards
    task_rotate(RB_TURN_CC, 180);

}

/**
 * @brief Turn robot using gyro if available
 * 
 * @param dir Turn clockwise of counter-clockwise
 * @param deg Number of degrees to turn, only used when using gyro
 */
void task_rotate(ROBOT_DIR dir, double deg) {

#ifdef USE_GYRO
    double output = get_rotation();

    // Use global heading so we dont accumulate error
    if (dir == RB_TURN_CW) {
        heading = add_degrees(heading, deg);
    } else if (dir == RB_TURN_CC) {
        heading = add_degrees(heading, -deg);
    }

    // speed so we can slow as we approach target
    int speed = max_speed;
    // Difference from where we are to where we are targeting
    double deg_diff = deg_difference(output, heading);

    while (abs(deg_diff) > 1) {
        // Slow down as we approach target, based on proximity
        if (abs(deg_diff) < 20) {
            speed = max_speed/2 - ((20-abs(deg_diff))*84);
        }

        // Rotate towards target
        if (deg_diff < 0) {
            robot_rotation(RB_TURN_CC, speed);
        } else {
            robot_rotation(RB_TURN_CW, speed);
        }
        
        delay(25);
        output = get_rotation();
        deg_diff = deg_difference(output, heading);
    } 

    robot_move(RB_STOP);

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
    sand_servo.write(0);
    sand_servo.attach(3);
    

    robot_move(RB_FORWARD);
    delay(900);
    robot_move(RB_STOP);

    delay(500);

    sand_servo.write(100);

    delay(500);

    robot_move(RB_BACKWARD);
    delay(900);
    robot_move(RB_STOP);

    sand_servo.detach();

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

    if (!reverse) {
        // Lower arm to dump off side
        claw_servo.attach(11, 750, 2250);
        
        claw_servo.write(90);

        arm_servo.write(140);
        arm_servo.attach(12);
        arm_servo.write(140);

        delay(300);

        arm_servo.write(180);

        delay(300);

        arm_servo.detach();
        claw_servo.detach();

        // Rotate to go down ramp
        task_rotate(RB_TURN_CC, 45);       
    }

    // Begin moving then lower arm, then move towards far wall
    robot_move(RB_FORWARD);
    if (!reverse) {
        delay(300);
        arm_servo.write(0);
        arm_servo.attach(12);
        arm_servo.write(0);
    }
    delay(2700);
    task_move(RB_FORWARD, -1);


    arm_servo.write(180);

    if (reverse) {
        arm_servo.attach(12);
        arm_servo.write(180);
    }

    delay(300);
    
    // same as above, but if going up the ramp, then we need to move backwards and limit the amount
    // that we correct, otherwise we dont have the power since only two wheels end up driving
    if (reverse) {
        task_rotate(RB_TURN_CW, 90);

        robot_move(RB_BACKWARD);
        delay(300);
        arm_servo.write(0);
        delay(2700);
        task_move(RB_BACKWARD, 15, 0, true, true, true, false, -20, 20);
        arm_servo.write(180);
        delay(300);
        
        task_rotate(turn_dir, 180);

        delay(300);

        task_move(RB_FORWARD, -1);

        task_rotate(RB_TURN_CW, 90);



        robot_move(RB_BACKWARD);
        delay(300);
        arm_servo.write(0);
        delay(2700);
        task_move(RB_BACKWARD, 15, 0, true, true, true, false, -20, 20);
        arm_servo.write(180);
        delay(300);
        
        task_rotate(turn_dir, 180);

        task_move(RB_FORWARD, -1);

    } else {
        task_rotate(turn_dir, 90);

        robot_move(RB_FORWARD);
        delay(300);
        arm_servo.write(0);
        delay(2700);
        task_move(RB_FORWARD, -1, 0, true, true, false, false, -20, 20);
        arm_servo.write(180);

        delay(300);
        
        task_rotate(turn_dir, 90);

        robot_move(RB_FORWARD);
        delay(2700);
        task_move(RB_FORWARD, -1);

        task_move(RB_BACKWARD, 1);
    }

    robot_move(RB_STOP);

    arm_servo.detach();
    
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
                task_move(RB_LEFT, command.at(1), offset);
                break;
            case(T_STRAFE_R):
                task_move(RB_RIGHT, command.at(1), offset);
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
            case(T_OBSTACLE):
                task_obstacle();
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
            task_move(RB_LEFT, command.at(1), offset);
            break;
        case(T_STRAFE_R):
            task_move(RB_RIGHT, command.at(1), offset);
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
                task_move(RB_LEFT, command.at(1), offset);
                break;
            case(T_STRAFE_R):
                task_move(RB_RIGHT, command.at(1), offset);
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

// Turn off all motors
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

    heading = get_rotation();
    flat_inclination = get_inclination();

}


/**
 * @brief                       Low level movement commands, allows for the motors to be rearranged
 *                              and all movement to be updated
 *
 * @param direction             Direction to move the robot
 * @param heading_correction    How much correction is needed for rotation
 * @param placement_correction  How much correction is needed for location towards wall
 * @param speed                 Speeed to spin the motors at 0-4095
 * 
 * @details                     Each motor has a defined forward and backward direction which can be configured
 *                              so that the wheels can be reliably turned in the correct direction
 *
 *                              Each mechanum wheel creates a vector, by summing these vectors, the overall
 *                              movement of the robot can be determined.
 *
 *                              When all wheels are moving forward, the horizontal component is zero,
 *                              When each side is moving opposite eachother (ie, each side has one forward rotating and one backwards)
 *                              then the robot can strafe
 */
void robot_move(ROBOT_DIR direction, double heading_correction, int16_t placement_correction, uint16_t speed) {

    if (abs(heading_correction) < 3) {
        heading_correction = 0;
    }

    double xc = 0;
    double yc = 0;

    // Setup heading correction, instead of removing
    // from one side and adding to the other, remove
    // twice as much from the one side. This is to make sure
    // we dont go over the max_speed
    double head_corr[4] = {0,0,0,0};

    for (int i = 0; i < 4; ++i) {
        head_corr[i] = heading_correction*2;
        if (i == MOTOR_FR-1 || i == MOTOR_RR-1) {
            head_corr[i] *= -1;
        }
    }

    // Setup direction and change heading_corr as needed
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

    double theta = placement_correction * PI/((double)180.0);

    if (direction == RB_RIGHT || direction == RB_FORWARD) {
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
    if (abs(speed_neg) < abs(speed_pos)) {
        speed_neg = speed_neg * (speed / abs(speed_pos));
        speed_pos = speed;
    } else {
        speed_pos = speed_pos * (speed / abs(speed_neg));
        speed_neg = speed;
    }

    // Only apply correction to one wheel,
    // more predictable rotation
    head_corr[MOTOR_FL-1] = 0;
    head_corr[MOTOR_RR-1] = 0;

    // Only apply correction when removing speed
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


