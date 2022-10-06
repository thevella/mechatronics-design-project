#include "movement.h"
#include "sensors.h"

#include <PID_v1.h>
#include <pid.h>


Adafruit_MotorShield motorshield = Adafruit_MotorShield();

BLA::ArrayMatrix<2, 1, double> xyc = {0,0};
BLA::ArrayMatrix<2, 1, double> speed_solved = {0, 0};
BLA::ArrayMatrix<2, 2, double> speed_u_vec = {0.70710678118655, -0.70710678118655, 0.70710678118655, 0.70710678118655};
auto speed_decomp = BLA::LUDecompose(speed_u_vec);

uint16_t max_speed = 4095/2;
int16_t cur_speed = 0;
uint16_t robot_acceleration = 20;

#define MOTOR_FR_CONSTANT (double)1.0
#define MOTOR_FL_CONSTANT (1.07/(double)1.0)
#define MOTOR_RL_CONSTANT (0.95/(double)1.0)
#define MOTOR_RR_CONSTANT (0.96/(double)1.0)


double pid_rot_input=80, pid_rot_output=50, pid_rot_setpoint=180;
double pid_rot_kp=5, pid_rot_ki=0.5, pid_rot_kd=2;

#define PID_LIM_MIN -max_speed /* Limit for PWM. */
#define PID_LIM_MAX max_speed /* Limit for PWM. */
#define DEADBAND 0.0f /* Off==0 */
epid_t pid_rot;

//Define Variables we'll be connecting to
float Input;
int Output = 0;
float deadband_delta;

PID turning_pid(&pid_rot_input, &pid_rot_output, &pid_rot_setpoint, pid_rot_kp, pid_rot_ki, pid_rot_kd, DIRECT);

void calibrate_center(CENTER_TYPE);
void recenter();

void setup_movement() {
    Serial.println("Adafruit Motorshield v2 - DC Motor test!");

    if (!motorshield.begin(1600, &Wire1)) {         // create with the default frequency 1.6KHz
        // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    
    Serial.println("Motor Shield found.");

    pid_rot_setpoint = add_degrees(get_rotation(), (float)-90);
    pid_rot_input = get_rotation();

    //Setup the pid 
    turning_pid.SetMode(AUTOMATIC);
    turning_pid.SetOutputLimits(PID_LIM_MIN, PID_LIM_MAX);


    epid_info_t pid_rot_err = epid_init(&pid_rot,
        pid_rot_input, pid_rot_input, -max_speed,
        pid_rot_kp, pid_rot_ki, pid_rot_kd);

    if (pid_rot_err != EPID_ERR_NONE) {
        Serial.print("\n\n** ERROR: epid_err != EPID_ERR_NONE **\n\n");
        while (1) { ; }
    }

    calibrate_center(CENTER_L_R);

    while(true) {
        recenter();
        delay(2000);
    }
}

void recenter() {
    int64_t left = dist_left.read_dist_wheels();
    int64_t right = dist_right.read_dist_wheels();
    if (abs(left - right) > 1000) {
        return;
    }

    char out1[50];

    sprintf(out1, "Difference: %i", dist_left.read_dist_wheels() - dist_right.read_dist_wheels());
    Serial.println(out1);

    while (abs(left - right) > 10) {
        left = dist_left.read_dist_wheels();
        right = dist_right.read_dist_wheels();
        Serial.println("need to move");
        if (left < right) {
            robot_move(RB_LEFT, max_speed/4);
            Serial.println("moving left");
        } else {
            robot_move(RB_RIGHT, max_speed/4);
            Serial.println("moving right");
        }
        delay(100);
    } 

    robot_move(RB_STOP);
}

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

void calibrate_center(CENTER_TYPE dir) {
    uint16_t temp[2];

    if (dir == CENTER_F_B) {
        // uint16_t old_dist_val = dist_right.raw_value();
        // robot_move(RB_FORWARD);

        // while (abs(dist_front.raw_value() - old_dist_val) > 20) {
        //     old_dist_val = dist_front.raw_value();
        //     delay(100);
        // }

        // robot_move(RB_STOP);

        // temp[0] = dist_front.raw_value();
        // temp[1] = dist_back.raw_value();

        // old_dist_val = dist_back.raw_value();
        
        // robot_move(RB_BACKWARD);

        // while (abs(dist_back.raw_value() - old_dist_val) > 2) {
        //     old_dist_val = dist_back.raw_value();
        //     delay(100);
        // }

        // robot_move(RB_STOP);

        // dist_front.calibrate(temp[0], dist_front.raw_value());

        // dist_back.calibrate(dist_back.raw_value(), temp[1]);

    } else if (dir == CENTER_L_R) {
        Serial.println("centering right");
        uint16_t old_dist_val = dist_right.raw_value();
        robot_move(RB_RIGHT);
        delay(2000);

        // while (abs(dist_right.raw_value() - old_dist_val) > 10) {
        //     old_dist_val = dist_right.raw_value();
        //     delay(500);
        // }

        robot_move(RB_STOP);

        delay(400);

        temp[0] = dist_right.raw_value();
        temp[1] = dist_left.raw_value();

        old_dist_val = dist_left.raw_value();
        
        Serial.println("centering left");
        robot_move(RB_LEFT);
        delay(2000);

        // while (abs(dist_left.raw_value() - old_dist_val) > 10) {
        //     old_dist_val = dist_left.raw_value();
        //     delay(500);
        // }

        robot_move(RB_STOP);

        dist_right.calibrate(temp[0], dist_right.raw_value());
        Serial.print(dist_left.raw_value());
        Serial.print(",");
        Serial.println(temp[1]);
        delay(2000);
        dist_left.calibrate(dist_left.raw_value(), temp[1]);
    }
}


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

void robot_move(ROBOT_DIR direction, uint16_t speed, uint16_t acceleration) {
    /*
    double xc = (max_analog/(double)2.0) - x;
    double yc = (max_analog/(double)2.0) - y;

    if (abs(xc) < joystick_deadzone) {
        xc = 0;
    }

    if (abs(yc) < joystick_deadzone) {
        yc = 0;
    }


    xyc = {xc, yc};

    speed_solved = BLA::LUSolve(speed_decomp, xyc);
    
    double speed_abs = sqrt((speed_solved(0)*speed_solved(0)) + (speed_solved(1)*speed_solved(1)));

    double speed_neg = (speed_solved(0)/speed_abs) * max_speed;
    double speed_pos = (speed_solved(1)/speed_abs) * max_speed;
    */
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
    

    motorshield.getMotor(MOTOR_FL)->setSpeedFine( (uint16_t)round(abs(speed_neg*MOTOR_FL_CONSTANT)) );
    motorshield.getMotor(MOTOR_RL)->setSpeedFine( (uint16_t)round(abs(speed_pos*MOTOR_RL_CONSTANT)) );
    motorshield.getMotor(MOTOR_RR)->setSpeedFine( (uint16_t)round(abs(speed_neg*MOTOR_RR_CONSTANT)) );
    motorshield.getMotor(MOTOR_FR)->setSpeedFine( (uint16_t)round(abs(speed_pos*MOTOR_FR_CONSTANT)) );
}

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

void robot_rotation_by_deg (ROBOT_DIR direction, uint16_t degrees, uint16_t speed, uint16_t acceleration) {
    uint16_t final_degrees = (round(get_rotation()) + degrees)%360;

    while(get_rotation()) {
        
    }

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


