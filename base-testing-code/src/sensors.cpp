#include "sensors.h"
#define reverse_sensor

#define DIST_LEFT  0
#define DIST_RIGHT 1
#define DIST_FRONT 2
#define DIST_BACK  3

#if MOTOR_FR_FORWARD == FORWARD
const uint8_t MOTOR_FR_ENC_PINS[] = {36, 38};
#else
const uint8_t MOTOR_FR_ENC_PINS[] = {38, 36};
#endif

#if MOTOR_FL_FORWARD == FORWARD
const uint8_t MOTOR_FL_ENC_PINS[] = {44, 46};
#else
const uint8_t MOTOR_FL_ENC_PINS[] = {46, 44};
#endif

#if MOTOR_RL_FORWARD == FORWARD
const uint8_t MOTOR_RL_ENC_PINS[] = {34, 32};
#else
const uint8_t MOTOR_RL_ENC_PINS[] = {32, 34};
#endif

#if MOTOR_RR_FORWARD == FORWARD
const uint8_t MOTOR_RR_ENC_PINS[] = {42, 40};
#else
const uint8_t MOTOR_RR_ENC_PINS[] = {40, 42};
#endif

const uint8_t MOTOR_ENC_PINS[4][2] = {*MOTOR_FR_ENC_PINS, *MOTOR_FL_ENC_PINS, *MOTOR_RL_ENC_PINS, *MOTOR_RR_ENC_PINS};
volatile struct encoders enc = {0, 0, 0, 0};

// Distance Sensors
const uint8_t distance_sensors[][2] = {{A0, 13}, {A1, 14}};
const uint8_t num_dist_sensors = 2;
uint16_t distance_sensor_cal_values[][4] = {{0,0,0,0}, {0,0,0,0}};

uint16_t robot_width_mm = 155;
uint16_t wall_to_wall_width_mm = 175+10*2-3;

uint16_t distance_mm_vals[][2] = {{43, wall_to_wall_width_mm-robot_width_mm+43}, {45, wall_to_wall_width_mm-robot_width_mm+45}};

dist_sensor dist_left(DIST_LEFT);
dist_sensor dist_right(DIST_RIGHT);
dist_sensor dist_front(DIST_FRONT);
dist_sensor dist_back(DIST_BACK);


const uint16_t joystick_deadzone = 100;
const uint16_t max_analog = 4095;

MPU9250_DMP imu;


void setup_sensors() {
    for (int i = 0; i < num_dist_sensors; ++i) {
        pinMode(distance_sensors[i][DIST_VAL], INPUT);
        pinMode(distance_sensors[i][DIST_GPIO], OUTPUT);
        digitalWrite(distance_sensors[i][DIST_GPIO], HIGH);
    }
    // Startup all pins and UART

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; i < 2; ++i) {
            pinMode(MOTOR_ENC_PINS[i][j], INPUT);
        }
    }

    attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENC_PINS[0]), MOTOR_FR_ENC_0, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENC_PINS[0]), MOTOR_FL_ENC_0, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENC_PINS[0]), MOTOR_RL_ENC_0, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENC_PINS[0]), MOTOR_RR_ENC_0, RISING);

    if (imu.begin() != INV_SUCCESS) {
        while (true) {
            Serial.println("Unable to communicate with MPU-9250");
            Serial.println("Check connections, and try again.");
            Serial.println();
            delay(5000);
        }
    }
    

    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               10); // Set DMP FIFO rate to 10 Hz
    
    delay(10000);
    for (int i = 0; i < 10; ++i) {
        get_rotation();
    }


}



dist_sensor::dist_sensor(uint8_t sensor) {
    this->val_pin = distance_sensors[sensor][DIST_VAL];
    this->gpio_pin = distance_sensors[sensor][DIST_GPIO];

    for (int i = 0; i < 2; ++i) {
        this->cal_distances_mm[i] = distance_mm_vals[sensor][i];
        this->cal_values[i] = 0;
    }
}

void dist_sensor::calibrate(uint16_t val1, uint16_t val2){
    this->cal_values[0] = val1;
    this->cal_values[1] = val2;

    this->b = (val2 * this->cal_distances_mm[1] - val1 * this->cal_distances_mm[0]) / (this->cal_distances_mm[1] - this->cal_distances_mm[0]);
    this->a = (val1 - this->b) * cal_distances_mm[0];
}

float get_rotation() {
    update_gyro();
    return -imu.yaw;
}

float add_degrees(float degrees, float addition) {
    float new_degrees = degrees + addition;
    int8_t sign = -1;

    if (new_degrees >= 0) {
        sign = 1;
    }
    
    new_degrees = ((int)round(abs(new_degrees)))%360;

    if (new_degrees > 180) {
        return (360 - new_degrees)*sign;
    }

    return new_degrees*sign;
}

void printIMUData(void)
{  
    // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
    // are all updated.
    // Quaternion values are, by default, stored in Q30 long
    // format. calcQuat turns them into a float between -1 and 1
    float q0 = imu.calcQuat(imu.qw);
    float q1 = imu.calcQuat(imu.qx);
    float q2 = imu.calcQuat(imu.qy);
    float q3 = imu.calcQuat(imu.qz);

    Serial.println("Q: " + String(q0, 4) + ", " +
                    String(q1, 4) + ", " + String(q2, 4) + 
                    ", " + String(q3, 4));

    Serial.println("R/P/Y: " + String(imu.roll) + ", "
            + String(imu.pitch) + ", " + String(imu.yaw));

    Serial.println("Time: " + String(imu.time) + " ms");

    Serial.println();
}

bool update_gyro() {
    // Check for new data in the FIFO
    if ( imu.fifoAvailable() ) {
        
        // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
        if ( imu.dmpUpdateFifo() == INV_SUCCESS){
            // computeEulerAngles can be used -- after updating the
            // quaternion values -- to estimate roll, pitch, and yaw
            imu.computeEulerAngles(true);
            printIMUData();
            return true;
        } 
    }
    return false;
}

void MOTOR_FR_ENC_0() {
    //check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_FR_ENC_PINS[1])->PIO_PDSR & digitalPinToBitMask(MOTOR_FR_ENC_PINS[1])) == LOW) {
        enc.FR += 1;
    } else {
        enc.FR -= 1;
    }
}

void MOTOR_FR_ENC_1() {//check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_FR_ENC_PINS[0])->PIO_PDSR & digitalPinToBitMask(MOTOR_FR_ENC_PINS[0])) == LOW) {
        enc.FR += 1;
    } else {
        enc.FR -= 1;
    }
}

void MOTOR_FL_ENC_0() {
    //check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_FL_ENC_PINS[1])->PIO_PDSR & digitalPinToBitMask(MOTOR_FL_ENC_PINS[1])) == LOW) {
        enc.FL += 1;
    } else {
        enc.FL -= 1;
    }
}

void MOTOR_FL_ENC_1() {//check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_FL_ENC_PINS[0])->PIO_PDSR & digitalPinToBitMask(MOTOR_FL_ENC_PINS[0])) == LOW) {
        enc.FL += 1;
    } else {
        enc.FL -= 1;
    }
}

void MOTOR_RL_ENC_0() {
    //check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_RL_ENC_PINS[1])->PIO_PDSR & digitalPinToBitMask(MOTOR_RL_ENC_PINS[1])) == LOW) {
        enc.RL += 1;
    } else {
        enc.RL -= 1;
    }
}

void MOTOR_RL_ENC_1() {//check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_RL_ENC_PINS[0])->PIO_PDSR & digitalPinToBitMask(MOTOR_RL_ENC_PINS[0])) == LOW) {
        enc.RL += 1;
    } else {
        enc.RL -= 1;
    }
}

void MOTOR_RR_ENC_0() {
    //check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_RR_ENC_PINS[1])->PIO_PDSR & digitalPinToBitMask(MOTOR_RR_ENC_PINS[1])) == LOW) {
        enc.RR += 1;
    } else {
        enc.RR -= 1;
    }
}

void MOTOR_RR_ENC_1() {//check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_RR_ENC_PINS[0])->PIO_PDSR & digitalPinToBitMask(MOTOR_RR_ENC_PINS[0])) == LOW) {
        enc.RR += 1;
    } else {
        enc.RR -= 1;
    }
}