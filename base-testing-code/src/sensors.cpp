#include "sensors.h"
#include <AceRoutine.h>

#define reverse_sensor

// The pins used by the sensors in each location
#define DIST_LEFT  0
#define DIST_RIGHT 1
#define DIST_FRONT 2
#define DIST_BACK  3

// Encoders proved unreliable, we dont need them
#ifdef USE_ENCODERS
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
#endif

// Distance Sensors definition
const uint8_t distance_sensors[][2] = {{A0, 13}, {A1, 13}, {A2, 13}, {A3, 13}};
const uint8_t num_dist_sensors = 3;
// Holder for calibration values, if any defined
uint16_t distance_sensor_cal_values[][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}};

// Offsets for calculating distance to wall from edge of robot
uint16_t robot_width_mm = 155;
uint16_t wall_to_wall_width_mm = 175+10*2-3;

// distances in mm for calibration
uint64_t distance_mm_vals[][2] = {{(43)*1, (wall_to_wall_width_mm-robot_width_mm+43)*1}, 
                                    {(45)*1, (wall_to_wall_width_mm-robot_width_mm+45)*1}, 
                                    {(54)*1, (120+54)*1}};

// Initialize sensors
dist_sensor dist_left(DIST_LEFT);
dist_sensor dist_right(DIST_RIGHT);
dist_sensor dist_front(DIST_FRONT);
// dist_sensor dist_back(DIST_BACK);


VL53L4CD TOF_left(&Wire, 52);
VL53L4CD TOF_front(&Wire, 53);

// Ignore small section at middle of joystick where readings
// are often fluctuating
const uint16_t joystick_deadzone = 100;
const uint16_t max_analog = 4095;

#ifdef USE_GYRO
MPU9250_DMP imu;
#endif

int target_deg = 0;

/**
 * @brief Setup the sensors for use, ensure they are ready before continuing
 * 
 */
void setup_sensors() {

    // If using encoders, attach interrupts
    #ifdef USE_ENCODERS
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; i < 2; ++i) {
            pinMode(MOTOR_ENC_PINS[i][j], INPUT);
        }
    }

    attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENC_PINS[0]), MOTOR_FR_ENC_0, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENC_PINS[0]), MOTOR_FL_ENC_0, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENC_PINS[0]), MOTOR_RL_ENC_0, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENC_PINS[0]), MOTOR_RR_ENC_0, RISING);

    #endif

    // If using gyro, ensure that we are connected to it, and it is ready
    #ifdef USE_GYRO
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
    
    // If using the gyro, have to delay to allow gyro to start up, otherwise it gets garbage readings
    delay(10000);

    for (int i = 0; i < 10; ++i) {
        get_rotation();
    }

    #endif



    TOF_left.begin();
    TOF_left.VL53L4CD_Off();

    TOF_front.begin();
    TOF_front.InitSensor();
    TOF_front.VL53L4CD_SetI2CAddress(0x53);
    TOF_front.VL53L4CD_SetRangeTiming(150, 50);
    TOF_front.VL53L4CD_StartRanging();

    TOF_left.VL53L4CD_On();
    TOF_left.InitSensor();
    TOF_left.VL53L4CD_SetRangeTiming(150, 50);
    TOF_left.VL53L4CD_StartRanging();

    while(true) {
        int dist = 0;

        
        while (!read_TOF_front(&dist));


        Serial.println(dist);
        
    }
    



}


bool read_tof(VL53L4CD* sensor, int* output, int* status_out = nullptr) {
    uint8_t NewDataReady = 0;
    VL53L4CD_Result_t results;
    uint8_t status;

    
    status = sensor->VL53L4CD_CheckForDataReady(&NewDataReady);
    
    if (status_out != nullptr) {
        *status_out = status;
    }

    if (!NewDataReady) {
        
        
        return false;
    }

    if ((!status) && (NewDataReady != 0)) {
        // (Mandatory) Clear HW interrupt to restart measurements
        sensor->VL53L4CD_ClearInterrupt();

        // Read measured distance. RangeStatus = 0 means valid data
        sensor->VL53L4CD_GetResult(&results);

        if (results.range_status != 0) {
            return false;
        }

        *output = results.distance_mm;
        return true;
    }
}

bool read_TOF_front(int* output, int* status) {
    return read_tof(&TOF_front, output, status);
}

bool read_TOF_left(int* output, int* status) {
    return read_tof(&TOF_left, output, status);
}


/**
 * @brief           Construct a new dist sensor::dist sensor object
 * 
 * @param sensor    Which sensor to init, pulls info from distance_sensor array
 */
dist_sensor::dist_sensor(uint8_t sensor) {
    this->val_pin = distance_sensors[sensor][DIST_VAL];
    this->gpio_pin = distance_sensors[sensor][DIST_GPIO];

    pinMode(this->val_pin, INPUT);
    pinMode(this->gpio_pin, OUTPUT);
    digitalWrite(this->gpio_pin, HIGH);

    for (int i = 0; i < 2; ++i) {
        this->cal_distances_mm[i] = distance_mm_vals[sensor][i];
        this->cal_values[i] = 0;
    }
    this->sens_num = sensor;
}

/**
 * @brief       Receive calibration data for use in distance values
 * 
 * @param val1  x1 when linearly interpolating
 * @param val2  y1 when linearly interpolating
 */
void dist_sensor::calibrate(uint16_t val1, uint16_t val2){
    this->cal_values[0] = val1;
    this->cal_values[1] = val2;
}


#ifdef USE_GYRO
/**
 * @brief           Grab current rotation from sensor, must first request an update
 * 
 * @return float    Rotation in deg, bound to range of -180 to 180
 */
float get_rotation() {
    update_gyro();
    return -imu.pitch;
}

/**
 * @brief           Add a certain amount of degrees to the passed value,
 *                  binding to a range of -180 to 180
 * 
 * @param degrees   Initial value of degrees
 * @param addition  How many degrees to add
 * @return float    The bound value of the addition
 */
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


/**
 * @brief Debug print the data from the gyro, taken from the library example
 * 
 */
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

/**
 * @brief           Ask for new data from sensor
 * 
 * @return true     Gryo had new data and has computed new position
 * @return false    Gyro has no new data, data is stale
 */
bool update_gyro() {
    // Check for new data in the FIFO
    if ( imu.fifoAvailable() ) {
        
        // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
        if ( imu.dmpUpdateFifo() == INV_SUCCESS){
            // computeEulerAngles can be used -- after updating the
            // quaternion values -- to estimate roll, pitch, and yaw
            imu.computeEulerAngles(true);
            //printIMUData();
            return true;
        } 
    }
    return false;
}
#endif

// Encoder interrupts, due to number of interrupts per
// rotation they dont seem to be that reliable for positioning
#ifdef USE_ENCODERS
/**
 * @brief Encoder interrupt, there are two per motor
 * 
 */
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
#endif