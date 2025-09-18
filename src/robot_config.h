#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// This enum is now globally accessible to any file that includes this header.
enum Vl53l0xSensor
{
    FLOOR_BELOW,
    STAIR_WALL1,
    STAIR_WALL2,
    FRONT,
    REAR,
    NUM_SENSORS // This will be 5
};

// You can also put the Wheel enum here if you want to use it elsewhere
enum Wheel
{
    FRONT_LEFT,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT,
    NUM_WHEELS // This will be 4
};

// --- Max Speed (steps/sec) ---
#define MAX_DRIVE_SPEED 100.0
#define MAX_STEER_SPEED 100.0
#define MAX_HEIGHT_SPEED 100.0

// --- Acceleration (steps/sec^2) ---
#define ACCEL_DRIVE 3000.0
#define ACCEL_STEER 1000.0
#define ACCEL_HEIGHT 1000.0

// --- Pin Definitions ---

// --- channels of the adc necessary for homing ---
#define FL_HOME_PIN 0 // 
#define FR_HOME_PIN 1
#define RL_HOME_PIN 2
#define RR_HOME_PIN 3

// --- Front Left Wheel Pins ---
#define FL_DRIVE_STEP_PIN 5
#define FL_DRIVE_DIR_PIN 4
#define FL_STEER_STEP_PIN 7
#define FL_STEER_DIR_PIN 6
#define FL_HEIGHT_STEP_PIN 18
#define FL_HEIGHT_DIR_PIN 17

// --- Front Right Wheel Pins ---
#define FR_DRIVE_STEP_PIN 12
#define FR_DRIVE_DIR_PIN 13
#define FR_STEER_STEP_PIN 14
// #define FR_STEER_DIR_PIN 15
// #define FR_HEIGHT_STEP_PIN 16

// #define FR_HEIGHT_DIR_PIN 17

// --- Rear Left Wheel Pins ---
#define RL_DRIVE_STEP_PIN 21
#define RL_DRIVE_DIR_PIN 22
#define RL_STEER_STEP_PIN 23
#define RL_STEER_DIR_PIN 25
#define RL_HEIGHT_STEP_PIN 26
#define RL_HEIGHT_DIR_PIN 27

// --- Rear Right Wheel Pins ---
#define RR_DRIVE_STEP_PIN 33
#define RR_DRIVE_DIR_PIN 34
#define RR_STEER_STEP_PIN 35
#define RR_STEER_DIR_PIN 36
#define RR_HEIGHT_STEP_PIN 37
#define RR_HEIGHT_DIR_PIN 38

// --- Front-Left Wheel ---
#define FL_INVERT_DRIVE false
#define FL_INVERT_STEER false
#define FL_INVERT_HEIGHT false

// --- Front-Right Wheel ---
#define FR_INVERT_DRIVE true // Example: This wheel might be mirrored
#define FR_INVERT_STEER false
#define FR_INVERT_HEIGHT false

// --- Rear-Left Wheel ---
#define RL_INVERT_DRIVE false
#define RL_INVERT_STEER true
#define RL_INVERT_HEIGHT false

// --- Rear-Right Wheel ---
#define RR_INVERT_DRIVE true // Example: This wheel might be mirrored
#define RR_INVERT_STEER true
#define RR_INVERT_HEIGHT false

#define CONVERSION_FACTOR_DRIVE 1  // Example: 0.1 mm per step
#define CONVERSION_FACTOR_STEER 1  // Example: 0.
#define CONVERSION_FACTOR_HEIGHT 1 // Example: 0.1 mm per step


// ----------------------  Vacuum stuff
#define VACUUM_PIN 35
#define BRIZZLES_PIN 0 

// start stop pin 

#define START_BUTTON_PIN 20
#define STOP_BUTTON_PIN 21

#endif // ROBOT_CONFIG_H