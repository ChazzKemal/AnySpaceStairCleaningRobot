#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// This enum is now globally accessible to any file that includes this header.
enum Vl53l0xSensor
{
    STAIR_WALL1,
    STAIR_WALL2,
    LEFT,
    RIGHT,
    FLOOR_BELOW,
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
#define MAX_DRIVE_SPEED 200.0
#define MAX_STEER_SPEED 100.0
#define MAX_HEIGHT_SPEED 100.0

// --- Acceleration (steps/sec^2) ---
#define ACCEL_DRIVE 3000.0
#define ACCEL_STEER 1000.0
#define ACCEL_HEIGHT 1000.0

// Initial State
#define INITIAL_HEIGHT 30.0
#define INITIAL_STEER_ANGLE 0.0

// Algorithm parameters
#define DISTANCE_TO_WALL 200.0 // in mm
#define ALIGN_THRESHOLD 5.0
#define CLIMBABLE_DISTANCE_TO_WALL 2.0                 // in mm
#define OVERCLIMB_HEIGHT 6.2                           // we need to adjust without the shamgumi                             // in mm
#define MAX_HEIGHT 16.5                                // in mm
#define NEXT_STAIR_DISTANCE 15.0                       // in mm
#define SAFE_DISTANCE_TO_EXTRACT_FRONT_WHEELS 100.0    // in mm we are not using this
#define SAFE_DISTANCE_TO_EXTRACT_REAR_WHEELS 50.0      // in mm we are not using this
#define SAFE_SENSOR_READING_TO_EXTRACT_FRONT_WHEELS 26 // in mm
#define SAFE_SENSOR_READING_TO_EXTRACT_REAR_WHEELS 11  // in mm we are not using this
#define SENSOR_CORRECTION 12.143
#define NEEDED_STEERING_PERPENDICULAR 85.0 // TODO:in a unit
#define CONSIDERED_CLEANED_SENSOR_DISTANCE 3.0

// --- Pin Definitions ---

// --- channels of the adc necessary for homing ---
#define FL_HOME_PIN 0 //
#define FR_HOME_PIN 0
#define RL_HOME_PIN 0
#define RR_HOME_PIN 0

// --- Front Left Wheel Pins ---
#define FL_DRIVE_STEP_PIN 36
#define FL_DRIVE_DIR_PIN 37
#define FL_STEER_STEP_PIN 16
#define FL_STEER_DIR_PIN 15
#define FL_HEIGHT_STEP_PIN 46
#define FL_HEIGHT_DIR_PIN 3

// --- Front Right Wheel Pins ---
#define FR_DRIVE_STEP_PIN 38
#define FR_DRIVE_DIR_PIN 39
#define FR_STEER_STEP_PIN 7
#define FR_STEER_DIR_PIN 6
#define FR_HEIGHT_STEP_PIN 11
#define FR_HEIGHT_DIR_PIN 10

// --- Rear Left Wheel Pins ---
#define RL_DRIVE_STEP_PIN 40
#define RL_DRIVE_DIR_PIN 41
#define RL_STEER_STEP_PIN 4 // 16
#define RL_STEER_DIR_PIN 5  // 15
#define RL_HEIGHT_STEP_PIN 13
#define RL_HEIGHT_DIR_PIN 12

// --- Rear Right Wheel Pins ---
#define RR_DRIVE_STEP_PIN 42
#define RR_DRIVE_DIR_PIN 2
#define RR_STEER_STEP_PIN 18
#define RR_STEER_DIR_PIN 17
#define RR_HEIGHT_STEP_PIN 1
#define RR_HEIGHT_DIR_PIN 14

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

#define CONVERSION_FACTOR_DRIVE 1      // Example: 0.1 mm per step
#define CONVERSION_FACTOR_STEER 1      // Example: 0.
#define CONVERSION_FACTOR_HEIGHT 36.76 // Example: 0.1 mm per step

#define FL_HAS_DRIVE true
#define FL_HAS_STEER true

#define FR_HAS_DRIVE true
#define FR_HAS_STEER true

#define RL_HAS_DRIVE false
#define RL_HAS_STEER false

#define RR_HAS_DRIVE false
#define RR_HAS_STEER false

// ----------------------  Vacuum stuff
#define VACUUM_PIN 35
#define BRIZZLES_PIN 0

#endif // ROBOT_CONFIG_H