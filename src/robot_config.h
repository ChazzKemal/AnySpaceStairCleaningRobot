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

// --- Pin Definitions ---

// --- HOMING SWITCH PINS ---
#define FL_HOME_PIN 2 // Example pin
#define FR_HOME_PIN 4 // Example pin
#define RL_HOME_PIN 5 // Example pin
#define RR_HOME_PIN 7 // Example pin

// --- Front Left Wheel Pins ---
#define FL_DRIVE_STEP_PIN 2
#define FL_DRIVE_DIR_PIN 3
#define FL_STEER_STEP_PIN 4
#define FL_STEER_DIR_PIN 5
#define FL_HEIGHT_STEP_PIN 6
#define FL_HEIGHT_DIR_PIN 7
#define FL_STEER_LIMIT_PIN 8

// --- Front Right Wheel Pins ---
#define FR_DRIVE_STEP_PIN 12
#define FR_DRIVE_DIR_PIN 13
#define FR_STEER_STEP_PIN 14
#define FR_STEER_DIR_PIN 15
#define FR_HEIGHT_STEP_PIN 16
#define FR_HEIGHT_DIR_PIN 17
#define FR_STEER_LIMIT_PIN 18

// --- Rear Left Wheel Pins ---
#define RL_DRIVE_STEP_PIN 21
#define RL_DRIVE_DIR_PIN 22
#define RL_STEER_STEP_PIN 23
#define RL_STEER_DIR_PIN 25
#define RL_HEIGHT_STEP_PIN 26
#define RL_HEIGHT_DIR_PIN 27
#define RL_STEER_LIMIT_PIN 32

// --- Rear Right Wheel Pins ---
#define RR_DRIVE_STEP_PIN 33
#define RR_DRIVE_DIR_PIN 34
#define RR_STEER_STEP_PIN 35
#define RR_STEER_DIR_PIN 36
#define RR_HEIGHT_STEP_PIN 37
#define RR_HEIGHT_DIR_PIN 38
#define RR_STEER_LIMIT_PIN 39

#endif // ROBOT_CONFIG_H