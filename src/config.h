#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =================================================================
// PIN DEFINITIONS
// =================================================================
// Update these pin numbers to match your physical wiring.

// --- Front-Left Wheel ---
#define FL_DRIVE_STEP_PIN 13
#define FL_DRIVE_DIR_PIN 14
#define FL_STEER_STEP_PIN 3
#define FL_STEER_DIR_PIN 6
#define FL_HEIGHT_STEP_PIN 2
#define FL_HEIGHT_DIR_PIN 4

// --- Front-Right Wheel ---
#define FR_DRIVE_STEP_PIN 21
#define FR_DRIVE_DIR_PIN 22
#define FR_STEER_STEP_PIN 23
#define FR_STEER_DIR_PIN 25
#define FR_HEIGHT_STEP_PIN 26
#define FR_HEIGHT_DIR_PIN 27

// --- Rear-Left Wheel ---
#define RL_DRIVE_STEP_PIN 19
#define RL_DRIVE_DIR_PIN 18
#define RL_STEER_STEP_PIN 5
#define RL_STEER_DIR_PIN 17
#define RL_HEIGHT_STEP_PIN 16
#define RL_HEIGHT_DIR_PIN 4

// --- Rear-Right Wheel ---
#define RR_DRIVE_STEP_PIN 32
#define RR_DRIVE_DIR_PIN 33
#define RR_STEER_STEP_PIN 25
#define RR_STEER_DIR_PIN 26
#define RR_HEIGHT_STEP_PIN 12
#define RR_HEIGHT_DIR_PIN 14

// =================================================================
// MOTOR PARAMETERS
// =================================================================
// Tune these values for optimal performance of your motors.

// --- Max Speed (steps/sec) ---
#define MAX_DRIVE_SPEED 1000.0
#define MAX_STEER_SPEED 500.0
#define MAX_HEIGHT_SPEED 500.0

// --- Acceleration (steps/sec^2) ---
#define ACCEL_DRIVE 3000.0
#define ACCEL_STEER 1000.0
#define ACCEL_HEIGHT 1000.0

// =================================================================
// MOTOR DIRECTION INVERSION
// =================================================================
// Set to 'true' if a motor runs in the wrong direction.

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

#endif // CONFIG_H