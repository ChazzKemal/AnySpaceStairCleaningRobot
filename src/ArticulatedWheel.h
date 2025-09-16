#ifndef ARTICULATED_WHEEL_H
#define ARTICULATED_WHEEL_H

#include <FastAccelStepper.h>

/**
 * @class ArticulatedWheel
 * @brief Manages a single advanced wheel assembly with 3 degrees of freedom.
 *
 * This class encapsulates the creation and control of the three stepper motors
 * for a single wheel unit (drive, steer, and height).
 */

class Stepper
{
public:
    Stepper(FastAccelStepperEngine *engine, uint8_t drive_step_pin, uint8_t drive_dir_pin, bool invert);
    void init(float speed, float acceleration, float conversionFactor);
    float position; // this is the real world position so e.g. mm or degrees
    bool invertDir;
    float default_Speed;
    float default_acceleration;
    int convertToSteps(float realWorldValue);
    FastAccelStepper *_stepper;
    void moveSteps(int steps);
    void moveToPosition(float pos);
    void moveRelative(float relPos);

private:
    float conversionFactor;
};

class ArticulatedWheel
{
public:
    /**
     * @brief Constructor for a single articulated wheel.
     * Initializes the three stepper motors for you.
     * @param drive_step_pin The STEP pin for the drive motor.
     * @param drive_dir_pin The DIRECTION pin for the drive motor.
     * @param steer_step_pin The STEP pin for the steer motor.
     * @param steer_dir_pin The DIRECTION pin for the steer motor.
     * @param height_step_pin The STEP pin for the height motor.
     * @param height_dir_pin The DIRECTION pin for the height motor.
     * @param invert_drive Set to true to reverse the drive motor's direction.
     * @param invert_steer Set to true to reverse the steer motor's direction.
     * @param invert_height Set to true to reverse the height motor's direction.
     *
     */
    Stepper *drive;
    Stepper *steer;
    Stepper *height;

    ArticulatedWheel(FastAccelStepperEngine *engine, uint8_t drive_step_pin, uint8_t drive_dir_pin,
                     uint8_t steer_step_pin, uint8_t steer_dir_pin,
                     uint8_t height_step_pin, uint8_t height_dir_pin, uint8_t _homingPin,
                     bool invert_drive = false, bool invert_steer = false, bool invert_height = false);

    /**
     * @brief Initializes the motors for this wheel.
     * @param max_drive_speed Max speed for the drive motor (steps/sec).
     * @param max_steer_speed Max speed for the steering motor (steps/sec).
     * @param max_height_speed Max speed for the height motor (steps/sec).
     */
    void begin(float max_drive_speed = 1000.0, float max_steer_speed = 500.0, float max_height_speed = 500.0,
               float acceleration_drive = 2000,
               float acceleration_steer = 2000,
               float acceleration_height = 2000);

    bool checkHomingPin();

private:
    // The AccelStepper objects are now members of the class, not references.

    // Direction multipliers (-1 for inverted, 1 for normal)
    int m_drive_direction;
    int m_steer_direction;
    int m_height_direction;
    uint8_t homingPin;
};

#endif // ARTICULATED_WHEEL_H
