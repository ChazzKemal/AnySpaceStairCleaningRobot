#ifndef ARTICULATED_WHEEL_H
#define ARTICULATED_WHEEL_H

#include <AccelStepper.h>

/**
 * @class ArticulatedWheel
 * @brief Manages a single advanced wheel assembly with 3 degrees of freedom.
 *
 * This class encapsulates the creation and control of the three stepper motors
 * for a single wheel unit (drive, steer, and height).
 */
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
     */
    ArticulatedWheel(uint8_t drive_step_pin, uint8_t drive_dir_pin,
                     uint8_t steer_step_pin, uint8_t steer_dir_pin,
                     uint8_t height_step_pin, uint8_t height_dir_pin,
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

    // --- Low-Level Control for This Wheel ---

    /**
     * @brief Sets the rolling speed of the wheel.
     * @param speed Speed in steps/sec (positive is forward, negative is backward).
     */
    void setDriveSpeed(float speed);

    /**
     * @brief Sets the steering angle of the wheel assembly.
     * @param angle_degrees Target angle in degrees.
     */
    void setSteerAngle(long angle_degrees);

    /**
     * @brief Sets the vertical position (height) of the wheel.
     * @param position Target height in steps.
     */
    void setHeight(long position);

    /**
     * @brief Must be called continuously to update all three motors.
     */
    void run();

private:
    // The AccelStepper objects are now members of the class, not references.
    AccelStepper m_drive;
    AccelStepper m_steer;
    AccelStepper m_height;

    // Direction multipliers (-1 for inverted, 1 for normal)
    int m_drive_direction;
    int m_steer_direction;
    int m_height_direction;
};

#endif // ARTICULATED_WHEEL_H
