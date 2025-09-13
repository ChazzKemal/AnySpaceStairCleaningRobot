#include "ArticulatedWheel.h"

// The constructor now takes pin numbers and initializes the AccelStepper objects directly.
ArticulatedWheel::ArticulatedWheel(uint8_t drive_step_pin, uint8_t drive_dir_pin,
                                   uint8_t steer_step_pin, uint8_t steer_dir_pin,
                                   uint8_t height_step_pin, uint8_t height_dir_pin,
                                   bool invert_drive, bool invert_steer, bool invert_height)
    : // Use an initializer list to construct the AccelStepper objects.
      // This assumes you are using a standard STEP/DIR driver.
      m_drive(AccelStepper::DRIVER, drive_step_pin, drive_dir_pin),
      m_steer(AccelStepper::DRIVER, steer_step_pin, steer_dir_pin),
      m_height(AccelStepper::DRIVER, height_step_pin, height_dir_pin)
{
    // Set direction multipliers based on the boolean flags.
    m_drive_direction = invert_drive ? -1 : 1;
    m_steer_direction = invert_steer ? -1 : 1;
    m_height_direction = invert_height ? -1 : 1;
}

void ArticulatedWheel::begin(float max_drive_speed,
                             float max_steer_speed,
                             float max_height_speed,
                             float acceleration_drive,
                             float acceleration_steer,
                             float acceleration_height)
{
    m_drive.setMaxSpeed(max_drive_speed);
    m_steer.setMaxSpeed(max_steer_speed);
    m_height.setMaxSpeed(max_height_speed);

    m_drive.setAcceleration(acceleration_drive); // Set acceleration
    m_steer.setAcceleration(acceleration_steer);
    m_height.setAcceleration(acceleration_height);
}

void ArticulatedWheel::setDriveSpeed(float speed)
{
    // Apply the direction multiplier to reverse the motor if needed
    m_drive.setSpeed(speed * m_drive_direction);
}

void ArticulatedWheel::setSteerAngle(long angle_degrees)
{
    // This assumes you have calibrated how many steps correspond to one degree.
    // Example: 200 steps/rev * 8 microsteps / 360 degrees = 4.44 steps/degree
    const float STEPS_PER_DEGREE = 4.44;
    // Apply the direction multiplier
    m_steer.moveTo(angle_degrees * STEPS_PER_DEGREE * m_steer_direction);
}

void ArticulatedWheel::setHeight(long position)
{
    // Apply the direction multiplier
    m_height.moveTo(position * m_height_direction);
}

void ArticulatedWheel::run()
{
    m_drive.runSpeed(); // Use runSpeed() for continuous rotation
    m_steer.run();      // Use run() for moving to a target position
    m_height.run();     // Use run() for moving to a target position
}
