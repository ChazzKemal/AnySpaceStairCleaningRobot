#include "AnySpace1.h"
#include "distance_handler.h" // Include the handler
#include "robot_config.h"
#include "ArticulatedWheel.h"
#include "mpu_handler.h"
std::array<uint16_t, NUM_SENSORS> AnySpace1::get_vl53l0x_data()
{
    // Call the handler function and pass our private sensor array to it.
    return ::get_vl53l0x_data(this->distance_sensors, this->sensor_data);
}

std::tuple<float, float> AnySpace1::get_mpu_data()
{
    return get_angles(this->mpu);
}

void AnySpace1::begin()
{
    Wire.begin();
    mpu.begin();
    FastAccelStepperEngine engine = FastAccelStepperEngine();
    FastAccelStepper *m_drive = NULL;
    for (auto &wheel : wheels)
    {
        wheel.begin();
    }
    AnySpace1::home();
}

AnySpace1::AnySpace1()
    : m_engine(FastAccelStepperEngine()), // Initialize the engine object
      wheels{
          {// The order here MUST match the 'Wheel' enum
           // FRONT_LEFT
           ArticulatedWheel(&m_engine, FL_DRIVE_STEP_PIN, FL_DRIVE_DIR_PIN, FL_STEER_STEP_PIN, FL_STEER_DIR_PIN, FL_HEIGHT_STEP_PIN, FL_HEIGHT_DIR_PIN, false, false, false),
           // FRONT_RIGHT
           ArticulatedWheel(&m_engine, FR_DRIVE_STEP_PIN, FR_DRIVE_DIR_PIN, FR_STEER_STEP_PIN, FR_STEER_DIR_PIN, FR_HEIGHT_STEP_PIN, FR_HEIGHT_DIR_PIN, false, false, false),
           // REAR_LEFT
           ArticulatedWheel(&m_engine, RL_DRIVE_STEP_PIN, RL_DRIVE_DIR_PIN, RL_STEER_STEP_PIN, RL_STEER_DIR_PIN, RL_HEIGHT_STEP_PIN, RL_HEIGHT_DIR_PIN, false, false, false),
           // REAR_RIGHT
           ArticulatedWheel(&m_engine, RR_DRIVE_STEP_PIN, RR_DRIVE_DIR_PIN, RR_STEER_STEP_PIN, RR_STEER_DIR_PIN, RR_HEIGHT_STEP_PIN, RR_HEIGHT_DIR_PIN, false, false, false)}},
      distance_sensors{
          // The order here MUST match the 'Sensor' enum
          Adafruit_VL53L0X(), // FRONT_LEFT
          Adafruit_VL53L0X(), // FRONT_RIGHT
          Adafruit_VL53L0X(), // REAR_LEFT
          Adafruit_VL53L0X()  // REAR_RIGHT
      }

{
}

void AnySpace1::home()
{
    bool height_seek[NUM_WHEELS] = {true};
    bool steer_seek[NUM_WHEELS] = {false};

    for (auto &wheel : wheels)
    {
        wheel.height->_stepper->runForward();
    }

    const uint8_t home_pins[] = {FL_HOME_PIN, FR_HOME_PIN, RL_HOME_PIN, RR_HOME_PIN};

    bool all_homed = false;
    int homed_wheels = 0;
    while (!all_homed)
    {
        for (int i = 0; i < NUM_WHEELS; ++i)
        {
            // Check if this specific motor is still running
            if (height_seek[i])
            {
                // Read the limit switch.
                if (digitalRead(home_pins[i]) == HIGH)
                {
                    // The switch is pressed! Stop the motor and reset its position to 0.
                    wheels[i].height->_stepper->forceStopAndNewPosition(0);
                    Serial.print("Wheel height ");
                    Serial.print(i);
                    Serial.println(" is home.");
                    wheels[i].height->moveSteps(-100); // Ensure motor goes back a bit
                    wheels[i].steer->_stepper->runForward();
                    height_seek[i] = false;
                    steer_seek[i] = true;
                }
            }
            if (steer_seek[i])
            {
                if (digitalRead(home_pins[i]) == HIGH)
                {
                    wheels[i].steer->_stepper->forceStopAndNewPosition(0);
                    Serial.print("Wheel steer ");
                    Serial.print(i);
                    Serial.println(" is home.");
                    homed_wheels++;
                    if (homed_wheels >= NUM_WHEELS)
                    {
                        all_homed = true;
                    }
                }
            }
        }
    };
}