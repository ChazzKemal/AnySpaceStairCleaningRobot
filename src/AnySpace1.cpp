#include "AnySpace1.h"
#include "distance_handler.h" // Include the handler
#include "robot_config.h"
#include "ArticulatedWheel.h"
#include "mpu_handler.h"
std::array<uint16_t, NUM_SENSORS> AnySpace1::get_vl53l0x_data()
{
    // Call the handler function and pass our private sensor array to it.
    return ::get_vl53l0x_data(this->lox1, this->sensor_data);
}
std::tuple<float, float> AnySpace1::get_mpu_data()
{
    return get_angles(this->mpu);
}

void AnySpace1::begin()
{
    Serial.begin(115200);
    Wire.begin();
    mpu.begin();
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        tca_select(i);
        if (!lox1.begin())
        {
            Serial.print("Failed to boot VL53L0X on channel ");
            Serial.println(i);
        }
    }
    wheels[0]->begin();
    wheels[1]->begin();
    // for (auto &wheel : wheels)
    // {
    //     wheel->begin();
    // }
    // AnySpace1::home();
}

AnySpace1::AnySpace1()
// TODO: decide on if to keep these or not
//  : distance_sensors{
//        Adafruit_VL53L0X(),
//        Adafruit_VL53L0X(),
//        Adafruit_VL53L0X(),
//        Adafruit_VL53L0X(),
//        Adafruit_VL53L0X()}

{
    m_engine->init(); // Call init() first.
    // distance_sensors[0] = new Adafruit_VL53L0X();
    // distance_sensors[1] = new Adafruit_VL53L0X();
    // Now, create the wheels.
    wheels[0] = new ArticulatedWheel(m_engine, FL_DRIVE_STEP_PIN, FL_DRIVE_DIR_PIN, FL_STEER_STEP_PIN, FL_STEER_DIR_PIN, FL_HEIGHT_STEP_PIN, FL_HEIGHT_DIR_PIN, FL_HOME_PIN, FL_INVERT_DRIVE, FL_INVERT_STEER, FL_INVERT_HEIGHT);
    wheels[1] = new ArticulatedWheel(m_engine, FR_DRIVE_STEP_PIN, FR_DRIVE_DIR_PIN, FR_STEER_STEP_PIN, FR_STEER_DIR_PIN, FR_HEIGHT_STEP_PIN, FR_HEIGHT_DIR_PIN, FL_HOME_PIN, FR_INVERT_DRIVE, FR_INVERT_STEER, FR_INVERT_HEIGHT);
    // wheels[2] = new ArticulatedWheel(m_engine, RL_DRIVE_STEP_PIN, RL_DRIVE_DIR_PIN, RL_STEER_STEP_PIN, RL_STEER_DIR_PIN, RL_HEIGHT_STEP_PIN, RL_HEIGHT_DIR_PIN, FL_HOME_PIN, RL_INVERT_DRIVE, RL_INVERT_STEER, RL_INVERT_HEIGHT);
    // wheels[3] = new ArticulatedWheel(m_engine, RR_DRIVE_STEP_PIN, RR_DRIVE_DIR_PIN, RR_STEER_STEP_PIN, RR_STEER_DIR_PIN, RR_HEIGHT_STEP_PIN, RR_HEIGHT_DIR_PIN, FL_HOME_PIN, RR_INVERT_DRIVE, RR_INVERT_STEER, RR_INVERT_HEIGHT);
}

void AnySpace1::home()
{
    bool height_seek[NUM_WHEELS] = {true};
    bool steer_seek[NUM_WHEELS] = {false};
    wheels[0]->height->_stepper->runForward(); // max step rate (steps/second)
    // for (auto &wheel : wheels)
    // {
    //     wheel->height->_stepper->runForward();
    // }

    bool all_homed = false;
    int homed_wheels = 0;
    while (!all_homed)
    {
        for (int i = 0; i < 1; ++i) //< NUM_WHEELS; ++i)
        {
            // Check if this specific motor is still running
            if (height_seek[i])
            {
                // Read the limit switch.
                if (wheels[i]->checkHomingPin())
                {
                    // The switch is pressed! Stop the motor and reset its position to 0.
                    wheels[i]->height->_stepper->forceStopAndNewPosition(0);
                    Serial.print("Wheel height ");
                    Serial.print(i);
                    Serial.println(" is home.");
                    wheels[i]->height->moveSteps(-100); // Ensure motor goes back a bit
                    wheels[i]->steer->_stepper->runForward();
                    height_seek[i] = false;
                    steer_seek[i] = true;
                }
            }
            if (steer_seek[i])
            {
                if (wheels[i]->checkHomingPin() && !wheels[i]->height->_stepper->isRunning())
                {
                    wheels[i]->steer->_stepper->forceStopAndNewPosition(0);
                    Serial.print("Wheel steer ");
                    Serial.print(i);
                    Serial.println(" is home.");
                    homed_wheels++;
                    if (homed_wheels >= 1) // change this NUM_WHEELS)
                    {
                        all_homed = true;
                    }
                }
            }
        }
    };
}

void AnySpace1::go_n_steps(float n_steps)
{
    for (auto &wheel : wheels)
    {
        wheel->drive->moveRelative(n_steps);
    }
}

void AnySpace1::reverse_direction()
{
    for (auto &wheel : wheels)
    {
        wheel->drive->invertDir = !wheel->drive->invertDir;
    }
}
void AnySpace1::go_vertically(float n_steps)
{
    for (auto &wheel : wheels)
    {
        wheel->height->moveRelative(n_steps);
    }
}
void AnySpace1::steer_wheel(float n_steps)
{
    for (auto &wheel : wheels)
    {
        wheel->steer->moveRelative(n_steps);
    }
}
void AnySpace1::get_sensor_data()
{
    this->sensor_data = get_vl53l0x_data();
    std::tie(this->pitch, this->roll) = get_mpu_data();
}

void AnySpace1::run_forward()
{
    for (auto &wheel : wheels)
    {
        wheel->drive->_stepper->runForward();
        break;
    }
}

void AnySpace1::run_backward()
{
    for (auto &wheel : wheels)
    {
        wheel->drive->_stepper->runBackward();
    }
}

void AnySpace1::stop()
{
    for (auto &wheel : wheels)
    {
        wheel->drive->_stepper->forceStop();
    }
}

void AnySpace1::run()
{
    // AnySpace1::get_sensor_data();
    // AnySpace1::print_sensor_data();

    wheels[0]->drive->moveToPosition(300);
    get_sensor_data();
    print_sensor_data();
    delay(2000);
    wheels[0]->drive->moveToPosition(-300);
    delay(2000);
    // wheels[0]->height->moveToPosition(300);
    // delay(2000);
    // wheels[0]->height->moveToPosition(-300);
    // delay(2000);
    // wheels[0]->steer->moveToPosition(300);
    // delay(2000);
    // wheels[0]->steer->moveToPosition(-300);
    // delay(2000);
    wheels[1]->drive->moveToPosition(300);
    delay(2000);
    wheels[1]->drive->moveToPosition(-300);
    delay(2000);
    // wheels[1]->drive->moveToPosition(0);
    // wheels[0]->steer->moveToPosition(0);
    // wheels[0]->height->moveToPosition(0);
    // wheels[0]->drive->moveToPosition(0);
    // delay(2000);
    // stop();
    // run_backward();
    // delay(500);
    // stop();
    // delay(1000);
    // steer_wheel(200);
    // delay(500);
    // go_vertically(200);
    // delay(500);
}

void AnySpace1::go_initial_state()
{
    for (auto &wheel : wheels)
    {
        wheel->steer->moveToPosition(INITIAL_HEIGHT);
        wheel->height->moveToPosition(INITIAL_STEER_ANGLE);
    }
}

void AnySpace1::align_with_wall()
{
    bool left_stopped = false;
    bool right_stopped = false;
    for (auto &wheel : wheels)
    {
        wheel->drive->_stepper->runForward();
    }
    while (true)
    {
        get_sensor_data();
        float left_distance = this->sensor_data[STAIR_WALL1];
        float right_distance = this->sensor_data[STAIR_WALL2];

        if (left_distance < DISTANCE_TO_WALL && !left_stopped)
        {
            wheels[0]->drive->_stepper->forceStop();
            wheels[2]->drive->_stepper->forceStop();
            left_stopped = true;
            if (right_stopped)
            {
                break;
            }
        }
        if (right_distance < DISTANCE_TO_WALL && !right_stopped)
        {
            wheels[1]->drive->_stepper->forceStop();
            wheels[3]->drive->_stepper->forceStop();
            right_stopped = true;
            if (left_stopped)
            {
                break;
            }
        }
    }
}

void AnySpace1::print_sensor_data()
{
    Serial.print("Pitch: ");
    Serial.print(this->pitch);
    Serial.print(", Roll: ");
    Serial.println(this->roll);
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        Serial.print("Distance Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(this->sensor_data[i]);
    }
}