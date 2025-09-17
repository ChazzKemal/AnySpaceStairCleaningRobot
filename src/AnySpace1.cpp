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

    ads->begin(0x48);

    ads->setGain(GAIN_ONE);

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
    wheels[2]->begin();
    wheels[3]->begin();
    // for (auto &wheel : wheels)
    // {
    //     wheel->begin();
    // }
    // AnySpace1::home();
    // AnySpace1::go_initial_state();
    // AnySpace1::align_with_wall();
}

AnySpace1::AnySpace1()

{
    m_engine->init(); // Call init() first.
    // distance_sensors[0] = new Adafruit_VL53L0X();
    // distance_sensors[1] = new Adafruit_VL53L0X();
    ads = new Adafruit_ADS1115();

    // Now, create the wheels.
    wheels[0] = new ArticulatedWheel(m_engine, ads, FL_DRIVE_STEP_PIN, FL_DRIVE_DIR_PIN, FL_STEER_STEP_PIN, FL_STEER_DIR_PIN, FL_HEIGHT_STEP_PIN, FL_HEIGHT_DIR_PIN, FL_HOME_PIN, FL_INVERT_DRIVE, FL_INVERT_STEER, FL_INVERT_HEIGHT, FL_HAS_DRIVE, FL_HAS_STEER);
    wheels[1] = new ArticulatedWheel(m_engine, ads, FR_DRIVE_STEP_PIN, FR_DRIVE_DIR_PIN, FR_STEER_STEP_PIN, FR_STEER_DIR_PIN, FR_HEIGHT_STEP_PIN, FR_HEIGHT_DIR_PIN, FL_HOME_PIN, FR_INVERT_DRIVE, FR_INVERT_STEER, FR_INVERT_HEIGHT, FR_HAS_DRIVE, FR_HAS_STEER);
    wheels[2] = new ArticulatedWheel(m_engine, ads, RL_DRIVE_STEP_PIN, RL_DRIVE_DIR_PIN, RL_STEER_STEP_PIN, RL_STEER_DIR_PIN, RL_HEIGHT_STEP_PIN, RL_HEIGHT_DIR_PIN, FL_HOME_PIN, RL_INVERT_DRIVE, RL_INVERT_STEER, RL_INVERT_HEIGHT, RL_HAS_DRIVE, RL_HAS_STEER);
    wheels[3] = new ArticulatedWheel(m_engine, ads, RR_DRIVE_STEP_PIN, RR_DRIVE_DIR_PIN, RR_STEER_STEP_PIN, RR_STEER_DIR_PIN, RR_HEIGHT_STEP_PIN, RR_HEIGHT_DIR_PIN, FL_HOME_PIN, RR_INVERT_DRIVE, RR_INVERT_STEER, RR_INVERT_HEIGHT, RR_HAS_DRIVE, RR_HAS_STEER);
}

void AnySpace1::home()
{
    bool height_seek[NUM_WHEELS] = {true, true, true, true};
    bool steer_seek[NUM_WHEELS] = {false, false, false, false};
    wheels[0]->height->_stepper->runForward(); // max step rate (steps/second)
    wheels[1]->height->_stepper->runForward(); // TODO: change to height
    wheels[2]->height->_stepper->runForward(); // max step rate (steps/second)
    wheels[3]->height->_stepper->runForward(); // TODO: change to height

    // for (auto &wheel : wheels)
    // {
    //     wheel->height->_stepper->runForward();
    // }

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
                if (wheels[i]->checkHomingPin())
                {
                    // The switch is pressed! Stop the motor and reset its position to 0.
                    wheels[i]->height->_stepper->forceStopAndNewPosition(0);
                    Serial.print("Wheel height ");
                    Serial.print(i);
                    Serial.println(" is home.");
                    wheels[i]->height->moveSteps(-10); // Ensure motor goes back a bit
                    if (wheels[i]->steer)
                    {
                        wheels[i]->steer->_stepper->runForward();
                    }
                    height_seek[i] = false;
                    steer_seek[i] = true;
                }
            }
            if (steer_seek[i])
            {
                if (wheels[i]->checkHomingPin() && !wheels[i]->height->_stepper->isRunning())
                {
                    if (wheels[i]->steer)
                    {
                        wheels[i]->steer->_stepper->forceStopAndNewPosition(0);
                    }
                    Serial.print("Wheel steer ");
                    Serial.print(i);
                    Serial.println(" is home.");
                    homed_wheels++;
                    if (homed_wheels >= 4) // change this NUM_WHEELS)
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
        if (wheel->drive)
        {
            wheel->drive->moveRelative(n_steps);
        }
    }
}

void AnySpace1::reverse_direction()
{
    for (auto &wheel : wheels)
    {
        if (wheel->drive)
        {
            wheel->drive->invertDir = !wheel->drive->invertDir;
        }
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
        if (wheel->steer)
        {
            wheel->steer->moveRelative(n_steps);
        }
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
        if (wheel->drive)
        {
            wheel->drive->_stepper->runForward();
        }
    }
}

void AnySpace1::run_backward()
{
    for (auto &wheel : wheels)
    {
        if (wheel->drive)
        {
            wheel->drive->_stepper->runBackward();
        }
    }
}

void AnySpace1::stop()
{
    for (auto &wheel : wheels)
    {
        if (wheel->drive)
        {
            wheel->drive->_stepper->forceStop();
        }
    }
}

void AnySpace1::run()
{
    // AnySpace1::get_sensor_data();
    // AnySpace1::print_sensor_data();
    int wheel_no = 0;
    for (auto &wheel : wheels)
    {

        if (wheel->drive)
        {
            wheel_no++;
            if (!wheel->drive->_stepper->isRunning())
            {
                wheel->drive->moveRelative(300);
            }
            Serial.println("wheel drive commanded");
            Serial.println(wheel_no);
        }
    }
    for (int i = 0; i < NUM_WHEELS; ++i)
    {
        // Get a reference to the current wheel for cleaner code
        auto &wheel = wheels[i];

        if (wheel->steer)
        {
            static int steer_directions[NUM_WHEELS] = {1, 1, 1, 1};

            wheel_no++;
            if (!wheel->steer->_stepper->isRunning())
            {
                wheel->steer->moveRelative(steer_directions[i] * 30);
                steer_directions[i] *= -1;
            }
            Serial.println("wheel rotate commanded");
            Serial.println(wheel_no);
        }
    }
}

void AnySpace1::go_initial_state()
{
    // for (auto &wheel : wheels)
    // {
    //     wheel->steer->moveToPosition(INITIAL_HEIGHT);
    //     wheel->height->moveToPosition(INITIAL_STEER_ANGLE);
    // }

    for (int i = 0; i < NUM_WHEELS; ++i)
    {
        // Check if the pointer is valid before using it to prevent crashes
        if (wheels[i]->steer)
        {
            wheels[i]->steer->moveToPosition(INITIAL_STEER_ANGLE);
        }
        wheels[i]->height->moveToPosition(INITIAL_HEIGHT);
    }

    bool motors_running;
    do
    {
        motors_running = false;
        for (int j = 0; j < NUM_WHEELS; ++j)
        {
            auto wheel = wheels[j];

            if (wheel->height->_stepper->isRunning() ||
                (wheel->steer && wheel->steer->_stepper->isRunning()))
            {
                motors_running = true;
                break;
            }
        }
    } while (motors_running);
    Serial.println("Reached initial state.");
}

void AnySpace1::alignWithWall()
{
    bool left_stopped = false;
    bool right_stopped = false;

    // Start both motors
    // for (auto &wheel : wheels)
    // {
    //     wheel->drive->_stepper->runForward();
    // }
    // for (auto &wheel : wheels)
    // {
    //     if (wheel->drive)
    //     {
    //         wheel->drive->_stepper->runForward();
    //     }
    // }
    run_forward();

    while (true)
    {
        get_sensor_data();

        print_sensor_data();
        float left_distance = this->sensor_data[STAIR_WALL1];
        float right_distance = this->sensor_data[STAIR_WALL2];

        float diff = fabs(left_distance - right_distance);

        // Serial.print("Align Distance: ");
        // Serial.println(diff);
        // If left already stopped, keep right running and check difference
        if (left_stopped && !right_stopped)
        {
            if (diff < ALIGN_THRESHOLD)
            {
                wheels[FRONT_LEFT]->drive->_stepper->forceStop();
                wheels[FRONT_RIGHT]->drive->_stepper->forceStop();
                // wheels[3]->drive->_stepper->forceStop();
                right_stopped = true;
                Serial.println("Aligned with wall.");
                break;
            }
        }
        // If right already stopped, keep left running and check difference
        else if (right_stopped && !left_stopped)
        {
            if (diff < ALIGN_THRESHOLD)
            {
                wheels[FRONT_LEFT]->drive->_stepper->forceStop();
                wheels[FRONT_RIGHT]->drive->_stepper->forceStop();
                // wheels[2]->drive->_stepper->forceStop();
                left_stopped = true;
                Serial.println("Aligned with wall.");
                break;
            }
        }
        else
        {
            // Neither side stopped yet → check if one reaches initial wall threshold
            if (diff < 0 && !left_stopped)
            {
                if (left_distance < DISTANCE_TO_WALL)
                {
                    wheels[FRONT_LEFT]->drive->_stepper->runBackward();
                    Serial.println("Left wheel reached wall distance, moving back.");
                    left_stopped = true;
                }
            }

            // Right side closer → check only right
            else if (diff > 0 && !right_stopped)
            {
                if (right_distance < DISTANCE_TO_WALL)
                {
                    wheels[FRONT_RIGHT]->drive->_stepper->runBackward();
                    Serial.println("Right wheel reached wall distance, moving back.");
                    right_stopped = true;
                }
            }
            Serial.println("Aligning with wall...");
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
void AnySpace1::climb_stairs()
{
    Serial.println("Climbing stairs...");
    alignWithWall();
    approachStairs();
    //  raiseBodyToNextStair();
    //  shiftWeightForwardOntoStair();
    //  retractRearWheels();
    Serial.println("Finished climbing one stair.");
}

void AnySpace1::approachStairs()
{
    Serial.println("Approaching stairs...");
    run_forward();
    while (true)
    {
        get_sensor_data();
        print_sensor_data();
        float left_distance = this->sensor_data[STAIR_WALL1];
        if (left_distance < CLIMBABLE_DISTANCE_TO_WALL)
        {
            stop();
            Serial.println("Reached stairs.");
            break;
        }
    }
}

void AnySpace1::stopHeightMotors()
{
    for (auto &wheel : wheels)
    {
        wheel->height->_stepper->forceStop();
    }
}

void AnySpace1::raiseBodyToNextStair()
{
    go_vertically(MAX_HEIGHT);
    get_sensor_data();
    float initial_distance_reading = sensor_data[FRONT_LEFT];
    float distance_reading = sensor_data[FRONT_LEFT];
    while (true)
    {
        get_sensor_data();
        distance_reading = 0.5 * distance_reading + 0.5 * sensor_data[FRONT_LEFT];
        if (distance_reading > initial_distance_reading + NEXT_STAIR_DISTANCE)
        {
            stopHeightMotors();
            Serial.println("Body raised to next stair.");
            delay(20); // To make sure the motors have completely stopped
            go_vertically(OVERCLIMB_HEIGHT);
            go_n_steps(SAFE_DISTANCE_TO_EXTRACT_FRONT_WHEELS);
            wheels[0]->height->moveToPosition(0);
            wheels[1]->height->moveToPosition(0);
            go_n_steps(SAFE_DISTANCE_TO_EXTRACT_REAR_WHEELS);
            wheels[2]->height->moveToPosition(0);
            wheels[3]->height->moveToPosition(0);
            break;
        }
    }
}