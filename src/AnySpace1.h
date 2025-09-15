#ifndef ANYSPACE1_H
#define ANYSPACE1_H
#include "ArticulatedWheel.h"
#include <Adafruit_MPU6050.h>
#include "Adafruit_VL53L0X.h"
#include "robot_config.h"
/**
 * @class AnySpace1
 * @brief Represents the entire robot, managing all its components.
 */
class AnySpace1
{
public:
    AnySpace1();

    void begin();
    void run();
    // for drive movement, could be backwards could be forward
    void go_n_steps(float n_steps);
    // basically for drive direction
    void change_direction();
    // for height direction. Could be positive could be negative
    void go_vertically(float n_steps);
    // for steering direction. Could be positive could be negative
    void turn_wheel(float n_steps);
    // get data from all sensors
    void get_sensor_data();
    // get data from mpu6050
    std::tuple<float, float> get_mpu_data();
    // get data from vl53l0x sensors maybe 8 bit?
    std::array<uint16_t, NUM_SENSORS> get_vl53l0x_data();
    void home();

private:
    Adafruit_MPU6050 mpu;
    FastAccelStepperEngine m_engine; // The robot owns the single stepper engine

    std::array<ArticulatedWheel, NUM_WHEELS> wheels;
    std::array<Adafruit_VL53L0X, NUM_SENSORS> distance_sensors;
    std::array<uint16_t, NUM_SENSORS> sensor_data;
};

#endif