#ifndef ANYSPACE1_H
#define ANYSPACE1_H
#include "ArticulatedWheel.h"
#include <Adafruit_MPU6050.h>
#include "Adafruit_VL53L0X.h"
#include "robot_config.h"
#include <Adafruit_ADS1X15.h>
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
    void run_forward();
    void run_backward();
    void stop();
    void go_n_steps(float n_steps);
    // basically for drive direction
    void reverse_direction();
    // for height direction. Could be positive could be negative
    void go_vertically(float n_steps);
    // for steering direction. Could be positive could be negative
    void steer_wheel(float n_steps);
    // get data from all sensors
    void get_sensor_data();
    // get data from mpu6050
    std::tuple<float, float> get_mpu_data();
    // get data from vl53l0x sensors TODO: maybe 8 bit?
    std::array<uint16_t, NUM_SENSORS> get_vl53l0x_data();
    void print_sensor_data();
    void go_initial_state();
    void align_with_wall();
    void home();

private:
    Adafruit_MPU6050 mpu;
    Adafruit_ADS1115 * ads ;
    FastAccelStepperEngine *m_engine = new FastAccelStepperEngine();
    std::array<ArticulatedWheel *, NUM_WHEELS> wheels; // Changed to array of pointers
    // std::array<ArticulatedWheel, NUM_WHEELS> wheels;
    Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
    // std::array<Adafruit_VL53L0X, NUM_SENSORS> distance_sensors;
    std::array<uint16_t, NUM_SENSORS> sensor_data;
    float pitch = 0.0f;
    float roll = 0.0f;
    
};

#endif