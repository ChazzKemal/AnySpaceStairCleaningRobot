#ifndef ANYSPACE1_H
#define ANYSPACE1_H
#include "ArticulatedWheel.h"
#include <Adafruit_MPU6050.h>
#include "Adafruit_VL53L0X.h"

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

private:
    ArticulatedWheel front_left_wheel;
    ArticulatedWheel front_right_wheel;
    ArticulatedWheel rear_left_wheel;
    ArticulatedWheel rear_right_wheel;
    Adafruit_MPU6050 mpu;
    Adafruit_VL53L0X distance_floor_below;
    Adafruit_VL53L0X distance_stair_wall1;
    Adafruit_VL53L0X distance_stair_wall2;
    Adafruit_VL53L0X distance_front;
    Adafruit_VL53L0X distance_rear;
};

#endif