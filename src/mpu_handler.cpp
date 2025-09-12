#include "mpu_handler.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <tuple>

// Sensor and filter variables are now private to this file
static Adafruit_MPU6050 mpu;
static float pitch = 0.0;
static float roll = 0.0;

std::tuple<float, float> get_angles()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate angles from accelerometer (in degrees)
    float accPitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
    float accRoll = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
    return std::make_tuple(accPitch, accRoll);
}
