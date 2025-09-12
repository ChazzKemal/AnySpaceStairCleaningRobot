#ifndef MPU_HANDLER_H
#define MPU_HANDLER_H
#include <Adafruit_MPU6050.h>
#include <tuple>

// Call this once in your main setup()
// void setup_mpu();

// Call this repeatedly in your main loop()
std::tuple<float, float> get_angles(Adafruit_MPU6050 &mpu);

#endif // MPU_HANDLER_H