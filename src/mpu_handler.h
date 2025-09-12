#ifndef MPU_HANDLER_H
#define MPU_HANDLER_H

// Call this once in your main setup()
// void setup_mpu();

// Call this repeatedly in your main loop()
std::tuple<float, float> get_angles();

#endif // MPU_HANDLER_H