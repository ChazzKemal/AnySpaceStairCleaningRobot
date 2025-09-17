#ifndef DISTANCE_HANDLER_H
#define DISTANCE_HANDLER_H
#include <stdint.h>
#include <array>
#include "robot_config.h"
#include "Adafruit_VL53L0X.h"

void tca_select(uint8_t channel);

std::array<float, NUM_SENSORS> get_vl53l0x_data(Adafruit_VL53L0X &sensors,
                                                std::array<float, NUM_SENSORS> &sensor_data);

#endif // DISTANCE_HANDLER_H