#include "distance_handler.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define TCAADDR 0x70
void tca_select(uint8_t channel)
{
    if (channel > 7)
        return;

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}
std::array<uint16_t, NUM_SENSORS> get_vl53l0x_data(Adafruit_VL53L0X &lox1,
                                                   std::array<uint16_t, NUM_SENSORS> &sensor_data)
{
    // The function assumes Wire.begin() because there is a sensors class

    VL53L0X_RangingMeasurementData_t measure;
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        // Select the correct sensor on the I2C multiplexer
        tca_select(i);

        // Take a reading from the sensor in the array
        lox1.rangingTest(&measure, false);

        // Check if the reading was valid
        if (measure.RangeStatus != 4)
        {
            sensor_data[i] = measure.RangeMilliMeter;
        }
        else
        {
            // Use a special value (like 0 or max uint16_t) to indicate an error or "out of range"
            // sensor_data[i] = 0;
            sensor_data[i] = 8190; // 8190 is the max range for VL53L0X
        }
    }
    return sensor_data;
}
