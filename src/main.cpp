#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "mpu_handler.h"
#include <AccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <FastAccelStepper.h>
#include <AnySpace1.h>
// Adafruit_MPU6050 mpu; // for esp32-s3-n16r8 sda 8, scl 9 pin, sda 21,scl 22 for esp32
//  float pitch = 0.0, roll = 0.0;
//  // float alpha = 0; // complementary filter constant
//  unsigned long lastTime;
// #define TCAADDR 0x70

// Create two sensor objects
// Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Helper function to select the I2C channel on the multiplexer
// void tca_select(uint8_t channel)
// {
//   if (channel > 7)
//     return;

//   Wire.beginTransmission(TCAADDR);
//   Wire.write(1 << channel);
//   Wire.endTransmission();
// }

// #define drive_step_pin 14
// #define drive_dir_pin 12
// #define steer_step_pin 3
// #define steer_dir_pin 6

// AccelStepper m_drive(AccelStepper::DRIVER, drive_step_pin, drive_dir_pin);
//  AccelStepper m_steer(AccelStepper::DRIVER, steer_step_pin, steer_dir_pin);
// FastAccelStepperEngine engine = FastAccelStepperEngine();
// FastAccelStepper *m_drive = NULL;

// This function will be the dedicated task for motor control on Core 0
// void motorTask(void *pvParameters)
// {
//   Serial.println("Motor task started (High Priority)");
//   for (;;)
//   {
//     // This logic will always be prioritized
//     if (m_drive.distanceToGo() == 0)
//     {
//       m_drive.moveTo(m_drive.currentPosition() + 300);
//       // m_drive.moveTo(-m_drive.currentPosition());
//     }
//     m_drive.run();

//     // Sleep for 1ms to allow other tasks a chance to run if they need to.
//     // This is crucial to prevent watchdog resets.
//     // vTaskDelay(1);
//   }
// }

// --- Task 2: Low Priority Sensor Reading ---
// void sensorTask(void *pvParameters)
// {
//   // while (!lox.begin())
//   // {
//   //   Serial.println(F("Failed to boot VL53L0X"));
//   // }
//   // Wire.begin(8, 9);
//   // Serial.println("Sensor task started (Low Priority)");
//   VL53L0X_RangingMeasurementData_t measure; // Create a struct to hold the data
//   for (;;)
//   {
//     // This is a blocking call. It will start a measurement and wait for it to finish.
//     // This guarantees one reading per loop.
//     lox.rangingTest(&measure, false);

//     // Check if the reading was valid
//     if (measure.RangeStatus != 4)
//     {
//       // This print block is now effectively atomic for this task
//       Serial.print("Distance: ");
//       Serial.println(measure.RangeMilliMeter);
//     }
//     else
//     {
//       Serial.println("Out of range");
//     }

// This task can sleep for longer as it's less critica

// void setup()
// {
//   Serial.begin(115200);
//   // while (!Serial)
//   //   ; // Wait for serial port to connect. Needed for native USB
//   // Serial.println("Steer and drive demo");
//   // mpu.begin();
//   Wire.begin();

//   // // Initialize the sensor
//   if (!lox.begin())
//   {
//     Serial.println(F("Failed to boot VL53L0X"));
//     while (1)
//       ;
//   }
//   // m_steer.moveTo(1600);         // Set an initial target position (e.g., 1600 steps)
//   // m_steer.setMaxSpeed(1000);    // Set maximum speed
//   // m_steer.setAcceleration(500); // Set acceleration
//   m_drive.moveTo(300);           // Set an initial target position (e.g., 3200 steps)
//   m_drive.setMaxSpeed(500);      // Set maximum speed
//   m_drive.setAcceleration(2000); // Set acceleration

//   lox.startRangeContinuous(1000);
// }

// unsigned long lastReadingTime = 0;
// const long readingInterval = 2000; // Read the sensor every 100 milliseconds (10 times/sec)

AnySpace1 cleaning_robot;
void setup()
{
  cleaning_robot.begin();
}

void loop()
{
  cleaning_robot.run();
}
