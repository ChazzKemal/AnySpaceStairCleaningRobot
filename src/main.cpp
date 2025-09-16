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

//     // This task can sleep for longer as it's less critical
//     vTaskDelay(100);
//   }
// }

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

void setup()
{
  AnySpace1 cleaning_robot;
  // Serial.begin(115200);
  // while (!Serial)
  // {
  //   delay(1); // Wait for serial port to connect
  // }
  // Serial.println("Dual VL53L0X Test with TCA9548A Multiplexer");

  // // Initialize I2C bus
  // Wire.begin();

  // // --- Initialize Sensor 1 on Channel 0 ---
  // tca_select(0);
  // if (!lox1.begin())
  // {
  //   Serial.println(F("Failed to boot VL53L0X on channel 0"));
  //   while (1)
  //     ;
  // }
  // Serial.println(F("VL53L0X 1 booted on channel 0"));

  // // --- Initialize Sensor 2 on Channel 1 ---
  // tca_select(1);
  // if (!lox2.begin())
  // {
  //   Serial.println(F("Failed to boot VL53L0X on channel 1"));
  //   while (1)
  //     ;
  // }
  // Serial.println(F("VL53L0X 2 booted on channel 1"));
  // delay(1000);
  // engine.init();
  // Serial.begin(115200);
  // // delay(1000);
  // Wire.begin(8, 9); // Use correct I2C pins for your board
  // if (!lox.begin())
  // {
  //   Serial.println(F("Failed to boot VL53L0X"));
  //   while (1)
  //     ; // Halt if sensor is not found
  // }
  // --- Sensor and Motor Configuration ---
  // while (!lox.begin())
  // {
  //   Serial.println(F("Failed to boot VL53L0X"));
  // }
  // lox.begin();
  // m_drive.moveTo(300);
  // m_drive.setMaxSpeed(500);
  // m_drive.setAcceleration(5000);

  // Create the stepper object from the engine
  // m_drive = engine.stepperConnectToPin(drive_step_pin);
  // if (m_drive)
  // {
  //   m_drive->setDirectionPin(drive_dir_pin);
  //   m_drive->setAcceleration(1000); // steps/s^2
  //   m_drive->setSpeedInHz(500);     // steps/s
  //   // m_drive->runForward();
  //   // m_drive->move(300);
  // }

  // --- Task Creation ---
  // Get the handle for the current task (the setup/loop task)
  // TaskHandle_t loopTaskHandle = xTaskGetCurrentTaskHandle();
  // // // // Get the priority of the loop task
  // UBaseType_t loopTaskPriority = uxTaskPriorityGet(loopTaskHandle);

  // Create the motor task with a HIGHER priority than the loop
  // xTaskCreatePinnedToCore(
  //     motorTask,
  //     "MotorTask",
  //     4096,
  //     NULL,
  //     loopTaskPriority + 1, // Higher priority
  //     NULL,
  //     0); // Pin to Core 1 (Arduino core)

  // Create the sensor task with a LOWER priority than the loop
  // xTaskCreatePinnedToCore(
  //     sensorTask,
  //     "SensorTask",
  //     4096,
  //     NULL,
  //     loopTaskPriority, // Lower priority
  //     NULL,
  //     0); // Pin to Core 1 (Arduino core)
}

void loop()
{

  // VL53L0X_RangingMeasurementData_t measure;

  // // --- Read from Sensor 1 ---
  // tca_select(0);
  // lox1.rangingTest(&measure, false); // false = not in debug mode

  // if (measure.RangeStatus != 4)
  // { // phase failures have incorrect data
  //   Serial.print("Sensor 1 Distance (mm): ");
  //   Serial.println(measure.RangeMilliMeter);
  // }
  // else
  // {
  //   Serial.println("Sensor 1 out of range");
  // }

  // // --- Read from Sensor 2 ---
  // tca_select(1);
  // lox2.rangingTest(&measure, false);

  // if (measure.RangeStatus != 4)
  // {
  //   Serial.print("Sensor 2 Distance (mm): ");
  //   Serial.println(measure.RangeMilliMeter);
  // }
  // else
  // {
  //   Serial.println("Sensor 2 out of range");
  // }

  // Serial.println("--------------------");
  // delay(500); // Wait half a second between measurement sets
  // Serial.println("Running forward...");
  // m_drive->runForward(); // Start running forward
  // delay(2000);           // Wait for 2 seconds

  // Serial.println("Running backward...");
  // m_drive->runBackward(); // Start running backward
  // delay(2000);            // Wait for 2 seconds
  // if (!m_drive->isRunning())
  // {
  //   // Move back to the starting position (0) or to a new position (3000)
  //   long new_pos = m_drive->getCurrentPosition() == 0 ? 3000 : 0;
  //   m_drive->moveTo(new_pos);
  // }

  // if (millis() - lastReadingTime >= readingInterval)
  // {
  //   lastReadingTime = millis(); // Update the last reading time

  //   VL53L0X_RangingMeasurementData_t measure;
  //   lox.rangingTest(&measure, false); // This is a blocking call

  //   if (measure.RangeStatus != 4)
  //   {
  //     Serial.print("Distance: ");
  //     Serial.println(measure.RangeMilliMeter);
  //   }
  //   else
  //   {
  //     Serial.println("Out of range");
  //   }
  // }
  // --- Sensor Reading Logic ---
  // Use a non-blocking timer to read the sensor periodically
  // unsigned long currentTime = millis();
  // if (currentTime - lastReadingTime >= readingInterval)
  // {
  //   lastReadingTime = currentTime;
  //   m_drive.moveTo(-m_drive.currentPosition());
  // }
  // m_steer.run();
  // if (m_drive.distanceToGo() == 0)
  // {
  //   // If it has arrived, set a new target in the opposite direction.
  //   // If the current position is 1600, the new target will be -1600.
  //   // If the current position is -1600, the new target will be 1600.
  //   m_drive.moveTo(-m_drive.currentPosition());
  // }
  // m_drive.run();
  // if (lox.isRangeComplete())
  // {

  //   // 2. If it is ready, get the data. This is very fast.
  //   int distance = lox.readRangeResult();

  //   Serial.print("Distance: ");
  //   Serial.println(distance);
  // }
  // The main loop can be left empty, as all work is now done in tasks.
  // Or it can be used for the absolute lowest priority work.
  // vTaskDelay(1000); // Don't let the loop spin without purpose
}

// void loop()
// {

//   if (m_drive.distanceToGo() == 0)
//   {
//     // If it has arrived, set a new target in the opposite direction.
//     // If the current position is 1600, the new target will be -1600.
//     // If the current position is -1600, the new target will be 1600.
//     m_drive.moveTo(-m_drive.currentPosition());
//   }
//   // m_steer.run();
//   m_drive.run();
//   //  float accPitch, accRoll;
//   //  std::tie(accPitch, accRoll) = get_angles(mpu);

//   // // uncomment below
//   // Serial.println("");
//   // Serial.print("Pitch: ");
//   // Serial.print(accPitch);
//   // Serial.print(", Roll: ");
//   // Serial.println(accRoll);

//   // Serial.print("Reading a measurement... ");
//   // Check if it's time to take a new sensor reading
//   if (lox.isRangeComplete())
//   {

//     // 2. If it is ready, get the data. This is very fast.
//     int distance = lox.readRangeResult();

//     // Serial.print("Distance: ");
//     // Serial.println(distance);
//   }
//   // if (measure.RangeStatus != 4)
//   // { // phase failures have incorrect data
//   //   Serial.print("Distance (mm): ");
//   //   Serial.println(measure.RangeMilliMeter);
//   // }
//   // else
//   // {
//   //   Serial.println(" out of range ");
//   // }
// }
