#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "mpu_handler.h"
#include <AccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <FastAccelStepper.h>
// Adafruit_MPU6050 mpu; // for esp32-s3-n16r8 sda 8, scl 9 pin
//  float pitch = 0.0, roll = 0.0;
//  // float alpha = 0; // complementary filter constant
//  unsigned long lastTime;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
SemaphoreHandle_t serialMutex;

#define drive_step_pin 5
#define drive_dir_pin 4
#define steer_step_pin 3
#define steer_dir_pin 6

// AccelStepper m_drive(AccelStepper::DRIVER, drive_step_pin, drive_dir_pin);
//  AccelStepper m_steer(AccelStepper::DRIVER, steer_step_pin, steer_dir_pin);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *m_drive = NULL;

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
void sensorTask(void *pvParameters)
{
  // while (!lox.begin())
  // {
  //   Serial.println(F("Failed to boot VL53L0X"));
  // }
  // Wire.begin(8, 9);
  // Serial.println("Sensor task started (Low Priority)");
  VL53L0X_RangingMeasurementData_t measure; // Create a struct to hold the data
  for (;;)
  {
    // This is a blocking call. It will start a measurement and wait for it to finish.
    // This guarantees one reading per loop.
    lox.rangingTest(&measure, false);

    // Check if the reading was valid
    if (measure.RangeStatus != 4)
    {
      // This print block is now effectively atomic for this task
      Serial.print("Distance: ");
      Serial.println(measure.RangeMilliMeter);
    }
    else
    {
      Serial.println("Out of range");
    }

    // This task can sleep for longer as it's less critical
    vTaskDelay(100);
  }
}



unsigned long lastReadingTime = 0;
const long readingInterval = 2000; // Read the sensor every 100 milliseconds (10 times/sec)

void setup()
{ 
  // digitalWrite(drive_step_pin, LOW);
  // digitalWrite(drive_dir_pin, LOW);
  Serial.begin(115200);
  
  engine.init();
  


  m_drive = engine.stepperConnectToPin(drive_step_pin);
  if (m_drive)
  {
    m_drive->setDirectionPin(drive_dir_pin);
    m_drive->setAcceleration(1000); // steps/s^2
    m_drive->setSpeedInHz(100);     // steps/s

  }


}

void loop()
{
  Serial.println("Running forward...");
  m_drive->runForward(); // Start running forward
  delay(2000);           // Wait for 2 seconds

  Serial.println("Running backward...");
  m_drive->runBackward(); // Start running backward
  delay(2000);            // Wait for 2 seconds
    // m_drive->runForward();
    // m_drive->move(300);
    // m_drive->runBackward();
    // m_drive->move(300);


}
