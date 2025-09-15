#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "mpu_handler.h"
#include <AccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <FastAccelStepper.h>
#include "ArticulatedWheel.h"
// Adafruit_MPU6050 mpu; // for esp32-s3-n16r8 sda 8, scl 9 pin
//  float pitch = 0.0, roll = 0.0;
//  // float alpha = 0; // complementary filter constant
//  unsigned long lastTime;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
SemaphoreHandle_t serialMutex;

// -------------WHEEL 1 --------------------
#define DRIVE_STEP_PIN 5
#define DRIVE_DIR_PIN 4
#define HEIGHT_STEP_PIN 7
#define HEIGHT_DIR_PIN 6
#define STEER_STEP_PIN 16
#define STEER_DIR_PIN 15




//  ------------------ Speeds   (speed in steps)

#define HEIGHT_SPEED 100
#define DRIVE_SPEED 100
#define STEER_SPEED 100 

//  ------------------Accelerations 


#define HEIGHT_ACC 1000
#define DRIVE_ACC 1000
#define STEER_ACC 1000



// AccelStepper m_drive(AccelStepper::DRIVER, DRIVE_STEP_PIN, DRIVE_DIR_PIN);
//  AccelStepper m_steer(AccelStepper::DRIVER, steer_step_pin, steer_dir_pin);
FastAccelStepperEngine *engine = new FastAccelStepperEngine();
FastAccelStepper *m_drive = NULL;
ArticulatedWheel *wheel1;
Stepper *stepper;
// This function will be the dedicated task for motor control on Core 
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
  // digitalWrite(DRIVE_STEP_PIN, LOW);
  // digitalWrite(DRIVE_DIR_PIN, LOW);
  Serial.begin(115200);
  delay(500);

  Serial.println("start code");


  
  engine->init();

  wheel1 = new ArticulatedWheel(engine, DRIVE_STEP_PIN, DRIVE_DIR_PIN,
                            STEER_STEP_PIN, STEER_DIR_PIN,
                            HEIGHT_STEP_PIN, HEIGHT_DIR_PIN,
                            false, false, false);

  

  wheel1->begin(DRIVE_SPEED,STEER_SPEED,HEIGHT_SPEED,DRIVE_ACC,STEER_ACC,HEIGHT_ACC);
  // stepper = new Stepper(engine,DRIVE_STEP_PIN,DRIVE_DIR_PIN,false);
  // stepper->init(DRIVE_SPEED,STEER_SPEED,1);
}

void loop()
{
  Serial.println("loop");
  wheel1->drive->moveSteps(300);
  delay(1000);
  wheel1->drive->moveSteps(-300);
  delay(1000);
  // stepper->moveSteps(30);
  // delay(800);
  // stepper->moveSteps(-30);
  // delay(800);
  



}
