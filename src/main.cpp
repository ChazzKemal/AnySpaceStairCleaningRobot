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
#include <robot_config.h>

// AnySpace1 cleaning_robot;
ArticulatedWheel *wheel1;
FastAccelStepperEngine *m_engine = new FastAccelStepperEngine();
FastAccelStepper *m_drive = NULL;

//  ------------------ Speeds   (speed in steps)

#define HEIGHT_SPEED 100
#define DRIVE_SPEED 100
#define STEER_SPEED 100

//  ------------------Accelerations

#define HEIGHT_ACC 1000
#define DRIVE_ACC 1000
#define STEER_ACC 1000

void setup()
{

  m_engine->init();
  wheel1 = new ArticulatedWheel(m_engine, FL_DRIVE_STEP_PIN, FL_DRIVE_DIR_PIN, FL_STEER_STEP_PIN, FL_STEER_DIR_PIN, FL_HEIGHT_STEP_PIN, FL_HEIGHT_DIR_PIN, FL_HOME_PIN, FL_INVERT_DRIVE, FL_INVERT_STEER, FL_INVERT_HEIGHT); // Remove trailing comma
  wheel1->begin(DRIVE_SPEED, STEER_SPEED, HEIGHT_SPEED, DRIVE_ACC, STEER_ACC, HEIGHT_ACC);
  //  cleaning_robot.begin();
  //  cleaning_robot.run();
  // m_drive = engine.stepperConnectToPin(FL_DRIVE_STEP_PIN);
  // m_drive->setDirectionPin(FL_DRIVE_DIR_PIN, /*dir_high_is_forward=*/true);

  // m_drive->setSpeedInHz(400);     // max step rate (steps/second)
  // m_drive->setAcceleration(2000); // steps/second^2

  // m_drive->moveTo(2000);
  wheel1->drive->moveToPosition(300);
}

void loop()
{
  // cleaning_robot.run();
}
