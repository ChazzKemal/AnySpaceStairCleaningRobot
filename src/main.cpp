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

AnySpace1 cleaning_robot;
// ArticulatedWheel *wheel1;
// FastAccelStepperEngine *m_engine = new FastAccelStepperEngine();
// FastAccelStepper *m_drive = NULL;

// FastAccelStepperEngine *engine = new FastAccelStepperEngine();
// FastAccelStepper *m_drive = NULL;
// ArticulatedWheel *wheel1;
// Stepper *stepper;

void setup()
{

  Serial.begin(115200);

  delay(500);

  Serial.println("start code");

  // engine->init();

  // wheel1 = new ArticulatedWheel(engine, FL_DRIVE_STEP_PIN, FL_DRIVE_DIR_PIN,
  //                               FL_STEER_STEP_PIN, FL_STEER_DIR_PIN,
  //                               FL_HEIGHT_STEP_PIN, FL_HEIGHT_DIR_PIN, FL_HOME_PIN,
  //                               false, false, false);

  // wheel1->begin();
  cleaning_robot.begin();
  cleaning_robot.home();
  // m_drive = engine.stepperConnectToPin(FL_DRIVE_STEP_PIN);
  // m_drive->setDirectionPin(FL_DRIVE_DIR_PIN, /*dir_high_is_forward=*/true);

  // m_drive->setSpeedInHz(400);     // max step rate (steps/second)
  // m_drive->setAcceleration(2000); // steps/second^2

  // m_drive->moveTo(2000);
}

void loop()
{
  // cleaning_robot.run();
  //  wheel1->drive->moveToPosition(300);
  //  delay(1000);
  //  wheel1->drive->moveToPosition(-300);
  //  delay(1000);
  //  cleaning_robot.run();
}
