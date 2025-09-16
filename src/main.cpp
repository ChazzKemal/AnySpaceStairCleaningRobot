#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "mpu_handler.h"
#include "distance_handler.h"
#include <AccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <FastAccelStepper.h>
#include <AnySpace1.h>
#include <robot_config.h>
#include "vacuumUnit.h"
#include <Adafruit_ADS1X15.h>

#define TCAADDR 0x70
AnySpace1 cleaning_robot;
Vacuum *vacUnit;
// Adafruit_ADS1115 * ads;

// ArticulatedWheel *wheel1;
// FastAccelStepperEngine *m_engine = new FastAccelStepperEngine();
// FastAccelStepper *m_drive = NULL;

// FastAccelStepperEngine *engine = new FastAccelStepperEngine();
// FastAccelStepper *m_drive = NULL;
// ArticulatedWheel *wheel1;
// Stepper *stepper;

void setup()
{

  // Serial.begin(115200);
  // Wire.begin();
  // delay(500);

  // Serial.println("start code");

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
  // if (!lox1.begin())
  // {
  //   Serial.println(F("Failed to boot VL53L0X on channel 1"));
  //   while (1)
  //     ;
  cleaning_robot.begin();
}

// engine->init();

// wheel1 = new ArticulatedWheel(engine, FL_DRIVE_STEP_PIN, FL_DRIVE_DIR_PIN,
//                               FL_STEER_STEP_PIN, FL_STEER_DIR_PIN,
//                               FL_HEIGHT_STEP_PIN, FL_HEIGHT_DIR_PIN, FL_HOME_PIN,
//                               false, false, false);

// wheel1->begin();
cleaning_robot.begin();
cleaning_robot.home();
vacUnit = new Vacuum();
// m_drive = engine.stepperConnectToPin(FL_DRIVE_STEP_PIN);
// m_drive->setDirectionPin(FL_DRIVE_DIR_PIN, /*dir_high_is_forward=*/true);

// m_drive->setSpeedInHz(400);     // max step rate (steps/second)
// m_drive->setAcceleration(2000); // steps/second^2

// m_drive->moveTo(2000);
// ads   = new Adafruit_ADS1115();
// ads->begin(0x48)  ;

// ads->setGain(GAIN_ONE);
}

void loop()
{

  // Serial.println(ads->readADC_SingleEnded(0));
  vacUnit->turnOnBrizzles();
  vacUnit->turnOnVacuum();
  delay(500);
  vacUnit->turnOffBrizzles();
  vacUnit->turnOffVacuum();
  delay(500);

  // cleaning_robot.run();
  //  wheel1->drive->moveToPosition(300);
  //  delay(1000);
  //  wheel1->drive->moveToPosition(-300);
  //  delay(1000);
  // cleaning_robot.get_sensor_data();
  // cleaning_robot.print_sensor_data();
  cleaning_robot.run();
}
