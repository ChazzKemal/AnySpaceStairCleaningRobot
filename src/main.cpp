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

AnySpace1 cleaning_robot;
void setup()
{
  cleaning_robot.begin();
}

void loop()
{
  cleaning_robot.run();
}
