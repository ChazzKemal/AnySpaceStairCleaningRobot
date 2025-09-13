#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "mpu_handler.h"
#include <AccelStepper.h>

Adafruit_MPU6050 mpu; // for esp32-s3-n16r8 sda 8, scl 9 pin
//  float pitch = 0.0, roll = 0.0;
//  // float alpha = 0; // complementary filter constant
//  unsigned long lastTime;

#define drive_step_pin 13
#define drive_dir_pin 14
#define steer_step_pin 3
#define steer_dir_pin 6

AccelStepper m_drive(AccelStepper::DRIVER, drive_step_pin, drive_dir_pin);
//  AccelStepper m_steer(AccelStepper::DRIVER, steer_step_pin, steer_dir_pin);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial port to connect. Needed for native USB
  Serial.println("Steer and drive demo");

  // while (!mpu.begin())
  // {
  //   Serial.println("Sensor init failed, retrying...");
  //   Serial.println(mpu.begin()); // This would be redundant
  //   delay(1000);
  // }

  // if (!mpu.begin())
  // {
  //   Serial.println("Sensor init failed");
  //   while (1)
  //     yield();
  // }

  mpu.begin();
  //  sensors_event_t a, g, temp;
  //  mpu.getEvent(&a, &g, &temp);
  //  Serial.println("Found a MPU-6050 sensor");
  //  lastTime = millis();
  // m_steer.moveTo(1600);         // Set an initial target position (e.g., 1600 steps)
  // m_steer.setMaxSpeed(1000);    // Set maximum speed
  // m_steer.setAcceleration(500); // Set acceleration
  m_drive.moveTo(1600);          // Set an initial target position (e.g., 3200 steps)
  m_drive.setMaxSpeed(1000);     // Set maximum speed
  m_drive.setAcceleration(3000); // Set acceleration
}

void loop()
{
  // m_steer.run();
  m_drive.run();
  // put your main code here, to run repeatedly:
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);

  // unsigned long now = millis();
  // float dt = (now - lastTime) / 1000.0; // seconds
  // lastTime = now;

  float accPitch, accRoll;
  std::tie(accPitch, accRoll) = get_angles(mpu);

  // Serial.print(a.acceleration.x);
  // Serial.print(", Y: ");
  // Serial.print(a.acceleration.y);
  // Serial.print(", Z: ");
  // Serial.print(a.acceleration.z);
  // Serial.println(" m/s^2");

  // pitch += g.gyro.x * dt * 180 / PI;
  // roll += g.gyro.y * dt * 180 / PI;

  // pitch = alpha * pitch + (1 - alpha) * accPitch;
  // roll = alpha * roll + (1 - alpha) * accRoll;

  // Serial.print("Rotation X: ");
  // Serial.print(g.gyro.x);
  // Serial.print(", Y: ");
  // Serial.print(g.gyro.y);
  // Serial.print(", Z: ");
  // Serial.print(g.gyro.z);
  // Serial.println(" rad/s");

  // Serial.print("Temperature: ");
  // Serial.print(temp.temperature);
  // Serial.println(" degC");

  // uncomment below
  Serial.println("");
  Serial.print("Pitch: ");
  Serial.print(accPitch);
  Serial.print(", Roll: ");
  Serial.println(accRoll);
}
