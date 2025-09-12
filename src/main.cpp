#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "mpu_handler.h"

Adafruit_MPU6050 mpu;
float pitch = 0.0, roll = 0.0;
// float alpha = 0; // complementary filter constant
unsigned long lastTime;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial port to connect. Needed for native USB
  Serial.println("MPU6050 OLED demo");

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
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.println("Found a MPU-6050 sensor");
  lastTime = millis();
}

void loop()
{
  // put your main code here, to run repeatedly:
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);

  // unsigned long now = millis();
  // float dt = (now - lastTime) / 1000.0; // seconds
  // lastTime = now;

  float accPitch, accRoll;
  std::tie(accPitch, accRoll) = get_angles();

  // Serial.print("Acceleration X: ");
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

  Serial.println("");
  Serial.print("Pitch: ");
  Serial.print(accPitch);
  Serial.print(", Roll: ");
  Serial.println(accRoll);
  delay(500);
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}