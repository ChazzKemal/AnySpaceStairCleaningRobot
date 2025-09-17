#include "ArticulatedWheel.h"
#include <Arduino.h>
#include <robot_config.h>
#include <Adafruit_ADS1X15.h>
// The constructor now takes pin numbers and initializes the AccelStepper objects directly.
ArticulatedWheel::ArticulatedWheel(FastAccelStepperEngine *engine, Adafruit_ADS1115 *_ads , uint8_t drive_step_pin, uint8_t drive_dir_pin,
                                   uint8_t steer_step_pin, uint8_t steer_dir_pin,
                                   uint8_t height_step_pin, uint8_t height_dir_pin, uint8_t _homingPin,
                                   bool invert_drive, bool invert_steer, bool invert_height,bool has_drive, bool has_steer)
{
    drive = has_drive ? new Stepper(engine, drive_step_pin, drive_dir_pin, invert_drive) : nullptr;
    steer = has_steer ? new Stepper(engine, steer_step_pin, steer_dir_pin, invert_steer) : nullptr;
    //drive = new Stepper(engine, drive_step_pin, drive_dir_pin, invert_drive);
    //steer = new Stepper(engine, steer_step_pin, steer_dir_pin, invert_steer);
    height = new Stepper(engine, height_step_pin, height_dir_pin, invert_height);
    homingPin = _homingPin;
    ads = _ads;


}

void ArticulatedWheel::begin(float max_drive_speed,
                             float max_steer_speed,
                             float max_height_speed,
                             float acceleration_drive,
                             float acceleration_steer,
                             float acceleration_height)
{ // Those are the Default Speeds
    if (drive) {
    drive->init(max_drive_speed, acceleration_drive, CONVERSION_FACTOR_DRIVE);}
    if (steer) {
    steer->init(max_steer_speed, acceleration_steer, CONVERSION_FACTOR_STEER);}
    height->init(max_height_speed, acceleration_height, CONVERSION_FACTOR_HEIGHT);
    
}

bool ArticulatedWheel::checkHomingPin()
{
    if (ads->readADC_SingleEnded(homingPin)>=10000){return true   ;}
    return false;

}

Stepper::Stepper(FastAccelStepperEngine *engine, uint8_t step_pin, uint8_t dir_pin, bool invert)
{
    _stepper = engine->stepperConnectToPin(step_pin);
    invertDir = invert;
    if (_stepper)
    {
        _stepper->setDirectionPin(dir_pin);
        Serial.println("initialization worked");
    }
    else
    {

        Serial.println("Stepper initialization failed");
    }
}

void Stepper::init(float speed, float acceleration, float conversionfact)
{
    default_Speed = speed;
    default_acceleration = acceleration;
    conversionFactor = conversionfact;
    position = 0;
    _stepper->setAcceleration(acceleration); // steps/s^2
    _stepper->setSpeedInHz(speed);           // steps/s
}

void Stepper::moveSteps(int steps)
{

    _stepper->move(steps);
}

int Stepper::convertToSteps(float moveBy)
{
    return (int)conversionFactor * moveBy;
}

void Stepper::moveToPosition(float newPosition)
{
    moveSteps(convertToSteps(newPosition - position));
    position = newPosition;
}

void Stepper::moveRelative(float relPos)
{
    moveSteps(convertToSteps(relPos));
    position += relPos;
}
