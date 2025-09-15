#include "ArticulatedWheel.h"

// The constructor now takes pin numbers and initializes the AccelStepper objects directly.
ArticulatedWheel::ArticulatedWheel(FastAccelStepperEngine* engine,uint8_t drive_step_pin, uint8_t drive_dir_pin,
                                   uint8_t steer_step_pin, uint8_t steer_dir_pin,
                                   uint8_t height_step_pin, uint8_t height_dir_pin,
                                   bool invert_drive, bool invert_steer, bool invert_height)
{
    drive =  new Stepper(engine,drive_step_pin,drive_dir_pin,invert_drive);
    steer =  new Stepper(engine,steer_step_pin,steer_dir_pin,invert_steer);
    height =  new Stepper(engine,height_step_pin,height_dir_pin,invert_height);
}

void ArticulatedWheel::begin(float max_drive_speed,
                             float max_steer_speed,
                             float max_height_speed,
                             float acceleration_drive,
                             float acceleration_steer,
                             float acceleration_height)
{   // Those are the Default Speeds
    drive->init(max_drive_speed,acceleration_drive,1);
    steer->init(max_steer_speed,acceleration_steer,1);
    height->init(max_height_speed,acceleration_height,1);
}





Stepper::Stepper(FastAccelStepperEngine* engine,uint8_t step_pin, uint8_t dir_pin,bool invert){
    _stepper = engine->stepperConnectToPin(step_pin);
    invertDir = invert;
    if (_stepper){
        _stepper->setDirectionPin(dir_pin);
        Serial.println("initialization worked");
    }
    else{
        
        Serial.println("Stepper initialization failed");
    
    }
}

void Stepper::init(float speed,float acceleration,float conversionfact){
    default_Speed = speed;
    default_acceleration = acceleration;
    conversionFactor    = conversionfact;
    _stepper->setAcceleration(acceleration); // steps/s^2
    _stepper->setSpeedInHz(speed);     // steps/s

}

void Stepper:: moveSteps(int steps){
    Serial.println("doing steps ");
 
    _stepper->move(steps);
}

int Stepper::convertToSteps(float moveBy){
    return conversionFactor * moveBy;
}