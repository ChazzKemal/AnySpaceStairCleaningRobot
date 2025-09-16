

#include "vacuumUnit.h"
#include "config.h"
#include "Arduino.h"




Vacuum::Vacuum(){
 pinMode(VACUUM_PIN,OUTPUT);
 pinMode(BRIZZLES_PIN,OUTPUT);  
 digitalWrite(VACUUM_PIN,HIGH);
 digitalWrite(BRIZZLES_PIN,HIGH); 
}

void Vacuum::turnOnBrizzles() {
    digitalWrite(BRIZZLES_PIN, LOW);
}

void Vacuum::turnOffBrizzles() {
    digitalWrite(BRIZZLES_PIN, HIGH);
}

void Vacuum::turnOnVacuum() {
    digitalWrite(VACUUM_PIN, LOW);
}

void Vacuum::turnOffVacuum() {
    digitalWrite(VACUUM_PIN, HIGH);
}