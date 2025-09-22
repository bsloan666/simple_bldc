#include "opto_sensor.h"

OpticalSensor::OpticalSensor(int digital_pin):
    pin(digital_pin)
{
}

void OpticalSensor::initialize(){
    pinMode(pin, INPUT);
    Serial.begin(9600);

    Serial.print("OpticalSensor running on digital pin ");
    Serial.println(pin);

    delay(100);
    Serial.end();
}  

int OpticalSensor::read(){
    return digitalRead(pin);
}
