#include "opto_sensor.h"

OpticalSensor::OpticalSensor(int digital_pin):
    pin(digital_pin)
{
    // initialization if any
}

int OpticalSensor::read(){
    return digitalRead(pin);
}
