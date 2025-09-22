#include "abs_encoder.h"

AbsoluteRadialEncoder::AbsoluteRadialEncoder(int analog_pin):
    pin(analog_pin)
{
}

void AbsoluteRadialEncoder::initialize(){
    Serial.begin(9600);

    Serial.print("AbsoluteRadialEncoder running on analog pin ");
    Serial.println(pin);

    delay(100);
    Serial.end();
}

int AbsoluteRadialEncoder::read(){
    return analogRead(pin);
}
