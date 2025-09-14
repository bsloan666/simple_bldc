#include "abs_encoder.h"

AbsoluteRadialEncoder::AbsoluteRadialEncoder(int analog_pin):
    pin(analog_pin)
{
    // initialization if any
}

int AbsoluteRadialEncoder::read(){
    return analogRead(pin);
}
