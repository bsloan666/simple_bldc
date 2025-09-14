#ifndef __ABS_ENCODER_H__
#define __ABS_ENCODER_H__

#include <Arduino.h>

class AbsoluteRadialEncoder {
    
    public:
        AbsoluteRadialEncoder(int analog_pin);

        int read();

    private:
        int pin;
        unsigned int value;
};
#endif
