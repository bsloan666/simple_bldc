#ifndef __SINGLE_PHASE_SERVO_H__
#define __SINGLE_PHASE_SERVO_H__

#include <Arduino.h>

class SinglePhaseServo {
    
    public:
        SinglePhaseServo(int enc_pin, int sens_pin, int bridge_pin_a, int bridge_pin_b);

        int read();

    private:
        int encoder_pin;
        unsigned int value;
};
#endif
