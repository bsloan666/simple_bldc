#ifndef __ABS_ENCODER_H__
#define __ABS_ENCODER_H__

#include <Arduino.h>

class AbsoluteRadialEncoder {
    
    public:
        AbsoluteRadialEncoder();

        void initialize();
        int read();

    private:
        int pins[8];
};
#endif
