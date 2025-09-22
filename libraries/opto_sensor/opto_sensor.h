#ifndef __OPTO_SENSOR_H__
#define __OPTO_SENSOR_H__

#include <Arduino.h>

class OpticalSensor {
    
    public:
        OpticalSensor(int digital_pin);
        void initialize();
        int read();

    private:
        int pin;
        unsigned int value;
};
#endif
