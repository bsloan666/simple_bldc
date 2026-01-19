#ifndef __HARDWARE_ID_H__
#define __HARDWARE_ID_H__

#include <Arduino.h> 

class HardwareID {
    /*
    Use pins 7,8,9,10
    */
    public:
        HardwareID();

        void initialize();
        int address();

    private:
        int my_address;
        int address_pins[4];
        int num_pins;

};
#endif
