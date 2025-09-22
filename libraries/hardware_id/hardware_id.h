#ifndef __HARDWARE_ID_H__
#define __HARDWARE_ID_H__

#include <Arduino.h> 

class HardwareID {
    public:
        HardwareID(int base_pin, int n_pins);

        initialize();
        int address();

    private:
        int my_address;
        int address_pins[4];
        int num_pins;

};
#endif
