#include "hardware_id.h"


HardwareID::HardwareID(int base_pin, int n_pins):
    num_pins = n_pins {
    for (int i = 0; i < num_pins; i++) {
        address_pins[i]  = base_pin + i;
    }
}

void HardwareID::initialize() {
    for (int i = 0; i < num_pins; i++) {
        pinMode(address_pins[i], INPUT_PULLUP);
    }
    delay(100); // Give pins time to settle

    for (int i = 0; i < num_pins; i++) {
        if (digitalRead(address_pins[i]) == LOW) {
            my_address += (1 << i);
        }
    }
}

int HardwareID::address() {
    return my_address;
}
