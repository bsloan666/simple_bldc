#include "hardware_id.h"


HardwareID::HardwareID():
    num_pins(4) {
    for (int i = 0; i < num_pins; i++) {
        address_pins[i]  = 7 + i;
    }
}

void HardwareID::initialize() {
    Serial.begin(9600);
    for (int i = 0; i < num_pins; i++) {
        pinMode(address_pins[i], INPUT_PULLUP);
    }
    delay(100); // Give pins time to settle

    for (int i = 0; i < num_pins; i++) {
        if (digitalRead(address_pins[i]) == LOW) {
            my_address += (1 << i);
        }
    }

    delay(100);
    int last_pin = address_pins[num_pins - 1]; 

    Serial.flush();
    Serial.print("HardwareID sensed from pins ");
    Serial.print(address_pins[0]);
    Serial.print(" - ");
    Serial.print(last_pin);
    Serial.print(" is ");
    Serial.println(my_address);

    delay(100);
    Serial.end();
}

int HardwareID::address() {
    return my_address;
}
