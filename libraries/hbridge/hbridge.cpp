#include "hbridge.h"

HBridge::HBridge(unsigned int _pin_a, unsigned int _pin_b){
    pin_a = _pin_a;
    pin_b = pin_b;
}

void HBridge::initialize(){
    pinMode(pin_a, OUTPUT);
    pinMode(pin_b, OUTPUT);
    digitalWrite(pin_a, LOW);
    digitalWrite(pin_b, LOW);
}  

void HBridge::set(int velocity) {
    if(velocity > 0){
        analogWrite(pin_a, constrain(abs(velocity), 0, 255));
        digitalWrite(pin_b, LOW);
    } else if(velocity < 0){
        digitalWrite(pin_a, LOW);
        analogWrite(pin_b, constrain(abs(velocity), 0, 255));
    } else{
        digitalWrite(pin_a, LOW);
        digitalWrite(pin_b, LOW);
    }
}
