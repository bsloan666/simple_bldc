#include "hbridge.h"

HBridge::HBridge(unsigned int _pin_a, unsigned int _pin_b){
    pin_a = _pin_a;
    pin_b = _pin_b;
}

void HBridge::initialize(){
    Serial.begin(9600);

    pinMode(pin_a, OUTPUT);
    pinMode(pin_b, OUTPUT);
    digitalWrite(pin_a, LOW);
    digitalWrite(pin_b, LOW);

    Serial.print("HBridge running on digital pins ");
    Serial.print(pin_a);
    Serial.print(" and ");
    Serial.println(pin_b);

    delay(100);
    Serial.end();
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
