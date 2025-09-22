#ifndef __SINGLE_PHASE_SERVO_H__
#define __SINGLE_PHASE_SERVO_H__

#include <hbridge.h>
#include <hardware_id.h>
#include <abs_encoder.h>
#include <servo_bus.h>
#include <Arduino.h>

class SinglePhaseServo {
    
    public:
        SinglePhaseServo(int base_id_pin, int enc_pin, int sens_pin, int bridge_pin_a, int bridge_pin_b);
        void initialize();
        void poke();
        void lock();
        void unlock();
        void set_target(int position);
        int get_position();
        void cycle();

    private:
        AbsoluteRadialEncoder are;
        OpticalSensor ose;
        HBridge hbr;
        ServoBusSlave sbs;
        HardareID hid;
        int target_pos;
};
#endif
