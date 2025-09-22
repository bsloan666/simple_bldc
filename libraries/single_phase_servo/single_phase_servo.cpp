#include "single_phase_servo.h"


volatile int direction;
volatile int sensor_val;
volatile int lock;
volatile int speed;
   

SinglePhaseServo(int base_id_pin, int enc_pin, int sens_pin, int bridge_pin_a, int bridge_pin_b):
    hid(base_pin_id, 4),
    sbs(),
    are(enc_pin),
    ose(sens_pin),
    hbr(bridge_pin_a, bridge_pin_b)
{
    
}

int SinglePhaseServo::initialize(){
    hid.initialize();
    ose.initialize();
    sbs.initialize(hid.get_address())
    hbr.initialize();
    attachInterrupt(digitalPinToInterrupt(sensor_pin), step, CHANGE);
    speed = 0;
    target_pos = 512;
}

void SinglePhaseServo::poke(){
    hbr.set(128);
    hbr.set(-128);
    lock = 0;
}

void SinglePhaseServo::lock(){
    lock = 1;
}

void SinglePhaseServo::unlock(){
    lock = 0;
}

void SinglePhaseServo::set_target(int position){
    target_pos = position;
}

int SinglePhaseServo::get_position(){
    return are.read();
}

void SinglePhaseServo::cycle(){
    direction = are.read() - target_pos;
    speed = abs(direction); 
}

void step(){
  sensor_val = ose.read();
  if(!(lock)){
    if(direction > 0){
      if(sensor_val == HIGH){
          hbr.set(speed)
      } else {
          hbr.set(-speed)
      }
    } else {
      if(sensor_val == LOW){
          hbr.set(speed)
      } else {
          hbr.set(-speed)
      }
    }
  }
}
