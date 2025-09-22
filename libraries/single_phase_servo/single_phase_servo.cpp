#include "single_phase_servo.h"


volatile int direction;
volatile int sensor_val;
volatile int _lock;
volatile int speed;
volatile HBridge *ghbr;   
volatile OpticalSensor *gose;

void step(){
  sensor_val = gose->read();
  if(!(_lock)){
    if(direction > 0){
      if(sensor_val == HIGH){
          ghbr->set(speed);
      } else {
          ghbr->set(-speed);
      }
    } else {
      if(sensor_val == LOW){
          ghbr->set(speed);
      } else {
          ghbr->set(-speed);
      }
    }
  }
}

SinglePhaseServo::SinglePhaseServo(int base_id_pin, int enc_pin, int sens_pin, int bridge_pin_a, int bridge_pin_b):
    hid(base_id_pin, 4),
    sbs(),
    are(enc_pin),
    ose(sens_pin),
    hbr(bridge_pin_a, bridge_pin_b)
{
    
}

void SinglePhaseServo::initialize(){
    hid.initialize();
    ose.initialize();
    sbs.initialize(hid.address());
    hbr.initialize();
    attachInterrupt(digitalPinToInterrupt(ose.pin), step, CHANGE);
    speed = 0;
    target_pos = 512;
    ghbr = &hbr;
    gose = &ose;
}

void SinglePhaseServo::poke(){
    hbr.set(128);
    hbr.set(-128);
    _lock = 0;
}

void SinglePhaseServo::lock(){
    _lock = 1;
}

void SinglePhaseServo::unlock(){
    _lock = 0;
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
