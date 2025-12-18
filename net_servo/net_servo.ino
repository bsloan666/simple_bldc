#include <single_phase_servo.h>

SinglePhaseServo servo = SinglePhaseServo(9, A0, 2, 5, 6);

void setup() {
  // put your setup code here, to run once:
  servo.initialize();
}

void loop() {
  servo.cycle();
  
  delay(10);
}
