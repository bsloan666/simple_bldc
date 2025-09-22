
// a slave device driver for bsloan's servo_bus
// library. (Working, but needs fleshing out)

#include <servo_bus.h>

ServoBusSlave sbs = ServoBusSlave();

void setup() {
  sbs.initialize(8);
}

void loop() {
  // FIXME
  delay(1000);
}


