
// a slave device driver for bsloan's servo_bus
// library. (Working, but needs fleshing out)

#include <servo_bus.h>

ServoBusSlave sbs = ServoBusSlave(8);

void setup() {
  sbs.initialize();
}

void loop() {
  // FIXME
  delay(1000);
}


