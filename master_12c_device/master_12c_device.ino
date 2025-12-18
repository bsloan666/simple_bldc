// Example of using bsloan's primitive servo_bus
// library for daisy-chaining arduino servo controllers 

#include <servo_bus.h>

ServoBusMaster sbm = ServoBusMaster();

void setup() {
  sbm.initialize();
  Serial.begin(9600);  // start serial for output
}


int voosh = 1; 

void loop() {

  int value = sbm.request(8);
  Serial.print("received ");
  Serial.println(value);

  sbm.send(8, 12, voosh);
  Serial.print("sent ");
  Serial.print(12);
  Serial.print(" ");
  Serial.println(voosh);
  
  delay(1000);
  voosh++;
}
