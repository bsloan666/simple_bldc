#include <servo_bus.h>
#include <abs_encoder.h> 

ServoBusMaster sbm = ServoBusMaster();
AbsoluteRadialEncoder are = AbsoluteRadialEncoder(A0);
int curr_pot;

void setup() {
  // put your setup code here, to run once:
  sbm.initialize();
  Serial.begin(9600);  // start serial for outpu
}

void loop() {
  curr_pot = are.read();

  int value = sbm.request(8);
  Serial.print("SLAVE VALUE: ");
  Serial.println(value);
  
  Serial.print("MASTER VALUE: ");
  Serial.println(curr_pot);

  delay(1000);
}
