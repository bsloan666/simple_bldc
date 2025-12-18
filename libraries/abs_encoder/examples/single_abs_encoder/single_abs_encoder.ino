
#include <abs_encoder.h> 

/*
  A demonstration of a class that reads a 10 bit analog value from a 
  potentiometer. 
*/


AbsoluteRadialEncoder ar7 = AbsoluteRadialEncoder(A0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  
      Serial.print("[");
      Serial.print(ar7.read());
      Serial.println("]");
      delay(100);

}
