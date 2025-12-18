
#include <abs_encoder.h> 

/*
  A demonstration of a class that reads a 10 bit analog value from a 
  potentiometer. 
*/

AbsoluteRadialEncoder ar1 = AbsoluteRadialEncoder(A1);
AbsoluteRadialEncoder ar2 = AbsoluteRadialEncoder(A2);
AbsoluteRadialEncoder ar3 = AbsoluteRadialEncoder(A3);
AbsoluteRadialEncoder ar4 = AbsoluteRadialEncoder(A4);
AbsoluteRadialEncoder ar5 = AbsoluteRadialEncoder(A5);
AbsoluteRadialEncoder ar6 = AbsoluteRadialEncoder(A6);
AbsoluteRadialEncoder ar7 = AbsoluteRadialEncoder(A7);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()){ 
    int cmd = Serial.parseInt();
    if(cmd == 10){
      Serial.print("[");
      Serial.print(ar1.read());
      Serial.print(", ");
      Serial.print(ar2.read());
      Serial.print(", ");
      Serial.print(ar3.read());
      Serial.print(", ");
      Serial.print(ar4.read());
      Serial.print(", ");
      Serial.print(ar5.read());
      Serial.print(", ");
      Serial.print(ar6.read());
      Serial.print(", ");
      Serial.print(ar7.read());
      Serial.println("]");
    }
  }
}
