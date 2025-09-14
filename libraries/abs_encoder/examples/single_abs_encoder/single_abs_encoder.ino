
#include <abs_encoder.h> 

/*
  A demonstration of a class that reads a 10 bit analog value from a 
  potentiometer. 
*/
int curr_pot;
int prev_pot;

AbsoluteRadialEncoder are = AbsoluteRadialEncoder(A0);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  curr_pot = are.read();

  if(curr_pot != prev_pot){
    Serial.print("ENCODER VALUE: ");
    Serial.println(curr_pot);
  }
  // put your main code here, to run repeatedly:
  prev_pot = curr_pot;
}
