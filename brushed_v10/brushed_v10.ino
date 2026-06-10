#include <Wire.h>
#include <AS5600.h>
#include <Arduino.h>

int direction;
int curr_pos;
int prev_pos;
int target_pos = 2048;
long prev_time;

AS5600 encoder;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  if (!encoder.begin()) {
    Serial.println("AS5600 not detected. Check wiring!");
    while (1);
  }
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  delay(2000);
}

void motor_move(int dir, float vel){
  int tmp = abs(dir)/11;
  int power = constrain(pow(tmp, 1.8), 0, 128);

  if(dir > 0){
    digitalWrite(5, LOW);
    analogWrite(6, power);
  } else {
    analogWrite(5, power);
    digitalWrite(6, LOW);
  }
}


void loop() {
  curr_pos = encoder.readAngle();
  long curr_time = millis();

  float velocity = (curr_pos - prev_pos) * 32/(curr_time - prev_time);
  // float velocity = (curr_pos - prev_pos)* 255;
  // put your main code here, to run repeatedly:

  if(Serial.available()){ 
      char cmd = Serial.read(); // Read the first character (e.g., 'L')
      int val = Serial.parseInt(); // Read the following integer (e.g., 90)
      if(cmd  == 'T'){
          target_pos = val;
      } 
    } 
  direction = target_pos - curr_pos;

  motor_move(direction, velocity);
  
  if(velocity > 0.01){
    Serial.print(curr_pos);
    Serial.print(", ");
    Serial.print(direction);
    Serial.print(", ");
    Serial.println(velocity);
  }
  prev_pos = curr_pos;
  prev_time = curr_time;
}
