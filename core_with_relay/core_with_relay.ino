#include <PID_v1.h>



long index = 0;
long prev_index = 0;

unsigned long curr_time;
unsigned long prev_time = 0;
unsigned int sensor1;
unsigned int sensor2;
long pmeter;
unsigned long interval = 20;
long direction = -1;
long prev_dir;
long speed = 255;
int power;
int prev_phase;
int reverse = 0;
int steps = 36;
int aval;
int bval;
int req_pos;

int SET = 10;
int direction_lookup[4] = {
  -1, 1, 1, -1,
};
class SimpleBLDCServo {
  private:
  
    double target_position;
    double velocity;
    double target_velocity;
    double output_position_delta, output_position_magnitude;  
    double output_velocity_delta;
    bool encoder_changed; 
    PID position_pid;
    PID velocity_pid;
    int direction;

  public:
    double curr_encoder;
    double previous_encoder_value;
    SimpleBLDCServo():
      position_pid(&curr_encoder, &output_position_delta, &target_position, 7.0, 0.1, 0.03, DIRECT),
      velocity_pid(&velocity, &output_velocity_delta, &target_velocity, 2.5, 0.0, 0.0, DIRECT)
    {
        previous_encoder_value=0;  
        curr_encoder = 0;  
    }
    int get_pos(){
      return curr_encoder;
    }
    bool is_moving(){
      return encoder_changed;
    } 

    void init_pids(int _interval)
    {
      position_pid.SetMode(AUTOMATIC);            
      position_pid.SetOutputLimits(-255, 255);
      position_pid.SetSampleTime(_interval);

      velocity_pid.SetMode(AUTOMATIC);              
      velocity_pid.SetOutputLimits(-255, 255);
      velocity_pid.SetSampleTime(_interval);

     
      direction = 0;
    }

    void init_targets(double init_targ_pos, double init_targ_vel)
    {
       target_position = init_targ_pos;
       target_velocity = init_targ_vel;
    }
    void poke(){
        digitalWrite(5, LOW);
        delay(40); 
        digitalWrite(5, HIGH);
        delay(40);
        /*
        digitalWrite(5, LOW);
        delay(180); 
        digitalWrite(5, HIGH);
        delay(180);
        */
    }

    void dump()
    {
      Serial.print("Encoder:");
      Serial.print(curr_encoder);
      Serial.print(",");
      Serial.print("Target:");
      Serial.print(target_position);
      Serial.print(",");
      Serial.print("Delta:");
      Serial.println(output_position_delta);
    }
    void set_to_off(){
      speed = 0;
    }
    int has_arrived(){
      return curr_encoder == target_position;
    }

    void cycle()
    {
      encoder_changed = false;
      velocity = abs(curr_encoder - previous_encoder_value);
 
      position_pid.Compute();
      velocity_pid.Compute();
      output_position_magnitude = abs(output_position_delta);
      output_position_magnitude = constrain(output_position_magnitude, 0, abs(output_velocity_delta));
      speed = output_position_magnitude;
     
      reverse = output_position_delta < 0;

      if(previous_encoder_value != curr_encoder){
        encoder_changed = true;
      }
    }  
};

SimpleBLDCServo servo;


void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), set_polarity, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(3), set_polarity, CHANGE);
  Serial.begin(9600);
  prev_time = millis();
  power = 0;
  servo.init_pids(interval);
  // servo.init_targets(0.0, 64);
  delay(500);
  Serial.println("STARTUP!");
}

void set_polarity(){
  aval = digitalRead(2);
  bval = digitalRead(3);
  
  int dir = direction_lookup[aval * 2 + bval];
  servo.previous_encoder_value = servo.curr_encoder;
  servo.curr_encoder += dir;
  speed = 255;
  if(reverse){
    if(aval == HIGH){
        digitalWrite(5, LOW);
    } else {
        analogWrite(5, speed);
    }
    index--;
  } else {
    if(aval == LOW){
        digitalWrite(5, LOW);
    } else {
        analogWrite(5, speed);
    }
    index++;
  }
}

void loop() {
  curr_time = millis();
  if((curr_time - prev_time) >= interval){
    prev_time = curr_time;
    if(Serial.available()){ 
        int cmd = Serial.parseInt();
        if(cmd == SET){
          req_pos = Serial.parseFloat();
          speed = Serial.parseFloat();
          servo.init_targets(req_pos, speed); 
          
          digitalWrite(6, HIGH);
          power = 1;
          Serial.println("POWER ON");
          servo.poke();
          
        }
    }  
    
    if( servo.curr_encoder != servo.previous_encoder_value){
      servo.dump();
    }
    
    prev_index = index;
    if( servo.has_arrived()  && power){
      digitalWrite(6, LOW);
      power = 0;
      Serial.println("POWER OFF");
    }
    servo.cycle(); 
    
  }
}
