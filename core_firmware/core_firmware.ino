#include <PID_v1.h>


volatile int reverse = 0;
volatile int aval;
volatile int bval;
volatile int direction_lookup[4] = {
  -1, 1, 1, -1,
};


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
volatile long speed;
int req_speed = 0;

int steps = 36;

int req_pos;
unsigned long timeout;
unsigned long start_time;

int SET_POS = 10;
int SET_SPEED = 20;
int SET_LOCK  = 30;
int SET_STOP  = 40;


class SimpleBLDCServo {
  private:
    unsigned int motor_a_pin;
    unsigned int motor_b_pin;
    int _lock;
    double target_position;
    double velocity;
    double target_velocity;
    double  output_position_magnitude;  
    double output_velocity_delta;
    bool encoder_changed; 
    PID position_pid;
    PID velocity_pid;
    int direction;


  public:
    double curr_encoder;
    double prev_encoder;
    double output_position_delta;
    
    SimpleBLDCServo():
      position_pid(&curr_encoder, &output_position_delta, &target_position, 0.85, 0.04, 0.07, DIRECT),
      velocity_pid(&velocity, &output_velocity_delta, &target_velocity, 10.5, 0.0, 0.0, DIRECT)
    {
      curr_encoder = 0;
      prev_encoder = 0;
      _lock = 0;
    }
    void unlock(){
      _lock = 0;
       digitalWrite(5, LOW);
       digitalWrite(6, LOW);
    }
    void lock(){
      _lock = 1;
       digitalWrite(5, LOW);
       digitalWrite(6, HIGH);
    }

    int is_locked(){
      return _lock;
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

      pinMode(motor_a_pin, OUTPUT);
      pinMode(motor_b_pin, OUTPUT);
      digitalWrite(motor_a_pin, LOW);
      digitalWrite(motor_b_pin, LOW);
      direction = 0;
    }

    void init_targets(double init_targ_pos, double init_targ_vel)
    {
       target_position = init_targ_pos;
       target_velocity = init_targ_vel;
    }
    void poke(){
        unlock();
        if(target_position > curr_encoder){
          analogWrite(6, 255);
          digitalWrite(5, LOW);
        } else { 
          digitalWrite(6, LOW);
          analogWrite(5, 255);
        }
    }

    void dump()
    {
      Serial.print("Encoder:");
      Serial.print(curr_encoder);
      Serial.print(",");
      Serial.print("Target:");
      Serial.print(target_position);
      Serial.print(",");
      Serial.print("Timeout:");
      Serial.print(timeout);
      Serial.print(",");
      Serial.print("Delta:");
      Serial.println(output_position_delta);
    }

    void set_speed(long req_speed, unsigned long _timeout){
      output_position_delta = 1;
      if(req_speed < 0){
        output_position_delta = -1;
      }
      speed = abs(req_speed);
    }

    void set_to_off(){
      digitalWrite(motor_a_pin, LOW);
      digitalWrite(motor_b_pin, LOW);
    }

    void cycle()
    {
      encoder_changed = false;
      velocity = int(abs(curr_encoder - prev_encoder));
 
      position_pid.Compute();
      velocity_pid.Compute();
      output_position_magnitude = abs(output_position_delta);
      output_position_magnitude = constrain(output_position_magnitude, 0, abs(output_velocity_delta));
      speed = output_position_magnitude;
     
      if(velocity){
        encoder_changed = true;
      }
      prev_encoder = curr_encoder;
      
    }
};

volatile SimpleBLDCServo servo;


void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), set_polarity, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(3), get_sensor, CHANGE);
  Serial.begin(9600);
  prev_time = millis();
  servo.init_pids(interval);
  timeout = 0;
  // servo.init_targets(0.0, 64);
}
//void get_sensor(){
// bval = digitalRead(3);
//}
void set_polarity(){
  if(servo.is_locked()){
    return;
  }
  aval = digitalRead(2);
  bval = digitalRead(3);
  int dir = direction_lookup[aval * 2 + bval];
  servo.curr_encoder += dir;
  reverse = servo.output_position_delta < 0;
  
  if(reverse){
    if(aval == HIGH){
        analogWrite(6, speed);
        digitalWrite(5, LOW);
    } else {
        digitalWrite(6, LOW);
        analogWrite(5, speed);
    }
  } else {
    if(aval == LOW){
        analogWrite(6, speed);
        digitalWrite(5, LOW);
    } else {
        digitalWrite(6, LOW);
        analogWrite(5, speed);
    }
  }
}

void loop() {
  curr_time = millis();
  if((curr_time - prev_time) >= interval){
    prev_time = curr_time;
    if(Serial.available()){ 
      int cmd = Serial.parseInt();
      if(cmd == SET_POS){
        req_pos = Serial.parseFloat();
        speed = Serial.parseFloat();
        servo.init_targets(req_pos, speed);           
        servo.poke();
      }
      if(cmd == SET_SPEED){
        long req_speed =  Serial.parseInt();
        timeout = Serial.parseInt();
        start_time = curr_time;
        servo.set_speed(req_speed, timeout);
        servo.poke();
      }
      if(cmd == SET_LOCK){         
        servo.lock();
      }
    }
    if(timeout){
      if(curr_time >= start_time + timeout){
        servo.lock();
        delay(40);
        servo.unlock();
        servo.init_targets(servo.curr_encoder, 255);   
        timeout = 0;
      }
      servo.dump();
    } else {
      servo.cycle();
    } 
  }
}
