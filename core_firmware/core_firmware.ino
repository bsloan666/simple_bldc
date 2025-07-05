#include <PID_v1.h>


volatile int reverse = 0;
volatile int aval;
volatile int bval;
volatile int p_aval;
volatile int p_bval;
volatile long req_dir;
volatile unsigned int cmd;

volatile int direction_lookup[16] = {
  0, 1, -1, 2,
  -1, 0, -2, 1,
  1, -2, 0, -1,
  2, -1, 1, 0,
};


//long index = 0;
//long prev_index = 0;

unsigned long curr_time;
unsigned long prev_time = 0;
unsigned int sensor1;
unsigned int sensor2;
long pmeter;
int curr_photo;
int prev_photo;
unsigned long interval = 20;
long direction = -1;

volatile long speed;
int req_speed = 0;

int steps = 36;

int req_pos;
unsigned long timeout;
unsigned long start_time;

int SET_POS = 10;       // 10 POSITION SPEED
int SET_SPEED = 20;     // 20 +-SPEED TIMEOUT
int SET_LOCK  = 30;     // 30
int SET_UNLOCK  = 40;    // 40
int GET_POS = 50;       // 50
int SET_ZERO = 60; 

class SimpleBLDCServo {
  private:
    unsigned int motor_a_pin;
    unsigned int motor_b_pin;
    
    
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
    double target_position;
    int _lock;
    
    SimpleBLDCServo():
      position_pid(&curr_encoder, &output_position_delta, &target_position, 0.75, 0.04, 0.07, DIRECT),
      velocity_pid(&velocity, &output_velocity_delta, &target_velocity, 8.5, 0.0, 0.0, DIRECT)
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
      cmd = 0;
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
    set_zero(){
      target_position = curr_encoder + 316;
      Serial.println("SETTING_ZERO");
      req_dir = 1;
      if(curr_photo > 940){
        req_dir = -1;
      }
      speed = 10;
      cmd = 60;
      poke();
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
      if(cmd){
        Serial.print("Command:");
        Serial.print(cmd);
        Serial.print(",");
        if(cmd == SET_POS){
          Serial.print("Encoder:");
          Serial.print(curr_encoder);
          Serial.print(",");
          Serial.print("Target:");
          Serial.print(target_position);
          Serial.print(",");
          Serial.print("Delta:");
          Serial.println(output_position_delta);
        } else if(cmd == SET_SPEED) {
          
          Serial.print("Timeout:");
          Serial.print(timeout);
          Serial.print(",");
          Serial.print("TimeLapsed:");
          Serial.print(curr_time - start_time);
          Serial.print(",");
          Serial.print("Velocity:");
          Serial.println(req_dir);
        } else if(cmd == SET_ZERO) {
          
          Serial.print("Encoder:");
          Serial.print(curr_encoder);
          Serial.print(",");
          Serial.print("Lock:");
          Serial.print(_lock);
          Serial.print(",");
          Serial.print("Photo:");
          Serial.print(curr_photo);
          Serial.print(",");
          Serial.print("Velocity:");
          Serial.println(req_dir);
        }
      }
    }

    void set_speed(long req_speed, unsigned long _timeout){
      req_dir = req_speed;
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
      if(cmd == SET_POS){
        speed = output_position_magnitude;
      }
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
  attachInterrupt(digitalPinToInterrupt(2), sensor_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), sensor_b, CHANGE);
  Serial.begin(9600);
  prev_time = millis();
  servo.init_pids(interval);
  timeout = 0;
  aval = 0;
  bval = 0;
  p_aval = 0;
  p_bval = 0;
  delay(1000);
  prev_photo = analogRead(A0);
  servo.set_zero();
}

void step(int direction){
  if(direction > 0){
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
void sensor_b(){
  p_aval = aval;
  aval = digitalRead(2);
  p_bval = bval;
  bval = digitalRead(3);
  servo.curr_encoder += direction_lookup[bval * 8 + aval * 4 + p_bval * 2 + p_aval];
}
void sensor_a(){
  p_aval = aval;
  aval = digitalRead(2);
  p_bval = bval;
  bval = digitalRead(3);
  servo.curr_encoder += direction_lookup[bval * 8 + aval * 4 + p_bval * 2 + p_aval];
  if(!(servo._lock)){
    if(cmd == SET_POS){
      step(int(servo.target_position - servo.curr_encoder));
    } else if (cmd == SET_SPEED || cmd == SET_ZERO){
      step(req_dir);
    }
  }
}

void loop() {
  curr_time = millis();
  if((curr_time - prev_time) >= interval){
    prev_time = curr_time;
    curr_photo = analogRead(A0);
    if(Serial.available()){ 
      cmd = Serial.parseInt();
      if(cmd == SET_POS){
        req_pos = Serial.parseFloat();
        speed = Serial.parseFloat();
        req_dir =  servo.target_position - servo.curr_encoder;
        servo.init_targets(req_pos, speed);
        servo.unlock();
        delay(40);
        servo.poke();
      } else if(cmd == SET_SPEED){
        long req_speed =  Serial.parseInt();
        req_dir = req_speed;
        timeout = Serial.parseInt();
        start_time = curr_time;
        servo.set_speed(req_speed, timeout);
        servo.unlock();
        delay(40);
        servo.poke();
      }
      if(cmd == SET_LOCK){         
        servo.lock();
      } else if(cmd == SET_UNLOCK){
        servo.unlock();
        cmd = 0;
      } else if(cmd == SET_ZERO){
        servo.set_zero();
      } else if(cmd == GET_POS){
        Serial.println(servo.curr_encoder);
        cmd =0;
      } 
    } 
    if(cmd == SET_ZERO){
      servo.dump();
      if(abs(curr_photo - prev_photo) > 60){
        cmd = 0;
        servo.lock();
        delay(500);
        servo.unlock();
        servo.target_position = 0;
        servo.curr_encoder = 0;
        servo.prev_encoder = 0;
        Serial.println("DONE SETTING_ZERO");
      }
    }
    if(servo.is_moving()){
      servo.dump();
      if(cmd == SET_POS){
        if(servo.curr_encoder == servo.target_position){
          servo.lock();
          delay(500);
          servo.unlock();
          cmd = 0;
        }
      }
    } 
    if(timeout){
      if(curr_time >= start_time + timeout){
        servo.lock();
        delay(40);
        servo.unlock();
        delay(40);
        servo.init_targets(servo.curr_encoder, 255);   
        timeout = 0;
        cmd = SET_POS;
        req_dir = 0;
      }     
    }
    prev_photo = curr_photo;
    servo.cycle();
  }
}
