
volatile unsigned int motor_a_pin;
volatile unsigned int motor_b_pin;
volatile unsigned int sensor_pin;

unsigned int interval;

volatile int direction;

int cmd;
unsigned long timeout;
unsigned long start_time;
unsigned long curr_time;
unsigned long prev_time;
double curr_encoder;
double prev_encoder;

unsigned int curr_pot;
unsigned int prev_pot;
volatile int speed;
int _lock;
volatile unsigned int sensor_val;
int servo_index = 0;
int SET_SPEED = 20;     // 20 +-SPEED TIMEOUT
int SET_LOCK  = 30;     // 30
int SET_UNLOCK  = 40;    // 40

void step(){
  sensor_val = digitalRead(sensor_pin);
  if(!(_lock)){
    if(direction > 0){
      if(sensor_val == HIGH){
          analogWrite(motor_b_pin, speed);
          digitalWrite(motor_a_pin, LOW);
      } else {
          digitalWrite(motor_b_pin, LOW);
          analogWrite(motor_a_pin, speed);
      }
    } else {
      if(sensor_val == LOW){
          analogWrite(motor_b_pin, speed);
          digitalWrite(motor_a_pin, LOW);
      } else {
          digitalWrite(motor_b_pin, LOW);
          analogWrite(motor_a_pin, speed);
      }
    }
  }
}
class SimpleBLDCServo {
  public:

    SimpleBLDCServo(unsigned int pin_a, unsigned int pin_b, unsigned int sensor)
    {
      motor_a_pin = pin_a;
      motor_b_pin = pin_b;
      sensor_pin = sensor;
      curr_encoder = 0;
      prev_encoder = 0;
      _lock = 0;
      cmd = 0;
    }
    void unlock(){
      _lock = 0;
       digitalWrite(motor_a_pin, LOW);
       digitalWrite(motor_b_pin, LOW);
       cmd = 0;
    }
    void lock(){
      _lock = 1;
      cmd = 0;
      volatile unsigned int val = digitalRead(sensor_pin);
      if(direction > 0){
        if(val == LOW){
            digitalWrite(motor_b_pin, HIGH);
            digitalWrite(motor_a_pin, LOW);
        } else {
            digitalWrite(motor_b_pin, LOW);
            digitalWrite(motor_a_pin, HIGH);
        }
      } else {
        if(val == HIGH){
            digitalWrite(motor_b_pin, HIGH);
            digitalWrite(motor_a_pin, LOW);
        } else {
            digitalWrite(motor_b_pin, LOW);
            digitalWrite(motor_a_pin, HIGH);
        }
      }
    }

    int is_locked(){
      return _lock;
    }

    void init_pins()
    {
      pinMode(motor_a_pin, OUTPUT);
      pinMode(motor_b_pin, OUTPUT);
      pinMode(sensor_pin, INPUT);
      attachInterrupt(digitalPinToInterrupt(sensor_pin), step, CHANGE);
      digitalWrite(motor_a_pin, LOW);
      digitalWrite(motor_b_pin, LOW);
      direction = 0;
    }

   
    void poke(){
        unlock();
        if(direction > 0){
          analogWrite(motor_b_pin, 255);
          digitalWrite(motor_a_pin, LOW);
        } else { 
          digitalWrite(motor_b_pin, LOW);
          analogWrite(motor_a_pin, 255);
        }
    }

    void dump()
    {
      Serial.print("Timeout:");
      Serial.print(timeout);
      Serial.print(",");
      Serial.print("TimeLapsed:");
      Serial.println(curr_time - start_time);
    }

    void set_speed(){
      _lock = 0;
      curr_time = start_time = millis();
      direction = Serial.parseInt();
      timeout = Serial.parseInt();
      speed = abs(direction);
      poke();
    }

    void set_to_off(){
      digitalWrite(motor_a_pin, LOW);
      digitalWrite(motor_b_pin, LOW);
    }

    void cycle()
    {
      if(timeout){
        if(curr_time >= start_time + timeout){
          lock();
          delay(40);
          unlock();
          delay(40);
          timeout = 0;
          cmd = 0;
          direction = 0;
          speed = 0;
        }     
      }
    }
};

//SimpleBLDCServo servo = SimpleBLDCServo(5, 6, 2);
SimpleBLDCServo servo = SimpleBLDCServo(9, 10, 3);
void setup() {
  // put your setup code here, to run once:

  servo.init_pins();
  Serial.begin(9600);
  prev_time = millis();
  timeout = 0;
  interval = 20;
  delay(1000);
 }

void loop() {
  curr_time = millis();
  if((curr_time - prev_time) >= interval){
    prev_time = curr_time;
    curr_pot = analogRead(A4);

  
    if(Serial.available()){ 
      cmd = Serial.parseInt();
      
      if(cmd  == SET_SPEED){
        servo.set_speed();
      }
      if(cmd == SET_LOCK){         
        servo.lock();

      } else if(cmd == SET_UNLOCK){
        servo.unlock();
      }  
    } 
    Serial.println(curr_pot);    
    // servo.dump();
    //prev_photo = curr_photo;
    servo.cycle();
  }
}
