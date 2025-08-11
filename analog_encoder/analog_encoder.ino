
int curr_pot;
int prev_pot;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  curr_pot = analogRead(A4);

  if(curr_pot != prev_pot){
    Serial.print("VAL: ");
    Serial.println(curr_pot);
  }
  // put your main code here, to run repeatedly:
  prev_pot = curr_pot;
}
