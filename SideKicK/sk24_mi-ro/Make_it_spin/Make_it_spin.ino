const int mr_step = 2;
const int mr_dir = 4;

void setup() {
  digitalWrite(mr_step,LOW);
  digitalWrite(mr_dir,LOW);
  pinMode(mr_step,OUTPUT);
  pinMode(mr_dir,OUTPUT);
}

void loop() {
  for(int i = 0; i < 200; i++) {
    digitalWrite(mr_step,HIGH);
    delayMicroseconds(800);
    digitalWrite(mr_step,LOW);
    delayMicroseconds(800);
  }
  digitalWrite(mr_dir,HIGH);
  delay(1000);
  for(int i = 0; i < 200; i++) {
    digitalWrite(mr_step,HIGH);
    delayMicroseconds(800);
    digitalWrite(mr_step,LOW);
    delayMicroseconds(800);
  }
  digitalWrite(mr_dir,LOW);
  delay(1000);
}
