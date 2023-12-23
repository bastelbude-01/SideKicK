
#define xAxis 34
#define yAxis 35

#define swBtn 33
#define redBtn 32

int xVal = 0;
int yVal = 0;

void setup() {
  Serial.begin(9600);

  pinMode(swBtn, INPUT_PULLDOWN);
  pinMode(redBtn, INPUT_PULLDOWN);

}

void loop() {

  //int xVal = analogRead(xAxis);
  //int yVal = analogRead(yAxis);

  int swbVal = digitalRead(swBtn);
  int rdbVal = digitalRead(redBtn);

  Serial.printf("Joystick Testwerte :  RedB: %d ; SW_Btn: %d ; Y: %d; X: %d \n" , rdbVal, swbVal/*, yVal,xVal*/);

  delay(100);

}
