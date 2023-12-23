/**
 * Hardware Setup:
 * Arduino Pin 2 is connected to the positive side of an LED. -> To turn on the LED when the button is pressed.
 * The button state is constantly zero due to a pulldown resistor and is connectod to +5V over a button (obviously). -> Pin 3
 * Pin 2 of the Arduino = PD2 of Atmega
 * Pin 3 of the Arduino = PD3 of Atmega
 * 
 * Register D, Port 2 and 3 -> DDRD setzen im Setup
 */




void setup() {
  Serial.begin(9600);
  DDRD |= (1 << DDD2);    //Equals   DDRD = B00000100; -> Set PD3 as Input and PD2 as Output and PD4 as Output  
}

void loop() {
  
  bool button_State = (PIND & (1 << PIND3)) >> PIND3;  
  if (button_State == 1) {
    moveMotor();
  }
}

void moveMotor() {
  for(int i = 0; i < 200; i++) {
    PORTD |= (1 << DDD2);
    delayMicroseconds(800);
    PORTD &= ~(1 << DDD2);
    delayMicroseconds(800);
  }  
}
