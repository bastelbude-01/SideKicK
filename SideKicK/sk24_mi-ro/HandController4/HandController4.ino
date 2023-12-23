#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define X_PIN 36
#define Y_PIN 39
#define SW_BTN 33
#define RED_BTN 32


int valX = 0;
int valY = 0;
int valSW = 0;
int valBTN = 0;

int stickX = 0;
int stickY = 0;

long map (long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() {

  pinMode(SW_BTN, INPUT_PULLUP);
  pinMode(RED_BTN, INPUT_PULLDOWN);

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}



void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  valX = analogRead(X_PIN);       // Joystick Achse X auslesen
  valY = analogRead(Y_PIN);       // Joystick Achse Y auslesen
  valSW = digitalRead(SW_BTN);    // Joystick Button  auslesen
  valBTN = digitalRead(RED_BTN);  // Red      Button  auslesen

  stickX = map(valX,0, 4095, -90, 90); // Umrechnung von ADC Value zu min/max
  stickY = map(valY,0, 4095, -90, 90);

  /*
    stickX = map(valX,0,4095,-1, 1);    
    stickY = map(valY,0,4095,-1, 1);
  */

  Serial.print("RedButton State: ");
  Serial.print(valBTN);
  Serial.print("; SwButton State: ");
  Serial.println(valSW);

  Serial.print("X Achse: ");
  Serial.print(stickX);
  Serial.print(" ;Y Achse: ");
  Serial.println(stickY);

  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");


  delay(500);
}
