#include <ezButton.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define X_PIN 36
#define Y_PIN 39
#define SW_PIN 33
#define BTN_PIN 32

ezButton swBtn(SW_PIN);
ezButton redBtn(BTN_PIN);

int valX = 0;
int valY = 0;
int valSW = 0;
int valBTN = 0;


void setup() {
  
  redBtn.setDebounceTime(50);
  swBtn.setDebounceTime(50);
  
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
  redBtn.loop();
  swBtn.loop();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  int valX = analogRead(X_PIN); // Joystick achse X auslesen
  int valY = analogRead(Y_PIN);
  int valSW = swBtn.getStateRaw();
  int valBTN = redBtn.getStateRaw();

  Serial.print("RedButton State: ");
  Serial.print(valBTN);
  Serial.print("; SwButton State: ");
  Serial.println(valSW);

  Serial.print("X Achse: ");
  Serial.print(valX);
  Serial.print(" ;Y Achse: ");
  Serial.println(valY);

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
