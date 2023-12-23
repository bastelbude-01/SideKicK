#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>


Adafruit_MPU6050 mpu;
const int MPU_ADDR = 0x68;


const float X_ACCEL_OFFSET = 0.04;
const float Y_ACCEL_OFFSET = -0.01;
const float Z_ACCEL_OFFSET = 8.93;
const float X_GYRO_OFFSET = 0.07;
const float Y_GYRO_OFFSET = 0.02;
const float Z_GYRO_OFFSET = -0.01;


const float COPLEMENTARY_FILTER_WEIGHT = 0.98;

struct AxisValues {
  float X;
  float Y;
  float Z;
};

AxisValues accel_Raw;
AxisValues gyro_Raw;

AxisValues accel;
AxisValues gyro;

float accel_roll_angle = 0.0;
float accel_pitch_angle = 0.0;

float gyro_roll_angle = 0.0;
float gyro_pitch_angle = 0.0;
long lastMeasurementTime = 0;

float roll_angle = 0.0;
float pitch_angle = 0.0;

void setup() {
  Serial.begin(9600);
  while (!Serial)
    delay(10); // will pause until serial console opens

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  setupMPU();
}

void setupMPU() {
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); 
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Wire.begin(21,22,100000);
}

void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  applyOffsets();
  calculateRollPitchAccel();
  calculateComplFilter();
  logRawData();
  // delay(250);
}

void recordAccelRegisters() {
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel.X = a.acceleration.x/10; //Store  accelX
  accel.Y = a.acceleration.y/10; //Store  accelY
  accel.Z = a.acceleration.z; //Store  accelZ
}

void recordGyroRegisters() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gyro.X = g.gyro.x*10; 
  gyro.Y = g.gyro.y*10; 
  gyro.Z = g.gyro.z*10; 
}

void applyOffsets() {
  accel.X -= X_ACCEL_OFFSET ;
  accel.Y -= Y_ACCEL_OFFSET;
  accel.Z -= Z_ACCEL_OFFSET;
  gyro.X -= X_GYRO_OFFSET;
  gyro.Y -= Y_GYRO_OFFSET;
  gyro.Z -= Z_GYRO_OFFSET;
}

void calculateRollPitchAccel() {
  accel_roll_angle = atan2(accel.Y, accel.Z) / (2 * M_PI) * 360;
  accel_pitch_angle = -atan2(accel.X, accel.Z) / (2 * M_PI) * 360;
}

void calculateComplFilter() {
  long currentTime = millis();  //Millisec
  double dt = (currentTime - lastMeasurementTime)/1000.0;
  lastMeasurementTime = currentTime;
  gyro_roll_angle += gyro.X * dt;
  gyro_pitch_angle += gyro.Y * dt;

  roll_angle = COPLEMENTARY_FILTER_WEIGHT * (roll_angle + gyro.X * dt) + (1 - COPLEMENTARY_FILTER_WEIGHT) * accel_roll_angle;
  pitch_angle = COPLEMENTARY_FILTER_WEIGHT * (pitch_angle + gyro.Y * dt) + (1 - COPLEMENTARY_FILTER_WEIGHT) * accel_pitch_angle;
}

void logData() {
  //Serial.print("Roll Accel: ");
  Serial.print(accel_roll_angle); 
  Serial.print(", ");
  Serial.print(gyro_roll_angle);
  Serial.print(", ");
  Serial.println(roll_angle);
  /*Serial.print("--Pitch Accel: ");
  Serial.print(accel_pitch_angle); 
  Serial.print(";Pitch gyro: ");
  Serial.print(gyro_pitch_angle);
  Serial.print(";Pitch Compl: ");
  Serial.println(pitch_angle);*/
}

void logRawData() {
  Serial.print("Accel X: ");
  Serial.print(accel.X);
  Serial.print(";Accel Y: ");
  Serial.print(accel.Y);
  Serial.print(";Accel Z: ");
  Serial.print(accel.Z); 
  Serial.print("-------Gyro X: ");
  Serial.print(gyro.X);
  Serial.print(";Gyro Y: ");
  Serial.print(gyro.Y);
  Serial.print(";Gyro Z: ");
  Serial.println(gyro.Z); 
}
