#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define MPU_I2C 0b1101000

Adafruit_MPU6050 mpu;
const int MPU_ADDR = 0x68;

struct sensorVal {
  float X;
  float Y;
  float Z;
}; 

sensorVal gyro;
sensorVal accel;

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
  Serial.println("MPU6050 Found!");
  
  setupMPU();
  delay(20);
  calibrateMPU();
}

void setupMPU() {
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); 
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Wire.begin(21,22,100000);
}

void calibrateMPU() {
  double X_ACCEL_ERROR = 0;
  double Y_ACCEL_ERROR = 0;
  double Z_ACCEL_ERROR = 0;
  double X_GYRO_ERROR = 0;
  double Y_GYRO_ERROR = 0;
  double Z_GYRO_ERROR = 0;

  for(int i = 0; i < 20000; i++) {
    //Read the Acceleration values
    recordAccelRegisters();
    recordGyroRegisters();
    //Calculate the errors each time
    X_ACCEL_ERROR += accel.X;
    Y_ACCEL_ERROR += accel.Y;
    Z_ACCEL_ERROR += accel.Z; // - 16384.0; //Z-Sensor misst ja immer 1 am Anfang
    X_GYRO_ERROR += gyro.X;
    Y_GYRO_ERROR += gyro.Y;
    Z_GYRO_ERROR += gyro.Z;
    if(i % 1000 == 0) {
      Serial.print(". ");
    }
  }
  double X_ACCEL_OFFSET = X_ACCEL_ERROR/20000.0;
  double Y_ACCEL_OFFSET = Y_ACCEL_ERROR/20000.0;
  double Z_ACCEL_OFFSET = Z_ACCEL_ERROR/20000.0;
  double X_GYRO_OFFSET = X_GYRO_ERROR /20000.0;
  double Y_GYRO_OFFSET = Y_GYRO_ERROR /20000.0;
  double Z_GYRO_OFFSET = Z_GYRO_ERROR /20000.0;
  Serial.println("Calibration done:");
  Serial.println("Offsets: ");
  Serial.print("const float X_ACCEL_OFFSET = ");
  Serial.print(X_ACCEL_OFFSET);
  Serial.println(";");
  Serial.print("const float Y_ACCEL_OFFSET = ");
  Serial.print(Y_ACCEL_OFFSET);
  Serial.println(";");
  Serial.print("const float Z_ACCEL_OFFSET = ");
  Serial.print(Z_ACCEL_OFFSET);
  Serial.println(";");
  Serial.print("const float X_GYRO_OFFSET = ");
  Serial.print(X_GYRO_OFFSET);
  Serial.println(";");
  Serial.print("const float Y_GYRO_OFFSET = ");
  Serial.print(Y_GYRO_OFFSET);
  Serial.println(";");
  Serial.print("const float Z_GYRO_OFFSET = ");
  Serial.print(Z_GYRO_OFFSET);
  Serial.println(";");
}

void recordAccelRegisters() {
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel.X = a.acceleration.x; //Store  accelX
  accel.Y = a.acceleration.y; //Store  accelY
  accel.Z = a.acceleration.z; //Store  accelZ
}

void recordGyroRegisters() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  gyro.X = g.gyro.x; 
  gyro.Y = g.gyro.y; 
  gyro.Z = g.gyro.z; 
}

void loop() {

}
