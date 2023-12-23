#include <Wire.h>
#define MPU_I2C 0b1101000 //104 decimal

struct AxisValues {
  float X;
  float Y;
  float Z;
}; 

//The raw data from the IMU
AxisValues accel_Raw;
AxisValues gyro_Raw;

void setup() {
  Serial.begin(9600);
  Wire.begin(21,22,100000);
  setupMPU();
}

void setupMPU() {
  Wire.beginTransmission(MPU_I2C); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(MPU_I2C); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(MPU_I2C); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  logData();
  delay(100);
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accel_Raw.X = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accel_Raw.Y = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accel_Raw.Z = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyro_Raw.X = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyro_Raw.Y = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyro_Raw.Z = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
}
void logData() {
  Serial.print("Accel X: ");
  Serial.print(accel_Raw.X);
  Serial.print(";Accel Y: ");
  Serial.print(accel_Raw.Y);
  Serial.print(";Accel Z: ");
  Serial.print(accel_Raw.Z);
  Serial.print("----Gyro X: ");
  Serial.print(accel_Raw.X);
  Serial.print(";Gyro Y: ");
  Serial.print(accel_Raw.Y);
  Serial.print(";Gyro Z: ");
  Serial.println(accel_Raw.Z);
}
