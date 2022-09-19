#include <Wire.h>
#include <math.h>

#define MPU_I2C 0b1101000 //104 decimal

#define ACCEL_SENSITIVITY_CONVERION 16384
#define GYRO_SENSITIVITY_CONVERION 131

const float X_ACCEL_OFFSET = -1153.23;
const float Y_ACCEL_OFFSET = -409.57;
const float Z_ACCEL_OFFSET = -632.77;
const float X_GYRO_OFFSET = 824.44;
const float Y_GYRO_OFFSET = -17.71;
const float Z_GYRO_OFFSET = 24.51;



//const float X_ACCEL_OFFSET = 604.16;
//const float Y_ACCEL_OFFSET = 19.75;
//const float Z_ACCEL_OFFSET = -210.92;
//const float X_GYRO_OFFSET = -929.20;
//const float Y_GYRO_OFFSET = -264.49;
//const float Z_GYRO_OFFSET = -236.93;

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
  Wire.begin();
  setupMPU();
}

void setupMPU() {
  //Power Management register
  Wire.beginTransmission(MPU_I2C);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  //Gyroscope configuration
  Wire.beginTransmission(MPU_I2C);
  Wire.write(0x1B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  //Accel configuration
  Wire.beginTransmission(MPU_I2C);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();
}

void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  applyOffsets();
  convertFromRawData();
  calculateRollPitchAccel();
  calculateComplFilter();
  logData();
}

void recordAccelRegisters() {
  //0x3B -> 0x40
  Wire.beginTransmission(MPU_I2C);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_I2C, 6);
  while(Wire.available() < 6);
  accel_Raw.X = Wire.read() << 8 | Wire.read();
  accel_Raw.Y = Wire.read() << 8 | Wire.read();
  accel_Raw.Z = Wire.read() << 8 | Wire.read();
}

void recordGyroRegisters() {
  //0x43 -> 0x48
  Wire.beginTransmission(MPU_I2C);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU_I2C, 6);
  while(Wire.available() < 6);
  gyro_Raw.X = Wire.read() << 8 | Wire.read();
  gyro_Raw.Y = Wire.read() << 8 | Wire.read();
  gyro_Raw.Z = Wire.read() << 8 | Wire.read();
}

void applyOffsets() {
  accel_Raw.X -= X_ACCEL_OFFSET;
  accel_Raw.Y -= Y_ACCEL_OFFSET;
  accel_Raw.Z -= Z_ACCEL_OFFSET;
  gyro_Raw.X -= X_GYRO_OFFSET;
  gyro_Raw.Y -= Y_GYRO_OFFSET;
  gyro_Raw.Z -= Z_GYRO_OFFSET;
}

void convertFromRawData() {
  accel.X = accel_Raw.X / ACCEL_SENSITIVITY_CONVERION;
  accel.Y = accel_Raw.Y / ACCEL_SENSITIVITY_CONVERION;
  accel.Z = accel_Raw.Z / ACCEL_SENSITIVITY_CONVERION;

  gyro.X = gyro_Raw.X / GYRO_SENSITIVITY_CONVERION;
  gyro.Y = gyro_Raw.Y / GYRO_SENSITIVITY_CONVERION;
  gyro.Z = gyro_Raw.Z / GYRO_SENSITIVITY_CONVERION;
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
  Serial.print(",");
  Serial.print(gyro_roll_angle);
  Serial.print(",");
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
  Serial.print(accel_Raw.X);
  Serial.print(";Accel Y: ");
  Serial.print(accel_Raw.Y);
  Serial.print(";Accel Z: ");
  Serial.print(accel_Raw.Z); 
  Serial.print("-------Gyro X: ");
  Serial.print(gyro_Raw.X);
  Serial.print(";Gyro Y: ");
  Serial.print(gyro_Raw.Y);
  Serial.print(";Gyro Z: ");
  Serial.println(gyro_Raw.Z); 
}
