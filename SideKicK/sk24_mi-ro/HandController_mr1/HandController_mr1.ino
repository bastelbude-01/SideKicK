#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float32.h>


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>


rcl_publisher_t axes_x_pub;
std_msgs__msg__Int8 x_msg;

rcl_publisher_t axes_y_pub;
std_msgs__msg__Int8 y_msg;

rcl_publisher_t btn_pub;
std_msgs__msg__Int8 btn_msg;

rcl_publisher_t r_btn_pub;
std_msgs__msg__Int8 r_btn_msg;

rcl_publisher_t roll_pub;
std_msgs__msg__Float32 roll_msg;

rcl_publisher_t pitch_pub;
std_msgs__msg__Float32 pitch_msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

Adafruit_MPU6050 mpu;
const int MPU_ADDR = 0x68;

#define X_PIN 36
#define Y_PIN 39
#define SW_BTN 33
#define RED_BTN 32

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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

int8_t get_x_value();
int8_t get_y_value();
int8_t get_btn_value();
int8_t get_r_btn_value();
float get_angle_x();
float get_angle_y();
float get_angle_z();
float get_gyro_x();
float get_gyro_y();
float get_gyro_z();
int map_to_value(int x, int y);

int valX = 0;
int valY = 0;
int valSW = 0;
int valBTN = 0;

int stickX = 0;
int stickY = 0;

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

float temp_ = 0.0;


void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

    int8_t x = get_x_value();
    x_msg.data = x;
    RCSOFTCHECK(rcl_publish(&axes_x_pub, &x_msg, NULL));

    int8_t y = get_y_value();
    y_msg.data = y;
    RCSOFTCHECK(rcl_publish(&axes_y_pub, &y_msg, NULL));

    int8_t btn = get_btn_value();
    btn_msg.data = btn;
    RCSOFTCHECK(rcl_publish(&btn_pub, &btn_msg, NULL));

    int8_t r_btn = get_r_btn_value();
    r_btn_msg.data = r_btn;
    RCSOFTCHECK(rcl_publish(&r_btn_pub, &r_btn_msg, NULL));

    float roll_ = get_roll_angle();
    roll_msg.data = roll_;
    RCSOFTCHECK(rcl_publish(&roll_pub, &roll_msg, NULL));

    float pitch_ = get_pitch_angle();
    pitch_msg.data = pitch_;
    RCSOFTCHECK(rcl_publish(&pitch_pub, &pitch_msg, NULL));


  }
}

long map (long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// Joystick Werte
int8_t get_x_value() {
  valX = analogRead(X_PIN);       // Joystick Achse X auslesen
  stickX = map(valX, 0, 4095, 0, 110); // Umrechnung von ADC Value zu min/max
  return  stickX;
}

int8_t get_y_value() {
  valY = analogRead(Y_PIN);       // Joystick Achse Y auslesen
  stickY = map(valY, 0, 4095, 0, 110); // Umrechnung von ADC Value zu min/max
  return  stickY;
}

int8_t get_btn_value() {
  valSW = digitalRead(SW_BTN);    // Joystick Button  auslesen
  return valSW;
}
int8_t get_r_btn_value() {
  valBTN = digitalRead(RED_BTN);  // Red      Button  auslesen
  return valBTN;
}
// IMU Werte
float get_angle_x() {
  return accel.X;
}
float get_angle_y() {
  return accel.Y;
}
float get_angle_z() {
  return accel.Z;
}

float get_gyro_x() {
  return gyro.X;
}
float get_gyro_y() {
  return gyro.Y;
}
float get_gyro_z() {
  return gyro.Z;
}
float get_roll_angle() {
  return roll_angle;
}

float get_pitch_angle() {
  return pitch_angle;
}

float get_temperatur(){
  return temp_;  
}

void setupMPU() {
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Wire.begin(21, 22, 100000);
}

void setup() {
  // set_microros_transports();
  set_microros_wifi_transports("FRITZ!Box 7490", "54908635459129454475", "192.168.178.86", 8888);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(SW_BTN, INPUT_PULLUP);
  pinMode(RED_BTN, INPUT_PULLDOWN);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "hand_controller", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
            &axes_x_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
            "axes_x_publisher"));
  RCCHECK(rclc_publisher_init_default(
            &axes_y_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
            "axes_y_publisher"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
            &btn_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
            "btn_publisher"));
  RCCHECK(rclc_publisher_init_default(
            &r_btn_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
            "r_btn_publisher"));

  RCCHECK(rclc_publisher_init_default(
            &pitch_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "pitch_publisher"));

  RCCHECK(rclc_publisher_init_default(
            &roll_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "roll_publisher"));


  // create timer,
  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));



  Serial.begin(115200);
  while (!Serial)
    delay(10);

  if (!mpu.begin()) {
    // Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  setupMPU();
  delay(100);
}



void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  applyOffsets();
  calculateRollPitchAccel();
  calculateComplFilter();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}


void recordAccelRegisters() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel.X = a.acceleration.x / 10; //Store  accelX
  accel.Y = a.acceleration.y / 10; //Store  accelY
  accel.Z = a.acceleration.z; //Store  accelZ
}

void recordGyroRegisters() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gyro.X = g.gyro.x * 10;
  gyro.Y = g.gyro.y * 10;
  gyro.Z = g.gyro.z * 10;
}

void recTemp() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  temp_ = temp.temperature;
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
  double dt = (currentTime - lastMeasurementTime) / 1000.0;
  lastMeasurementTime = currentTime;
  gyro_roll_angle += gyro.X * dt;
  gyro_pitch_angle += gyro.Y * dt;

  roll_angle = COPLEMENTARY_FILTER_WEIGHT * (roll_angle + gyro.X * dt) + (1 - COPLEMENTARY_FILTER_WEIGHT) * accel_roll_angle;
  pitch_angle = COPLEMENTARY_FILTER_WEIGHT * (pitch_angle + gyro.Y * dt) + (1 - COPLEMENTARY_FILTER_WEIGHT) * accel_pitch_angle;
}
