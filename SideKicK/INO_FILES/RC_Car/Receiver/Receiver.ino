//Include Libraries
#include <SPI.h>
#include <RF24.h>

const int leftMotorForward = 3;
const int leftMotorBackward = 5;
const int rightMotorForward = 6;
const int rightMotorBackward = 10;

const float SPEED_ADAPTION_RATIO_Y = 0.3;
const float SPEED_ADAPTION_RATIO_X = 0.4;
const int MOTOR_DRIVER_HIGHEST_VALUE = 255;
const int MOTOR_DRIVER_LOWEST_VALUE = 0;

const int JOYSTICK_X_OFFSET = 520;
const int JOYSTICK_Y_OFFSET = 506;

int joystickX = 0;
int joystickY = 0;
int joystickSW = 0;
int IMU_Roll = 0;
int IMU_Pitch = 0;

int pwmMotorX = 0;
int pwmMotorY = 0;

int pwmMotorRight = 0;
int pwmMotorLeft = 0;
bool drivesForeward = false;

bool joystickControl = true;
//create an RF24 object
RF24 radio(8, 9);  // CE, CSN

//address through which two modules communicate.
const byte address[6] = "00001";

void setup()
{
  Serial.begin(9600);

  radio.begin();

  //set the address
  radio.openReadingPipe(0, address);

  //Set module as receiver
  radio.startListening();

  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward,OUTPUT);

  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward,OUTPUT);

}

void loop(){
  //Read the data if available in buffer
  if (radio.available()){
    int data[5] = {0};
    radio.read(data, sizeof(data));
    joystickX = (data[0] - JOYSTICK_X_OFFSET);
    //Change the logic of the direction with a -: Upwards -> JoystickY is positive, Downwards -> JoystickY is negative
    joystickY = -(data[1] - JOYSTICK_Y_OFFSET);
    joystickSW = data[2];
    IMU_Roll = data[3];
    IMU_Pitch = -data[4];
    if(joystickSW == 0) {
      //If joystickControl == true -> Afterwards: joystickControl = !joystickControl = !true = false
      joystickControl = !joystickControl;
    }
    if(joystickControl) {
      calculateYPWMJoy();
      calculateXPWMJoy();
    } else {
      calculateYPWMIMU();    
      calculateXPWMIMU();
    }
    calculateMotorSpeeds();
    adaptMotorSpeed();
  }
}

void calculateYPWMJoy() {
  if(joystickY > 10) { //Forward
    float controlRatio = (joystickY - 10) / 550.0;
    pwmMotorY = SPEED_ADAPTION_RATIO_Y * controlRatio * MOTOR_DRIVER_HIGHEST_VALUE + (1 - SPEED_ADAPTION_RATIO_Y) * pwmMotorY;
    drivesForeward = true;
  } else if (joystickY < -10) { //Backward
    float controlRatio = (abs(joystickY) - 10) / 550.0;
    pwmMotorY = SPEED_ADAPTION_RATIO_Y * controlRatio * MOTOR_DRIVER_HIGHEST_VALUE + (1 - SPEED_ADAPTION_RATIO_Y) * pwmMotorY;
    drivesForeward = false;
  } else { //Stop driving
    //Goes to zero
    pwmMotorY = (1 - SPEED_ADAPTION_RATIO_Y) * pwmMotorY;
  }
}

void calculateXPWMJoy() {
  if(joystickX > 10) {  //To the right
    float controlRatio = (joystickX - 10) / 550.0;
    pwmMotorX = SPEED_ADAPTION_RATIO_X * controlRatio * MOTOR_DRIVER_HIGHEST_VALUE + (1 - SPEED_ADAPTION_RATIO_X) * pwmMotorX;
  } else if (joystickX < -10) { //To the left
    float controlRatio = (abs(joystickX) - 10) / 550.0;
    pwmMotorX = - SPEED_ADAPTION_RATIO_X * controlRatio * MOTOR_DRIVER_HIGHEST_VALUE + (1 - SPEED_ADAPTION_RATIO_X) * pwmMotorX;
  } else {  //Stop driving
    //Goes to zero
    pwmMotorX = (1 - SPEED_ADAPTION_RATIO_X) * pwmMotorX;
  }
}

void calculateYPWMIMU() {
  if(IMU_Roll > 5) { //Forward
    float controlRatio = (IMU_Roll - 5) / 45.0;
    pwmMotorY = SPEED_ADAPTION_RATIO_Y * controlRatio * MOTOR_DRIVER_HIGHEST_VALUE + (1 - SPEED_ADAPTION_RATIO_Y) * pwmMotorY;
    drivesForeward = true;
  } else if (IMU_Roll < -5) { //Backward
    float controlRatio = (abs(IMU_Roll) - 5) / 45.0;
    pwmMotorY = SPEED_ADAPTION_RATIO_Y * controlRatio * MOTOR_DRIVER_HIGHEST_VALUE + (1 - SPEED_ADAPTION_RATIO_Y) * pwmMotorY;
    drivesForeward = false;
  } else { //Stop driving
    //Goes to zero
    pwmMotorY = (1 - SPEED_ADAPTION_RATIO_Y) * pwmMotorY;
  }
}

void calculateXPWMIMU() {
  if(IMU_Pitch > 5) {  //To the right
    float controlRatio = (IMU_Pitch - 5) / 45.0;
    pwmMotorX = SPEED_ADAPTION_RATIO_X * controlRatio * MOTOR_DRIVER_HIGHEST_VALUE + (1 - SPEED_ADAPTION_RATIO_X) * pwmMotorX;
  } else if (IMU_Pitch < -5) { //To the left
    float controlRatio = (abs(IMU_Pitch) - 5) / 45.0;
    pwmMotorX = - SPEED_ADAPTION_RATIO_X * controlRatio * MOTOR_DRIVER_HIGHEST_VALUE + (1 - SPEED_ADAPTION_RATIO_X) * pwmMotorX;
  } else {  //Stop driving
    //Goes to zero
    pwmMotorX = (1 - SPEED_ADAPTION_RATIO_X) * pwmMotorX;
  }
}

void calculateMotorSpeeds() {
  pwmMotorRight = abs(pwmMotorY) + pwmMotorX;
  pwmMotorLeft = abs(pwmMotorY) - pwmMotorX;
  if (abs(pwmMotorRight) > MOTOR_DRIVER_HIGHEST_VALUE) {
    pwmMotorRight = MOTOR_DRIVER_HIGHEST_VALUE;
  } else if ( pwmMotorRight < MOTOR_DRIVER_LOWEST_VALUE) {
    pwmMotorRight = MOTOR_DRIVER_LOWEST_VALUE;
  }
  if (abs(pwmMotorLeft) > MOTOR_DRIVER_HIGHEST_VALUE) {
    pwmMotorLeft = MOTOR_DRIVER_HIGHEST_VALUE;
  } else if ( pwmMotorLeft < MOTOR_DRIVER_LOWEST_VALUE) {
    pwmMotorLeft = MOTOR_DRIVER_LOWEST_VALUE;
  }
}

void adaptMotorSpeed() {
  if (drivesForeward) {
    analogWrite(rightMotorForward, pwmMotorRight);
    digitalWrite(rightMotorBackward, LOW);
    analogWrite(leftMotorForward, pwmMotorLeft);
    digitalWrite(leftMotorBackward, LOW);
  } else {
    analogWrite(rightMotorBackward, pwmMotorRight);
    digitalWrite(rightMotorForward, LOW);
    analogWrite(leftMotorBackward, pwmMotorLeft);
    digitalWrite(leftMotorForward, LOW);
  }
}
