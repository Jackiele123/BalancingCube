#include <Servo.h>
#include <PWM.h>
#include <MPU6050.h>

Servo Motor;

// MPU6050

MPU6050 mpu6050;
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
// Temporary 

int potValue;

// MOTOR VARIABLES

const int motorDirection = 7; // CW/CCW Motor Direction 
const int motorPower = 8; // Motor on/off
const int motorPWM = 9; // Motor PWM control

// ANGLE VARIABLES
float angle0 = -5; //mechanical balance angle (ideally 0 degrees) 
float Gyro_x;
float Gyro_y;
float Gyro_z;  //Angular velocity by gyroscope calculation

// PID PARAMETERS
double kp = 2;
double ki = 0; 
double kd = .25; 

//PID Speed loop variables
double kp_speed =  3; 
double ki_speed = 0.072;
double kd_speed = 0; // NOT USED  
double targetAngle = 90;
int PD_pwm;  //angle output


void setup() {
  
  // PWM.h Setup
  InitTimersSafe();
  // The Nidec 24H677 BLDC Motor requires a PWM frequency of 20KHz to 25KHz
  bool success = SetPinFrequencySafe(9, 20000);

  //Set motor Pins
  pinMode(motorDirection,OUTPUT);
  pinMode(motorPower,OUTPUT);
  pinMode(motorPWM,OUTPUT);

  // Inital output states

  digitalWrite(motorDirection,LOW); // CCW
  digitalWrite(motorPower,HIGH); // Motor ON
  analogWrite(motorPWM,0); //No PWM Signal

  // MPU6050

  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);
  delay(20);

}
//void counterclockwise(PWM){
//  analogWrite(motorPWM,255);    
//  digitalWrite(motorDirection,LOW); // CCW 
//  digitalWrite(motorPower,HIGH); // Motor ON
//  analogWrite(motorPWM,motorSpeed);
//}
double calculateAngle()
{
  const int OFFSET = 8;
   // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
 // calculate_IMU_error();
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX -3.47; // GyroErrorX ~(-0.56)
  GyroY = GyroY +3.03; // GyroErrorY ~(2)
  GyroZ = GyroZ -10.64; // GyroErrorZ ~ (-0.8)


  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  gyroAngleX = 0.96 * gyroAngleX + 0.04 * accAngleX;
  gyroAngleY = 0.96 * gyroAngleY + 0.04 * accAngleY;

  roll = gyroAngleX;
  pitch = gyroAngleY;
  // Print the values on the serial monitor
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);
  return pitch + OFFSET;
}

double calculatePID(double currentAngle)
{
  double error;
  double lastError;
  double cumError, rateError;
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - previousTime)/100;
  error = abs(targetAngle - currentAngle);
  
  cumError += error * elapsedTime;
  
  rateError = ((error - lastError)/elapsedTime);
  
  PD_pwm = kp * error + ki * cumError - kd *rateError;

  constrain(PD_pwm,0,255);

  PD_pwm = abs(PD_pwm - 255);
  
  lastError = error;
  previousTime = currentTime;

//  Serial.println(currentTime);
//  Serial.println(previousTime);
//  Serial.println(elapsedTime);

  return PD_pwm;
}

void loop(){

  double currentAngle = calculateAngle();
  double motorSpeed = calculatePID(currentAngle);
  
  
  potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
  potValue = map(potValue, 0, 1023, 0, 255);   // scale it to use it with the servo library (value between 0 and 180)
  //motorSpeed = map(potValue, 0, 1023, 0, 255);
//  Serial.println(currentAngle);
//  Serial.println(motorSpeed);
  Serial.println("-----------------------------------------");
  delay(400);
  
 if(currentAngle >= 110 || currentAngle <=70)
 {
  if(currentAngle >= 110)
  {
  digitalWrite(motorDirection,HIGH);
  analogWrite(motorPWM,0); 
  }
  else
  {
  digitalWrite(motorDirection,LOW);
  analogWrite(motorPWM,0); 
  }
 }
 else
 {
  if (currentAngle>90)
  {
      analogWrite(motorPWM,255);    
      digitalWrite(motorDirection,LOW); // CCW 
      digitalWrite(motorPower,HIGH); // Motor ON
      analogWrite(motorPWM,motorSpeed);
  }
  else
  {
      digitalWrite(motorDirection,HIGH); // CW
      digitalWrite(motorPower,HIGH); // Motor ON
      analogWrite(motorPWM,motorSpeed); 
  }
 }
}
