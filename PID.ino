#include <Wire.h>
#include <Servo.h>

Servo right_motor ;
Servo left_motor ;
double throttle = 1500 ;

//For Accelerometer
int16_t Acc_raw_X, Acc_raw_Y, Acc_raw_Z ;
float AccX, AccY, AccZ ;
float Acc_X ;

//For Gyroscope
int16_t Gyro_raw_X, Gyro_raw_Y, Gyro_raw_Z ;
float Gyro_X, Gyro_Y, Gyro_Z ;

float setpoint = 0 ;
float angle_X = 0 ;

unsigned long time_elapsed, time, previous_time ;
float rad_to_deg = 180/3.1415 ;

float PID, error_X, previous_error_X, pwm_right_motor, pwm_left_motor ;
float pid_p=0 ;
float pid_i=0 ;
float pid_d=0 ;

float Kp = 9 ;
float Ki = 9 ;
float Kd = 9 ;


void setup() {
  
  Wire.begin() ;
  Wire.beginTransmission(0x68) ; 
  Wire.write(0x6B) ;
  Wire.write(0) ;
  Wire.endTransmission(true) ;
  Serial.begin(9600) ; //Set Baud rate as 9600

  right_motor.attach(6) ;
  left_motor.attach(9) ;

  //Sending minimum pulse to ESCs to switch them on
  right_motor.writeMicroseconds(1000) ;
  left_motor.writeMicroseconds(1000) ;
  delay(5000) ;

  time = millis() ;

}

void loop() {
  
  //Store Previous time
  previous_time = time ;
  time = millis() ;
  time_elapsed = (time - previous_time) / 1000 ; //Divide by 1000 to get the value in milliseconds

  //Accelerometer
  Wire.beginTransmission(0x68) ;
  Wire.write(0x3B) ; //3B is the hexadecimal address for the Acc_X_H register
  Wire.endTransmission(false) ;
  Wire.requestFrom(0x68,6,true) ; //We read values from 6 registers (2 each for Acc_X, Acc_Y, and Acc_Z)
   
  /* The IMU stores Acc values in 2 seperate 8-bit registers
  We have to use left shift(<<) and bitwise or (|)
  to combine the two 8-bit values from the two registers into a 16-bit value  */
  
  Acc_raw_X = Wire.read() << 8 | Wire.read() ;
  Acc_raw_Y = Wire.read() << 8 | Wire.read() ;
  Acc_raw_Z = Wire.read() << 8 | Wire.read() ;

  //We divide the raw data with 16384, which is the value given in the MPU6050 datasheet, to obtain the value of acceleration 
  AccX = Acc_raw_X / 16384.0 ;
  AccY = Acc_raw_Y / 16384.0 ;
  AccZ = Acc_raw_Z / 16384.0 ;

  //Final value from accelerometer using the Euler formula
  Acc_X = atan((AccY)/sqrt(AccX * AccX + AccZ * AccZ))* rad_to_deg ;

  //Gyroscope
  Wire.beginTransmission(0x68) ;
  Wire.write(0x43) ; //43 is the hexadecimal address for the Gyro_X_H register
  Wire.endTransmission(false) ;
  Wire.requestFrom(0x68, 2, true) ; //We read values from the 2 registers corresponding to Gyro_X

  Gyro_raw_X = Wire.read() << 8 | Wire.read() ;

  //We divide the raw data with 131 because that's the default sensitivity value given in the data sheet
  Gyro_X = Gyro_raw_X / 131.0 ; 

  //Applying Complementary filter
  angle_X = 0.98 * (angle_X + Gyro_X * time_elapsed) + 0.02 * Acc_X ;

  //PID Loop
  error_X = angle_X - setpoint ;

  //Proportional Term
  pid_p = Kp * error_X ; 

  //Integral Term with integral windup
  if (-5 <= error_X <= 5) {
    pid_i = pid_i + Ki * (time_elapsed * error_X) ;
  }
  
  //Derivative Term
  pid_d = Kd * ((error_X - previous_error_X) / time_elapsed) ;

  //PID Output signal
  PID = pid_p + pid_i + pid_d ;

  /* Since PWM signal cannot be more than 2000 or less than 1000
  Our PID Output will have a minimum value of -1000 and a maximum value of +1000 */
  if (PID < -1000) {
    PID = -1000 ;
  }

  if (PID > 1000) {
    PID = 1000 ;
  }

  //Output Signal
  pwm_right_motor = throttle + PID ;
  pwm_left_motor = throttle - PID ;

  /* Final PWM output signal cannot be greater than 2000 or less than 1000 */
  if (pwm_right_motor < 1000) {
    pwm_right_motor = 1000 ;
  }

  if (pwm_right_motor > 2000) {
    pwm_right_motor = 2000 ;
  }

  if (pwm_left_motor < 1000) {
    pwm_left_motor = 1000 ;
  }

  if (pwm_left_motor > 2000) {
    pwm_left_motor = 2000 ;
  }

  //Sending final PWM signal to motors
  left_motor.writeMicroseconds(pwm_left_motor) ;
  right_motor.writeMicroseconds(pwm_right_motor) ;

  //Store Previous error
  previous_error_X = error_X ;


  
  Serial.println(Gyro_X) ;
  Serial.println(Acc_X) ;
  Serial.println(angle_X) ;
  Serial.println(error_X) ;
  Serial.println(PID) ;
  Serial.println(pwm_right_motor) ;
  Serial.println(pwm_left_motor) ;
  

}
