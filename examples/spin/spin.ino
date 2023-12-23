#define USE_PWM_RX

static const uint8_t num_DSM_channels = 6; 

#define USE_MPU6050_I2C 

#define GYRO_250DPS 

#define ACCEL_2G 

#include <Wire.h>     
#include <SPI.h>      
#include <PWMServo.h> 

#if defined USE_MPU6050_I2C
  #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
  #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
  #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
  #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
  #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
  #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
  #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
  #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
  #define GYRO_FS_SEL_250    mpu9250.GYRO_RANGE_250DPS
  #define GYRO_FS_SEL_500    mpu9250.GYRO_RANGE_500DPS
  #define GYRO_FS_SEL_1000   mpu9250.GYRO_RANGE_1000DPS                                                        
  #define GYRO_FS_SEL_2000   mpu9250.GYRO_RANGE_2000DPS
  #define ACCEL_FS_SEL_2     mpu9250.ACCEL_RANGE_2G
  #define ACCEL_FS_SEL_4     mpu9250.ACCEL_RANGE_4G
  #define ACCEL_FS_SEL_8     mpu9250.ACCEL_RANGE_8G
  #define ACCEL_FS_SEL_16    mpu9250.ACCEL_RANGE_16G
#endif

#if defined GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0
#endif

static unsigned long current_time;

static void loopRate(int freq) {

  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();

  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

unsigned long channel_1_fs = 1000; 
unsigned long channel_2_fs = 1500; 
unsigned long channel_3_fs = 1500; 
unsigned long channel_4_fs = 1500; 
unsigned long channel_5_fs = 2000; 
unsigned long channel_6_fs = 2000; 

float B_madgwick = 0.04;  
float B_accel = 0.14;     
float B_gyro = 0.1;       
float B_mag = 1.0;        

float MagErrorX = 0.0;
float MagErrorY = 0.0; 
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;

float i_limit = 25.0;     
float maxRoll = 30.0;     
float maxPitch = 30.0;    
float maxYaw = 160.0;     

float Kp_roll_angle = 0.2;    
float Ki_roll_angle = 0.3;    
float Kd_roll_angle = 0.05;   
float B_loop_roll = 0.9;      
float Kp_pitch_angle = 0.2;   
float Ki_pitch_angle = 0.3;   
float Kd_pitch_angle = 0.05;  
float B_loop_pitch = 0.9;     

float Kp_roll_rate = 0.15;    
float Ki_roll_rate = 0.2;     
float Kd_roll_rate = 0.0002;  
float Kp_pitch_rate = 0.15;   
float Ki_pitch_rate = 0.2;    
float Kd_pitch_rate = 0.0002; 

float Kp_yaw = 0.3;           
float Ki_yaw = 0.05;          
float Kd_yaw = 0.00015;       

const int ch1Pin = 15; 
const int ch2Pin = 16; 
const int ch3Pin = 17; 
const int ch4Pin = 20; 
const int ch5Pin = 21; 
const int ch6Pin = 22; 
const int PPM_Pin = 23;

const int m1Pin = 0;
const int m2Pin = 1;
const int m3Pin = 2;
const int m4Pin = 3;
const int m5Pin = 4;
const int m6Pin = 5;

const int servo1Pin = 6;
const int servo2Pin = 7;
const int servo3Pin = 8;
const int servo4Pin = 9;
const int servo5Pin = 10;
const int servo6Pin = 11;
const int servo7Pin = 12;
PWMServo servo1;  
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;
PWMServo servo5;
PWMServo servo6;
PWMServo servo7;

float dt;
unsigned long prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

#if defined USE_SBUS_RX
  SBUS sbus(Serial5);
  uint16_t sbusChannels[16];
  bool sbusFailSafe;
  bool sbusLostFrame;
#endif
#if defined USE_DSM_RX
  DSM1024 DSM;
#endif

float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; 
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

bool armedFly = false;

static void getDesState() {

  thro_des = (channel_1_pwm - 1000.0)/1000.0; 
  roll_des = (channel_2_pwm - 1500.0)/500.0; 
  pitch_des = (channel_3_pwm - 1500.0)/500.0; 
  yaw_des = (channel_4_pwm - 1500.0)/500.0; 
  roll_passthru = roll_des/2.0; 
  pitch_passthru = pitch_des/2.0; 
  yaw_passthru = yaw_des/2.0; 

  thro_des = constrain(thro_des, 0.0, 1.0); 
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; 
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; 
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; 
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

static void scaleCommands() {

  m2_command_PWM = m2_command_scaled*125 + 125;
  m3_command_PWM = m3_command_scaled*125 + 125;
  m4_command_PWM = m4_command_scaled*125 + 125;
  m5_command_PWM = m5_command_scaled*125 + 125;
  m6_command_PWM = m6_command_scaled*125 + 125;

  m2_command_PWM = constrain(m2_command_PWM, 125, 250);
  m3_command_PWM = constrain(m3_command_PWM, 125, 250);
  m4_command_PWM = constrain(m4_command_PWM, 125, 250);
  m5_command_PWM = constrain(m5_command_PWM, 125, 250);
  m6_command_PWM = constrain(m6_command_PWM, 125, 250);

  s1_command_PWM = s1_command_scaled*180;
  s2_command_PWM = s2_command_scaled*180;
  s3_command_PWM = s3_command_scaled*180;
  s4_command_PWM = s4_command_scaled*180;
  s5_command_PWM = s5_command_scaled*180;
  s6_command_PWM = s6_command_scaled*180;
  s7_command_PWM = s7_command_scaled*180;

  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  s2_command_PWM = constrain(s2_command_PWM, 0, 180);
  s3_command_PWM = constrain(s3_command_PWM, 0, 180);
  s4_command_PWM = constrain(s4_command_PWM, 0, 180);
  s5_command_PWM = constrain(s5_command_PWM, 0, 180);
  s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  s7_command_PWM = constrain(s7_command_PWM, 0, 180);

}

static void getCommands() {
}

static void failSafe() {

  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  }
}

static void commandMotors() {

  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0;
  int flagM2 = 0;
  int flagM3 = 0;
  int flagM4 = 0;
  int flagM5 = 0;
  int flagM6 = 0;

  digitalWrite(m1Pin, HIGH);
  digitalWrite(m2Pin, HIGH);
  digitalWrite(m3Pin, HIGH);
  digitalWrite(m4Pin, HIGH);
  digitalWrite(m5Pin, HIGH);
  digitalWrite(m6Pin, HIGH);
  pulseStart = micros();

  while (wentLow < 6 ) { 
    timer = micros();
    if ((m1_command_PWM <= timer - pulseStart) && (flagM1==0)) {
      digitalWrite(m1Pin, LOW);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
    if ((m2_command_PWM <= timer - pulseStart) && (flagM2==0)) {
      digitalWrite(m2Pin, LOW);
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
    if ((m3_command_PWM <= timer - pulseStart) && (flagM3==0)) {
      digitalWrite(m3Pin, LOW);
      wentLow = wentLow + 1;
      flagM3 = 1;
    }
    if ((m4_command_PWM <= timer - pulseStart) && (flagM4==0)) {
      digitalWrite(m4Pin, LOW);
      wentLow = wentLow + 1;
      flagM4 = 1;
    } 
    if ((m5_command_PWM <= timer - pulseStart) && (flagM5==0)) {
      digitalWrite(m5Pin, LOW);
      wentLow = wentLow + 1;
      flagM5 = 1;
    } 
    if ((m6_command_PWM <= timer - pulseStart) && (flagM6==0)) {
      digitalWrite(m6Pin, LOW);
      wentLow = wentLow + 1;
      flagM6 = 1;
    } 
  }
}

static void armMotors() {

  for (int i = 0; i <= 50; i++) {
    commandMotors();
    delay(2);
  }
}

void calibrateESCs() {

   while (true) {
      prev_time = current_time;      
      current_time = micros();      
      dt = (current_time - prev_time)/1000000.0;

      digitalWrite(13, HIGH); 

      getCommands(); 
      failSafe(); 
      getDesState(); 
      getDesState(); 

      m1_command_scaled = thro_des;
      m2_command_scaled = thro_des;
      m3_command_scaled = thro_des;
      m4_command_scaled = thro_des;
      m5_command_scaled = thro_des;
      m6_command_scaled = thro_des;
      s1_command_scaled = thro_des;
      s2_command_scaled = thro_des;
      s3_command_scaled = thro_des;
      s4_command_scaled = thro_des;
      s5_command_scaled = thro_des;
      s6_command_scaled = thro_des;
      s7_command_scaled = thro_des;
      scaleCommands(); 

      servo1.write(s1_command_PWM); 
      servo2.write(s2_command_PWM);
      servo3.write(s3_command_PWM);
      servo4.write(s4_command_PWM);
      servo5.write(s5_command_PWM);
      servo6.write(s6_command_PWM);
      servo7.write(s7_command_PWM);
      commandMotors(); 

      loopRate(2000); 
   }
}

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq){

  float diffParam = (param_max - param_min)/(fadeTime*loopFreq); 

  if (state == 1) { 
    param = param + diffParam;
  }
  else if (state == 0) { 
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max); 

  return param;
}

float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq){

  if (param > param_des) { 
    float diffParam = (param_upper - param_des)/(fadeTime_down*loopFreq);
    param = param - diffParam;
  }
  else if (param < param_des) { 
    float diffParam = (param_des - param_lower)/(fadeTime_up*loopFreq);
    param = param + diffParam;
  }

  param = constrain(param, param_lower, param_upper); 

  return param;
}

static void loopBlink() {

  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); 

    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}

static void setupBlink(int numBlinks,int upTime, int downTime) {

  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

void setup() {
  Serial.begin(500000); 
  delay(500);

  pinMode(13, OUTPUT); 
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  pinMode(m5Pin, OUTPUT);
  pinMode(m6Pin, OUTPUT);
  servo1.attach(servo1Pin, 900, 2100); 
  servo2.attach(servo2Pin, 900, 2100);
  servo3.attach(servo3Pin, 900, 2100);
  servo4.attach(servo4Pin, 900, 2100);
  servo5.attach(servo5Pin, 900, 2100);
  servo6.attach(servo6Pin, 900, 2100);
  servo7.attach(servo7Pin, 900, 2100);

  digitalWrite(13, HIGH);

  delay(5);

  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;

  delay(5);

  servo1.write(0); 
  servo2.write(0); 
  servo3.write(0); 
  servo4.write(0);
  servo5.write(0);
  servo6.write(0);
  servo7.write(0);

  delay(5);

  m1_command_PWM = 125; 
  m2_command_PWM = 125;
  m3_command_PWM = 125;
  m4_command_PWM = 125;
  m5_command_PWM = 125;
  m6_command_PWM = 125;
  armMotors(); 

  setupBlink(3,160,70); 

}

void loop() 
{

  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  loopBlink(); 

  commandMotors(); 

  loopRate(2000); 

  static uint32_t prev;
  auto msec = millis();
  if (msec - prev > 100) {
      m1_command_PWM += 1;
      prev = msec;
      if (m1_command_PWM == 250) {
          m1_command_PWM = 130;
      }
  }
}
