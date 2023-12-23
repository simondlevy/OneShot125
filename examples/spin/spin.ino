#include <PWMServo.h> 

#include <stdint.h>

static unsigned long current_time;

static void loopRate(int freq) {

  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();

  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

static int m1Pin = 0;
static int m2Pin = 1;
static int m3Pin = 2;
static int m4Pin = 3;
static int m5Pin = 4;
static int m6Pin = 5;

int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;

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

void setup() 
{
  Serial.begin(500000); 

  delay(500);

  pinMode(13, OUTPUT); 
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  pinMode(m5Pin, OUTPUT);
  pinMode(m6Pin, OUTPUT);

  delay(15);

  m1_command_PWM = 125; 
  m2_command_PWM = 125;
  m3_command_PWM = 125;
  m4_command_PWM = 125;
  m5_command_PWM = 125;
  m6_command_PWM = 125;

  armMotors(); 
}

void loop() 
{
  current_time = micros();      

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
