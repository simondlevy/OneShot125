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

int m1_command_PWM;

static void commandMotors() {

  int wentLow = 0;

  int pulseStart, timer;

  int flagM1 = 0;

  digitalWrite(m1Pin, HIGH);

  pulseStart = micros();

  while (wentLow < 1) { 

    timer = micros();

    if ((m1_command_PWM <= timer - pulseStart) && (flagM1==0)) {
      digitalWrite(m1Pin, LOW);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }

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

  pinMode(m1Pin, OUTPUT);

  delay(15);

  m1_command_PWM = 125; 

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
