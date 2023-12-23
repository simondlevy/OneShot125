#include <PWMServo.h> 

#include <stdint.h>

static const uint8_t m1Pin = 0;

static void set(const uint8_t pulseWidth) 
{
  int wentLow = 0;
  int flagM1 = 0;

  digitalWrite(m1Pin, HIGH);

  auto pulseStart = micros();

  while (wentLow < 1) { 

    auto timer = micros();

    if ((pulseWidth <= timer - pulseStart) && (flagM1==0)) {
      digitalWrite(m1Pin, LOW);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
  }
}

static void armMotors() 
{
  for (int i = 0; i <= 50; i++) {
    set(125);
    delay(2);
  }
}

///////////////////////////////////////////////////////////

static void loopDelay(const uint32_t freq, const uint32_t loopStartUsec) 
{

  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();

  while (invFreq > (checker - loopStartUsec)) {
    checker = micros();
  }
}

static uint8_t m1_command_PWM;

void setup() 
{
  pinMode(m1Pin, OUTPUT);

  delay(15);

  m1_command_PWM = 125; 

  armMotors(); 
}

void loop() 
{
  const auto loopStartUsec = micros();      

  set(m1_command_PWM); 

  loopDelay(2000, loopStartUsec); 

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
