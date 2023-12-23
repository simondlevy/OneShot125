#include <PWMServo.h> 

#include <stdint.h>

static const uint8_t m1Pin = 0;

static void set(const uint8_t pulseWidth) 
{
  digitalWrite(m1Pin, HIGH);

  auto pulseStart = micros();

  while (true) { 

    if (pulseWidth <= micros() - pulseStart) {

      digitalWrite(m1Pin, LOW);

      break;

    }
  }
}

static void arm() 
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

  arm(); 
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
