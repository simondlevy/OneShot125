#include <PWMServo.h> 

#include <stdint.h>


class OneShot125 {

    public:

        OneShot125(const uint8_t pin)
        {
            _pin = pin;
        }

        void arm(void) 
        {
            pinMode(_pin, OUTPUT);

            for (uint8_t i=0; i<50; i++) {
                set(125);
                delay(2);
            }
        }

        void set(const uint8_t pulseWidth) 
        {
            if (pulseWidth >= 125 && pulseWidth <= 250) {
                _set(pulseWidth);
            }
        }

    private:

        uint8_t _pin;

        void _set(const uint8_t pulseWidth) 
        {
            digitalWrite(_pin, HIGH);

            auto pulseStart = micros();

            while (true) { 

                if (pulseWidth <= micros() - pulseStart) {

                    digitalWrite(_pin, LOW);

                    break;

                }
            }
        }

};

///////////////////////////////////////////////////////////

static const uint8_t ESC_PIN = 0;

static auto esc = OneShot125(ESC_PIN);

static void loopDelay(const uint32_t freq, const uint32_t loopStartUsec) 
{

    float invFreq = 1.0/freq*1000000.0;
    unsigned long checker = micros();

    while (invFreq > (checker - loopStartUsec)) {
        checker = micros();
    }
}

static uint8_t pulseWidth;

void setup() 
{
    esc.arm(); 

    pulseWidth = 125;
}

void loop() 
{

    const auto loopStartUsec = micros();      

    esc.set(pulseWidth); 

    loopDelay(2000, loopStartUsec); 

    static uint32_t prev;
    auto msec = millis();
    if (msec - prev > 100) {
        pulseWidth += 1;
        prev = msec;
        if (pulseWidth == 250) {
            pulseWidth = 130;
        }
    }
}
