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
                _set(125);
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
