/*
   Header-only library for running OneShot125 ESCs from a Teensy board

   Adapted from the code in https://github.com/nickrehm/dRehmFlight

   This file is part of Teensy-OneShot125.

   Teensy-OneShot125 is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Teensy-OneShot125 is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Teensy-OneShot125. If not, see <https://www.gnu.org/licenses/>.
 */

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
