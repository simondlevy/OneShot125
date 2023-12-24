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
#include <vector>

class OneShot125Motor {

    public:


        OneShot125Motor(const uint8_t pin, const uint32_t loopFrequency=2000)
        {
            _pin = pin;
            _loopFrequency = loopFrequency;
        }

        void arm(void) 
        {
            pinMode(_pin, OUTPUT);

            for (uint8_t i=0; i<50; i++) {
                digitalWrite(_pin, LOW);
                delay(2);
            }
        }

        void spin(const uint8_t pulseWidth) 
        {
            if (pulseWidth >= 125 && pulseWidth <= 250) {

                const auto loopStartUsec = micros();

                _set(pulseWidth);

                auto time = micros();

                while ((time - loopStartUsec) < 1.0f / _loopFrequency * 1e6) {
                    time = micros();
                }
            }
        }

    private:

        uint8_t _pin;

        uint8_t _loopFrequency;

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

class OneShot125Motors {

    public:

        OneShot125Motors(
                const std::vector<uint8_t> pins, 
                const uint32_t loopFrequency=2000)
        {
            for (auto pin : pins) {
                _pins.push_back(pin);
            }

            _loopFrequency = loopFrequency;
        }

        void arm(void) 
        {
            for (auto pin : _pins) {

                pinMode(pin, OUTPUT);

                for (uint8_t i=0; i<50; i++) {
                    digitalWrite(pin, LOW);
                    delay(2);
                }
            }
        }

    private:

        std::vector<uint8_t> _pins;

        uint8_t _loopFrequency;
};
