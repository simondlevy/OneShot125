/*
   Header-only library for running OneShot125 ESCs from an Arduino-compatible
   board

   Adapted from the code in https://github.com/nickrehm/dRehmFlight

   This file is part of OneShot125.

   OneShot125 is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   OneShot125 is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   OneShot125. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <Arduino.h>

#include <vector>

class OneShot125 {

    public:

        OneShot125( const std::vector<uint8_t> pins)
        {
            for (auto pin : pins) {
                _pins.push_back(pin);
                _pulseWidths.push_back(0);
                _flags.push_back(false);
            }
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

        void set(const uint8_t index, const uint8_t pulseWidth)
        {
            _pulseWidths[index] = 
                index < _pins.size() && 
                (pulseWidth >= 125 && pulseWidth <= 250) ? pulseWidth : 
                125;
        }

        void run(void)
        {
            for (uint8_t k=0; k<_pins.size(); ++k) {
                digitalWrite(_pins[k], HIGH);
                _flags[k] = false;
            }

            const auto pulseStart = micros();

            uint8_t wentLow = 0;

            while (wentLow < _pins.size()) {

                const auto timer = micros();

                for (uint8_t k=0; k<_pins.size(); ++k) {

                    if ((_pulseWidths[k] <= timer - pulseStart) && !_flags[k]) {
                        digitalWrite(_pins[k], LOW);
                        wentLow++;
                        _flags[k] = true;
                    }
                }
            }
        }

    private:

        std::vector<uint8_t> _pins;

        std::vector<uint8_t> _pulseWidths;

        std::vector<bool> _flags;
};
