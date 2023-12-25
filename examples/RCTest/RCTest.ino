/*
   Test motors using R/C transmitter

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

#include <oneshot125.hpp>
#include <vector>

static const std::vector<uint8_t> PINS = {3, 4};

static auto motors = OneShot125(PINS);

void rcInit(void);

float rcGetThrottle();

void setup() 
{
    rcInit();

    motors.arm(); 
}

void loop() 
{
    auto pulseWidth = map(rcGetThrottle(), 0, 1, 125, 250);

    motors.set(0, pulseWidth);
    motors.set(1, pulseWidth);

    motors.run();
}
