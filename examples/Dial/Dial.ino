/*
   Spin motor using potentiometer dial

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

static const uint8_t INPUT_PIN = A9;

static const std::vector<uint8_t> MOTOR_PINS = {0};

static const uint8_t LOW_PULSE_WIDTH = 170;

static auto motors = OneShot125(MOTOR_PINS);

void setup() 
{
    while (analogRead(INPUT_PIN) > 0) {
        // Wait until potentiometer reads zero
    }

    motors.arm(); 
}

void loop() 
{
    motors.set(0, map(analogRead(INPUT_PIN), 0, 1024, LOW_PULSE_WIDTH, 250));

    motors.spin();
}
