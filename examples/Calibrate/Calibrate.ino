/*
   Calibrate ESCs using R/C transmitter

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

// Un-comment one of these
#define SBUS
//#define POTENTIOMETER

#include "input.hpp"

static const std::vector<uint8_t> MOTOR_PINS = {0, 1};

static auto motors = OneShot125(MOTOR_PINS);

static void setMotors(const uint8_t pulseWidth)
{
    for (auto pin : MOTOR_PINS) {
        motors.set(pin, pulseWidth);
    }
}

void setup() 
{
    pinMode(LED_BUILTIN, OUTPUT);

    inputInit();

    motors.arm();
}

void loop() 
{
    auto input = inputGet();

    if (input == 0.0) {
        digitalWrite(LED_BUILTIN, LOW);
        setMotors(125);
    }

    else if (input == 1.0) {
        digitalWrite(LED_BUILTIN, HIGH);
        setMotors(250);
    }

    motors.run();

}
