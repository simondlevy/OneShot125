/*
   "Uncalibrate" ESCs by setting min pulse width to 125 uSec and max to 250

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

static const std::vector<uint8_t> MOTOR_PINS = {0, 1};

static auto motors = OneShot125(MOTOR_PINS);

static uint8_t state;

void serialEvent(void)
{
    while (Serial.available()) {
        Serial.read();
    }

    state++;
}

static void setMotors(const uint8_t pulseWidth)
{
    for (auto pin : MOTOR_PINS) {
        motors.set(pin, pulseWidth);
    }
}

void setup() 
{
    Serial.begin(115200);

    motors.arm();
}

void loop() 
{
    static uint32_t timePrev;
    static char message[100];

    switch (state) {

        case 0:
            strcpy(message, "Plug in battery and hit Enter to begin");
            break;

         case 1:
            strcpy(message, "Wait for melody to end; then hit Enter");
            setMotors(250);
            break;

        case 2:
            strcpy(message, "Wait for melody to end; then unplug battery");
            setMotors(125);
            break;
    }

    motors.run();

    auto time = millis();

    if (time - timePrev > 1000) {
        Serial.println(message);
        timePrev = time;
    }
}
