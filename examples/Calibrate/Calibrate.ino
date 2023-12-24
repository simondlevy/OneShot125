/*
   Calibrate ESCs

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
           break;

        case 1:
            motors.set(0, 250);
            motors.set(1, 250);
            break;

        case 2:
            motors.set(0, 125);
            motors.set(1, 125);
            break;

        case 3:
            motors.set(0, 150);
            motors.set(1, 150);
            break;
     }

    motors.spin();

    auto time = millis();

    if (time - timePrev > 1000) {
        //Serial.println(message);
        timePrev = time;
    }
}
