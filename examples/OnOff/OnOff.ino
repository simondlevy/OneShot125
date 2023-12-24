/*
   Simple motor spinning example using Teensy-OneShot125

   Prompts for stopping and starting motor.

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

static const std::vector<uint8_t> PINS = {0};

static const uint32_t UPDATE_FREQUENCY = 10;

static auto motors = OneShot125(PINS);

static uint8_t pulseWidth;

static int8_t pulseIncrement;

static bool gotInput;

void serialEvent(void)
{
    while (Serial.available()) {
        Serial.read();
    }

    gotInput = true;
}

void setup() 
{
    Serial.begin(115200);

    Serial.println("Hit Enter to begin ...");

    while (!gotInput) {
        delay(1000);
    }

    gotInput = false;

    Serial.println("Arming ...");

    motors.arm(); 

    delay(2000);

    Serial.println("Hit Enter to stop ... ");

    pulseWidth = 125;

    pulseIncrement = +1;
}

void loop() 
{
    auto time = micros();

    motors.set(0, pulseWidth); 

    motors.spin(); 

    if (gotInput) {
        pulseWidth = 125;
    }

    else {

        static uint32_t prev;

        if ((time - prev) > 1.0f / UPDATE_FREQUENCY * 1e6) {
            pulseWidth += pulseIncrement;
            prev = time;
            if (pulseWidth == 250) {
                pulseIncrement = -1;
            }
            if (pulseWidth == 125) {
                pulseIncrement = +1;
            }
        }
    }
}
