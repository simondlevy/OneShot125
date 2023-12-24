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

static const uint8_t PIN = 0;
static const uint32_t UPDATE_FREQUENCY = 10;
static const uint8_t LOW_PULSE_WIDTH = 170;

static auto esc = OneShot125Motor(PIN);

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

    esc.arm(); 

    delay(2000);

    Serial.println("Hit Enter to stop ... ");

    pulseWidth = 125;

    pulseIncrement = +1;
}

void loop() 
{
    auto time = micros();

    esc.spin(pulseWidth); 

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
            if (pulseWidth == LOW_PULSE_WIDTH) {
                pulseIncrement = +1;
            }
        }
    }
}
