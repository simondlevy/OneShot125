/*
   Test ESCs.  Make sure to run Calibrate sketch first

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

#include <oneshot125.hpp>
#include <vector>

#define RX_SERIAL Serial5

// Un-comment on of these:
//#include "input_dsmx.hpp"
//#include "input_sbus.hpp"
#include "input_keyboard.hpp"

static const std::vector<uint8_t> MOTOR_PINS = {PA1, PB11, PA15, PB10};
static const std::vector<uint8_t> POWER_SWITCH_PINS = {PA0, PB12, PC8, PC15};

static auto motors = OneShot125(MOTOR_PINS);

static void enableMotor(const uint8_t id)
{
    pinMode(POWER_SWITCH_PINS[id], OUTPUT);
    digitalWrite(POWER_SWITCH_PINS[id], HIGH);
}

void setup() 
{
    Serial.begin(115200);

    inputInit();

    enableMotor(0);
    enableMotor(1);
    enableMotor(2);
    enableMotor(3);
 
    motors.arm(); 
}

void loop() 
{
    auto pulseWidth = (uint8_t)(125 * (inputGet() + 1));

    for (uint8_t k=0; k<MOTOR_PINS.size(); ++k) {
        motors.set(k, pulseWidth);
    }

    motors.run();
}
