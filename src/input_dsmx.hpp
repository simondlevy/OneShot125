/*
   Supports input using Spektrum DSMX receivers.

   Additional library required: https://github.com/simondlevy/DSMRX

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

#pragma once

#include <dsmrx.hpp>

static const uint8_t DSMX_CHANNELS = 8;

static Dsm2048 rx;

static void inputInit(void)
{
    RX_SERIAL.begin(115200);
}

static float inputGet(void)
{
    while (RX_SERIAL.available()) {
        rx.parse(RX_SERIAL.read(), micros());
    }

    static float throttle;

    if (rx.gotNewFrame()) {

        float values[DSMX_CHANNELS] = {};

        rx.getChannelValues(values, DSMX_CHANNELS);

        throttle = (values[0] + 1) / 2;

    }

    delay(10);

    return throttle;
}
