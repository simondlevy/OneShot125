/*
   Supports input using FrSky SBUS or potentiometer.

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

#include <Arduino.h>
#include <stdint.h>

#if defined(POTENTIOMETER)

///////////////////////////////////////////////////////////////////////////////////

static const uint8_t POTENTIOMETER_PIN = A0;

static void inputInit(void) 
{
}

static float inputGet(void)
{
    return analogRead(POTENTIOMETER_PIN) / 1024.f;
}


///////////////////////////////////////////////////////////////////////////////////

#elif defined(SBUS)

#include <sbus.h>

static bfs::SbusRx sbus_rx(&Serial2);

static void inputInit(void)
{
  Serial.begin(115200);

  sbus_rx.Begin();
}

static float inputGet(void)
{
    static float throttle;

    if (sbus_rx.Read()) {

        auto data = sbus_rx.data();

        throttle = (data.ch[0] - 172) / 1639.f;
    }

    return throttle;
}

#else

#endif
