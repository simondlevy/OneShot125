/*
   Supports input using FrSky SBUS, Spektrum DSMX, or potentiometer.

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

// Un-comment on of these
#define SBUS
//#define DSMX
//#define POTENTIOMETER

///////////////////////////////////////////////////////////////////////////////////

#if defined(POTENTIOMETER)

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

///////////////////////////////////////////////////////////////////////////////////

#elif defined(DSMX)

#include <dsmrx.h>

static const uint8_t DSMX_CHANNELS = 8;

static Dsm2048 dsm_rx;

void serialEvent2(void)
{
    while (Serial2.available()) {
        dsm_rx.handleSerialEvent(Serial2.read(), micros());
    }
}

static void inputInit(void)
{
    Serial2.begin(115200);
}

static float inputGet(void)
{
    static float throttle;

    if (dsm_rx.gotNewFrame()) {

        float values[DSMX_CHANNELS] = {};

        dsm_rx.getChannelValues(values, DSMX_CHANNELS);

        throttle = (values[0] + 1) / 2;

    }

    delay(10);

    return throttle;
}

#else

#endif
