/*
   Supports input using FrSky SBUS receivers

   Additional library required: https://github.com/bolderflight/sbus

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

#pragma once

#include <sbus.h>

static bfs::SbusRx rx(&RX_SERIAL);

static void inputInit(void)
{
  rx.Begin();
}

static float inputGet(void)
{
    static float throttle;

    if (rx.Read()) {

        auto data = rx.data();

        throttle = (data.ch[0] - 172) / 1639.f;
    }

    return throttle;
}
