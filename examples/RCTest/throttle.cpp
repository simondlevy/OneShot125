#include <sbus.h>

static bfs::SbusRx sbus_rx(&Serial1);

void rcInit(void)
{
  Serial.begin(115200);

  sbus_rx.Begin();
}

float rcGetThrottle(void)
{
    static float throttle;

    if (sbus_rx.Read()) {

        auto data = sbus_rx.data();

        throttle = (data.ch[0] - 172) / 1639.f;
    }

    return throttle;
}
