<a href="https://www.youtube.com/watch?v=b7x2g3awrsw"><img src="screenshot.jpg" width=500></a>

This a simple, header-only C++ library for using a 
[Teensy](https://www.pjrc.com/teensy/)
microcontroller board to run brushless motors via an electronic speed
controller (ESC) supporting the popular
[OneShot125](https://oscarliang.com/oneshot125-esc-quadcopter-fpv/) protocol.  I adapted the code from
Nicholas Rehm's awesome
[dRehmFlight](https://github.com/nickrehm/dRehmFlight) repository, with the goal of allowing
fellow Teensy lovers to use this kind of ESC in other projects.

As shown in the sketch below (which I used for making this
[video](https://www.youtube.com/watch?v=b7x2g3awrsw)), the API for the library
is extremely simple: you just declare a OneShot125 object, call its
```arm()``` method to arm it, and call its ```set()``` method periodically to
set the pulse width to a value between 125 (off) and 250 (max spin)
microseconds:

```
#include <oneshot125.hpp>

static const uint8\_t INPUT\_PIN = A9;
static const uint8\_t MOTOR\_PIN = 0;
static const uint8\_t LOW\_PULSE\_WIDTH = 170;

static auto esc = OneShot125(MOTOR\_PIN);

void setup() 
{
    while (analogRead(INPUT_PIN) > 0) {
        // Wait until potentiometer reads zero
    }

    esc.arm(); 
}

void loop() 
{
    esc.set(map(analogRead(INPUT_PIN), 0, 1024, LOW_PULSE_WIDTH, 250));
}
```

If you dont' have a potentiometer (variable resitor with dial) as shown in the
video, you can try this [sketch](https://github.com/simondlevy/TeensyOneShot125/blob/main/examples/OnOff/OnOff.ino), 
which automatically spins the motors at different RPMs.
