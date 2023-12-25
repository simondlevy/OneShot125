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
[video](https://www.youtube.com/watch?v=b7x2g3awrsw)), 
the API for the library is very simple: 

1. Declare a ```OneShot125``` object, passing it a list of pins.

2. In your ```setup()``` function, call the object's ```arm()``` function to
arm the motors.

3. In your ```loop()``` function, call the object's ```set()``` function to set the pulse
width on each pin; then call the ```run()``` function to run the motors at the specified
pulse widths.

```
static const std::vector<uint8_t> PINS = {0};

static auto motors = OneShot125(PINS);

void setup() 
{
    inputInit();

    motors.arm(); 
}

void loop() 
{
    auto pulseWidth = map(inputGet(), 0, 1, 125, 250);

    motors.set(0, pulseWidth);

    motors.run();
}
```

In this sketch, ```inputInit()``` is a (possibly empty) function that sets up
your input device, such as an R/C receiver, or the potentiometer used in the
video.  The ```inputGet()``` function returns a floating-point value between 0
and 1, such as the normalized throttle or potentiometer reading.

## Calibrating your ESCs

Before using this library in your project, you should 
calibrate the ESCs.  Calibration means that when your input (e.g., throttle
stick) is in its max position, you send the maximum pulse with of 250 uSec to
each ESC, and when your input is at its minimum you send the minimum 125uSec
pulse width.  Because of the potential to spin the motors unexpectedly, you
should <b>make sure to remove the propellers from your motors before calibrating</b>.

The
[Calibration sketch](https://github.com/simondlevy/TeensyOneShot125/blob/main/examples/Calibrate/Calibrate.ino)
allows you to do calibrate your ESCs using either the
sort of potentiometer shown in the video, or the more common method of and R/C
transmitter/receiver. This sketch supports the popular DSMX and SBUS protocols,
which you can select by un-commenting on of the ```#include``` lines at the top of the
sketch.  The DSMX version requires 
[this Arduino library](https://github.com/simondlevy/DSMRX),
and the SBUS version requires 
[this library](https://github.com/bolderflight/sbus).  This 
[video](https://youtu.be/kbhQ4j4VNBA)
shows me using the SBUS version, with a FrSky transmitter and receiver.
As you can see in the video, calibration uses the following steps:

1. With the battery and Teensy unpowered, turn on the transmitter and
max-out the throttle.

2. Plug in the Teensy board.  If it receiving full throttle correctly,
the LED will turn on.

3. Plug in the battery.  You should immediately hear the 
[calibration starting](https://simondlevy.academic.wlu.edu/files/2023/12/esc_calibration1.mp3)
melody.

4. Lower the throttle stick on the transmitter all the way. You should see the
LED turn off and should immediately hear the [calibration
completed](https://simondlevy.academic.wlu.edu/files/2023/12/esc_calibration2.mp3)
melody.
