<a href="https://www.youtube.com/watch?v=b7x2g3awrsw"><img src="screenshot.jpg" width=500></a>

This a simple, header-only C++ library for using a 
[Teensy](https://www.pjrc.com/teensy/)
microcontroller board to run brushless motors via an electronic speed
controller (ESC) supporting the popular
[OneShot125](https://oscarliang.com/oneshot125-esc-quadcopter-fpv/) protocol.  I adapted the code from
Nicholas Rehm's awesome
[dRehmFlight](https://github.com/nickrehm/dRehmFlight) repository, with the goal of allowing
fellow Teensy lovers to use this kind of ESC in other projects.

As shown in this
[sketch](https://github.com/simondlevy/TeensyOneShot125/tree/main/examples/Dial/Dial.ino) 
(which I used for making this
[video](https://www.youtube.com/watch?v=b7x2g3awrsw)), 
the API for the library
is very simple: 

1. In your ```setup()``` function, declare a OneShot125 object, passing it a
list of pins; then call the ```arm()``` method to arm the motors

2. In your ```loop()``` function, call the ```set()``` method to set the pulse
width for each motor; then call the ```spin()``` method to spin the motors.

As shown in this 
[sketch](https://github.com/simondlevy/TeensyOneShot125/tree/main/examples/TwoMotors/TwoMotors.ino), 
the library also supports running more than one motor at a time.

If you dont' have a potentiometer (variable resitor with dial) as shown in the
video, you can try this [sketch](https://github.com/simondlevy/TeensyOneShot125/blob/main/examples/OnOff/OnOff.ino), 
which prompts for you starting and stopping the motor and automatically spins it at different RPMs.
