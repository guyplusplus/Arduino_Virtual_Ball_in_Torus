# Arduino Virtual Ball inside a Torus

![My Toy](./sphere_in_torus_toy.png) ![Board at rest](./BoardAtRest.png)

This small Arduino construction reproduces the mechanics of a (single) ball kept inside a torus. By moving the Arduino board itself across the 6 dimensions, the ball virtually circulates inside the torus.

The behaviour is calculated realtime by the processor, based on classic mechanic equations, using moment of inertia.

This project is also registered on [Arduino Project Hub](https://create.arduino.cc/projecthub/guiliguili/virtual-ball-in-a-torus-6ee38f).

## The Math

All calculations are based on Euler angles, in the canonical (first) approach based on the ZXZ axis rotation order: &#x03c8; (psi), &#x03b8; (theta) then &#x03c6; (phi, &#x03d5; in the picture bellow). This picture from [Berkeley University](https://rotations.berkeley.edu/the-euler-angle-parameterization/) explains the used notation.

![Euler angles](./euler-angles.png)

Few additional notations:
- *l* : the torus radius
- *&#x03b1; (alpha)* : the angle of the ball in the torus referential with the x axis
- *g* : the gravity constant
- *m* : the mass of the ball
- *fr* : the friction factor

The Arduino board (the torus) is by convention located in the plane XY (p1p2 in the diagram above), where Z (p3) axis points vertically upward. When the board is steady in the earth referential, the only moment is from the gravity force:

&nbsp;&nbsp;&nbsp;&nbsp;![Moment from gravity force](./equ-gravity.svg)

The period of the oscillation of the ball is as bellow. For a 4cm radius vertically oriented torus, the period is about 0.4 seconds.

&nbsp;&nbsp;&nbsp;&nbsp;![Pendulum period](./equ-period.svg)

When the board starts to move by a human hand, the board referential can not be considered Galilean anymore. The moment of fictitious forces apply too. First as the referential translates along the 3 axis, the moment of the inertia force applies. As the referential rotates, the 3 new fictitious forces are the Euler force, the Coriolis force and the centrifugal force. The 3 forces are explained [here](https://en.wikipedia.org/wiki/Coriolis_force#Formula).

&nbsp;&nbsp;&nbsp;&nbsp;![Non Galilean fictitious forces](./non_galilean_virtual_forces.svg)

A friction force has also been added, it acts inverse to the ball speed.

The moment of the 4 fictitious forces and the friction are equal to:

&nbsp;&nbsp;&nbsp;&nbsp;![Non Galilean fictitious forces](./equ-virtualForces.svg)

So the complete equation is:

&nbsp;&nbsp;&nbsp;&nbsp;![Complete equation](./equ-complete.svg)

I have calculated all the equations myself, so if you find any error, please contact me!

## Arduino Schematic

I used [Fritzing](https://fritzing.org/) for the design for the board. The raw file is [here](./BallInTorus_FritzingSchematics.fzz).

![schematic](./BallInTorus_FritzingSchematics_bb.png)

## The Code

The MPU6050 library is from Jeff Rowberg on Git [here](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050), using the interruption model (frequency at 100Hz) to receive measurements.

Callibration is done with a home grown technique as the provided library did not work well for me. The board shall be placed in an horizontal position first, it can then be switched on (or reset). The first 3 seconds are in wait time to wait for the MPU to stabilize (it takes actually longer than this), the last second is used to average the current position and orientation to define the reference quarternion `sensorQ0` and its conjugate. The callibration time can be changed via the variable `CALLIBRATION_STEPS`.

There are a number of debug variables to output debug via serial line.

## Simple Test [YouTube]

[![](http://img.youtube.com/vi/k5dkkXRLZqw/0.jpg)](http://www.youtube.com/watch?v=k5dkkXRLZqw "Simple Movement")

## Thanks

Thanks to [Ole Eichhorn at Thingiverse](https://www.thingiverse.com/thing:324904) for the picture of the toy.