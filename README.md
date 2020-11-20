# Arduino Virtual Ball in Torus

![My Toy](./sphere_in_torus_toy.png)

This small Arduino construction reproduces the machanics of a (single) ball kept inside a torus. By moving the Arduino board itself accross the 6 dimansions, the ball circulates inside.

The behavior is calculated realtime by the processor, based on standard mechanics equations, using moment of inertia.

All calculations are based on Euler angles, in the canonical (first) approach based on the ZXZ angles: &#x03c8; (psi), &#x03b8; (theta) then &#x03c6; (phi). This picture from [Berkeley University](https://rotations.berkeley.edu/the-euler-angle-parameterization/) explains the used notation.

![Euler angles](./euler-angles.png)

Assuming a fixed board, where its z axis points verticaly upward, the only moment is from the gravity force:

![Moment from gravity force](./equ-gravity.svg)

Then as the board is actually a non galilean referencial, moment of virtual forces appply. First as the referential translate, the monent of the inertia force applies. As the referential rotates, the 3 forces are the Euler force, the Coriolis force and the centrifugal force. The 3 forces are explained [here](https://en.wikipedia.org/wiki/Coriolis_force).

![Non Galilean virtual forces](./non_galilean_virtual_forces.svg)

Each moment is equal to:

![Non Galilean virtual forces](./equ-virtualForces.svg)

So the complete equation is:

![Complete equation](./equ-complete.svg)

I have calculated all the equations myself, so if you find any error, please contact me!

Thank you to [STLFinder](https://www.stlfinder.com/model/voronoi-donut-with-holes-inside-saX8Zuwv/659237/) for the picture of the toy.