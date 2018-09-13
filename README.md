# SNF2018Orbit
Repository for Orbit, an installation done by LiteWerkz for the Singapore Night Festival 2018

# Code List
1. snf2018-r2-multiuniverse.ino
   Code example for running multiple universes on one device, without having to max out the first universe.
2. snf2018-r1.ino
   Code example of what we used during the Singapore Night Fest. Mainly used for planets with less than 170 LEDs. Reads resolume Artnet        output, displays the output on the LEDs, reads the MPU6050 accel and gyro for angular speed in the Z axis and changes the LED brighness    accordingly.
3. blinkanled-mod.ino
   Our modified version of the Blink an LED example from Particle. This version puts the Photon into manual mode then sets it to use          dynamic IP. The LED blinking helps us verify that the Photon is able to run new code.
