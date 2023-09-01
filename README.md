# Description
A simple library for Micropython, intended to drive 2 motors with a TB6612FNG IC, under Raspberry Pi Pico (RP2040 chip).

**Note:** It assumes both motors are driving a 2-wheeled mouse, one of which is left and the other is right (think of a micromouse robot for example).

# Dependencies:
- [simple-PID](https://github.com/m-lundberg/simple-pid). I used Martin Lundberg's version, although you may want to use the one [supported my Micropython](https://github.com/gastmaier/micropython-simple-pid)
- [pico-MP-modules/PWMCounter](https://github.com/phoreglad/pico-MP-modules/tree/main/PWMCounter) 
