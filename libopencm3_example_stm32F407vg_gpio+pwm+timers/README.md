# Software_nucleo

Readme
======

This is the source code for the stm32l053r8-nucleo used in the project014.
It is based on a fork of libopencm3 as stm32l0 port is still incomplete in the main repo.
For now, clock chain is configured and it blinks a led

Prerequisites
-------------
- An ARM toolchain 
https://launchpad.net/gcc-arm-embedded
- OpenOCD

Build it
--------
In Software_Nucleo/ :

    make

it should build libopencm3 and miniblink

Flash it
--------
In miniblink/ : 

    make flash


