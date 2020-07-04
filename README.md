# HexcopterController
This project aims to be hexcopter control software for a hexcopter matching the hardware description below. The end goal of this project is to have a system capable of fully autonomous flight using instruments connected to the controller such as an IMU, GPS and an altimeter.

Manual control of the hexcopter is achieved by interpreting signals from the radio (PPM) and then doing the appropriate calculations to control the ESCs (PWM) which in turn control the motors and produce the desired outcome. At the moment control is not based on a kinematic model - this will likely be required for autonomous flight.

This is being developed on a specific hexcopter which is operated in the UK in accordance with UK drone guidelines and law. Line of sight to the hexcopter is kept at all times during flight and the flight level is kept within the allowed range, testing is carried out on private land away from any people and safety precautions are taken to manage risks. When applicable these rules will be coded in to the controller to prevent accidental breaches.

## Flight Hardware
1 x F550 Hexcopter Frame

6 x Turnigy Multistar 2213-980 V2 Motors

6 x Turnigy Multistar 32bit 20A ESC

2 x 3S (~11.1V) Cells in series rated at 3000mAh, 65c each

1 x U5 PRO UBEC rated 5V@5A

## Controller Hardware (incl. Radios)

1 x Turnigy IA8 Receiver (AFHDS 2A system) and Turnigy 9X Transmitter with module for the receiver

1 x Arduino 101 (This was chosen for it's built in IMU and Bluetooth but in future this may be swapped for an Arduino MEGA or similar with I2C based IMU and Bluetooth) 

1 x Adafruit Servo shield 16 x 12bit PWM (part#: 1411)

