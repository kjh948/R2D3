# R2D3
R2D2 facelift


- OS
 : pi4, buster
 : ROS1 noetic

- GPIO connection
 : gpio26 - encoderA
 : gpio19 - encoderB
 : gpio13 - zero hole
 : gpio06 - ws2812
 : gpio26 - 
 : gpio26:


- Dome
 : dome motor is connected to mh.getMotor(3)
 : using encoder A and B, position control is done
 : at startup, by checking the zero hole, the initial position should be set
- LED
 : with 2 ws2812 LED at the head
 : API that control the LED patterns