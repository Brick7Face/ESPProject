# ESPProject
HTML and C code for Adafruit Feather Huzzah are included. 

This project controls a DC motor and servo motor from an HTMM websocket and outputs information about the structure the board is attached to. The code controls an Adafruit IMU as well. Here is a list of the hardware used in this project:
- Adafruit Feather Huzzah
- Adafruit Stepper + DC Motor FeatherWing 
- Adafruit LSM9DSO Accelerometer + Gyro + Magnetometer 9-DOF Breakout Board
- any DC motor
- any servo motor (a stepper motor is preferred, but both work)
  
For basic information on the hardware, here are some links:<br>
https://learn.adafruit.com/adafruit-feather-huzzah-esp8266/using-arduino-ide<br>
https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/usage<br>
https://learn.adafruit.com/adafruit-lsm9ds0-accelerometer-gyro-magnetometer-9-dof-breakouts/wiring-and-test<br>

Other Tips:
The servo motor is wired to the FeatherWing using power, ground, and a data wire. The power and ground wires match with a stepper motor, and the data wire works in either of the other two pins. Otherwise, the servo is treated as a stepper motor in the code. 
