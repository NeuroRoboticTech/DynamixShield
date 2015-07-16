# DynamixShield
Libraries and sketches related to the DynamixShield for the Arduino microcontroller.
DynamixShield is a shield for the Arduino Due, Zero, and Mega that allows you to directly
control Dynamixel smart servos from your Arduino. It also has numerous Grove connectors for
adding other sensors and controllers. Grove is a modular connector system that has lots of 
different hardware plug-ins available for purchase. This makes it easy to quickly get a new
robot up and running with just a few connections.

### Features
 - Able to write to Dynamixel servos and read from them quickly using a 3.3V signal.
 - Uses the second serial port on the Due and Mega and leave the first one alone for normal serial operations. 
 - Have a separate power jack to power servos, but it should also have the ability to control whether 
   power is applied to the servos from the board or not. This will make it possible to use servo hubs with distributed power. 
 - Be able to power the Arduino board from the power connector on the shield board. This means you only need one
   connection from the battery. A jumper allows the user to configure this.
 - Have as many standard servo connectors as possible that are level shifted to 5V. 
 - Have all of the analog signals broken out into standard 3-pin connectors with power and ground connections. 
 - Have as many standardized Grove connectors as I could fit on it, and ensure that they are all 5V level shifted. 

## Installation

You can install the libraries manually by copying the directories from the libraries folder into the libraries directory 
for Arduino on your computer. For windows this is in Documents/arduino/libraries. You can also follow the instructions
laid out on the arduino website for installing libraries: https://www.arduino.cc/en/guide/libraries. Use the install
from zip file option and load in the zip file for the library you are interested in that is located in the libraries
folder of this repo. 