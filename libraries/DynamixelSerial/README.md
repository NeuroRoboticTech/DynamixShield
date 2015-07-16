# DynamixSerial
This library is based on the one available from Savage Electronics: 
http://savageelectronics.blogspot.com/2011/01/arduino-y-dynamixel-ax-12.html
I have modified it so you can pass in any software serial connection that you want instead
of having multiple files for each serial connection. I have also updated it so it will work 
on Arduino Due, Zero, and Mega boards, and added a bunch of new control methods to it as well.


## Installation

You can install the libraries manually by copying the directories from the libraries folder into the libraries directory 
for Arduino on your computer. For windows this is in Documents/arduino/libraries. You can also follow the instructions
laid out on the arduino website for installing libraries: https://www.arduino.cc/en/guide/libraries. Use the install
from zip file option and load in the zip file for the library you are interested in that is located in the libraries
folder of this repo. 

## Usage

If you plan to use this on an SAM processor board like the Zero or Due then you must call the method SetSystemCoreClockFor1Mbaud
first thing in the setup section. The reason is that the Due and Zero run 88 and 48 Mhz, and neither of these can generate a true
1 Mbaud serial signal at that speed. So that call changes the clock speed to 80 and 40 Mhz instead. This allows the board to 
generate the 1 Mhz signals that are required to successfully talk to the Dynamixel servos. You also need to instantiate a 
DynamixSerial object and pass in the hardware serial link you want it to use for communications. Finally, call the begin method
and specify the pin number of the digital line you will use to control the half-duplex communications with the servos. For the
DynamixShield Due this is pin 22. For the DynamixShield Mega this is pin 30. However, if you use the DYN_CONTROL_PIN constant 
you will not need to worry about this. It will choose the right pin based on the board you are compiling against. After that
all you need to do is make calls to send and receive motor commands. Have fun!   