/*
  Grove- i2C motor driver demo v1.0
  by: http://www.seeedstudio.com
*/
  //  Author:LG
#include "Wire.h"

#define MotorSpeedSet             0x82
#define PWMFrequenceSet           0x84
#define DirectionSet              0xaa
#define MotorSetA                 0xa1
#define MotorSetB                 0xa5
#define Nothing                   0x01
#define Stepernu                  0x1c
#define I2CMotorDriverAdd         0x0f   // Set the address of the I2CMotorDriver

int data=0x5;
byte high=0x00, low=0x01;

void MotorSpeedSetAB(unsigned char MotorSpeedA , unsigned char MotorSpeedB)  {
  MotorSpeedA=map(MotorSpeedA,0,100,0,255);
  MotorSpeedB=map(MotorSpeedB,0,100,0,255);
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(MotorSpeedSet);                 // set pwm header 
  Wire.write(MotorSpeedA);                   // send pwma 
  Wire.write(MotorSpeedB);                   // send pwmb    
  Wire.endTransmission();                    // stop transmitting
}

void MotorPWMFrequenceSet(unsigned char Frequence)  {    
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(PWMFrequenceSet);               // set frequence header
  Wire.write(Frequence);                     //  send frequence 
  Wire.write(Nothing);                       //  need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();                    // stop transmitting
}

void MotorDirectionSet(unsigned char Direction)  {     //  Adjust the direction of the motors 0b0000 I4 I3 I2 I1
  Wire.beginTransmission(I2CMotorDriverAdd);           // transmit to device I2CMotorDriverAdd
  Wire.write(DirectionSet);                            // Direction control header
  Wire.write(Direction);                               // send direction control information
  Wire.write(Nothing);                                 // need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();                              // stop transmitting 
}

void MotorDriectionAndSpeedSet(unsigned char Direction,unsigned char MotorSpeedA,unsigned char MotorSpeedB)  {  //you can adjust the direction and speed together
  MotorDirectionSet(Direction);
  MotorSpeedSetAB(MotorSpeedA,MotorSpeedB);  
}

void setup()  {
  Wire.begin(); // join i2c bus (address optional for master)
  delayMicroseconds(10000);
  Serial.begin(57600);
  Serial.println("setup begin");
}

void loop()  {
/*
  Wire.beginTransmission(0x44); // transmit to device #44 (0x2c)
                              // device address is specified in datasheet
  Wire.write(byte(0x00));            // sends instruction byte  
  Wire.write(0xAB);             // sends potentiometer value byte  
  Wire.endTransmission();     // stop transmitting
  delay(5); //this delay needed
*/
  
  Wire.beginTransmission(I2CMotorDriverAdd);
  Wire.write(MotorSpeedSet);                 // set pwm header 
  Wire.write(0x03);                   // send pwma 
  Wire.write(0x05);                   // send pwmb    
  Wire.endTransmission();
  delay(5);
    
/*
  MotorSpeedSetAB(255,255);//defines the speed of motor 1 and motor 2;
  delay(10); //this delay needed

  //MotorDirectionSet(0b1010);  //"0b1010" defines the output polarity, "10" means the M+ is "positive" while the M- is "negative"
  // make sure M+ and M- is different polarity when driving DC motors.
  delay(1000); 

  //MotorDirectionSet(0b0101);  //0b0101  Rotating in the opposite direction
  delay(500);
*/  
}
