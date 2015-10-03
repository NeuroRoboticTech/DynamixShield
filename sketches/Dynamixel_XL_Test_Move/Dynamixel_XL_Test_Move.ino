#include <DynamixelSerial.h>

#define SERVO_ID1 1

DynamixelSerial Dynamixel;

void setup () {
  Serial.begin(57600);
  
  while(!Serial);
  Serial.println("Starting setup");
  //delay (5000); 
  
  Dynamixel.begin();

  delay(1000);
  
  delay (100); 
  Serial.println("Resetting positions");
  Dynamixel.move2 (SERVO_ID1,512);
  delay (100); 
} 

void loop () { 
  Serial.println("Move 450");
  Dynamixel.move2(SERVO_ID1, 450);
  
  delay (1000); 
  Serial.println("Move 650");
  Dynamixel.move2(SERVO_ID1, 650); 
   
  delay(1000);
} 

