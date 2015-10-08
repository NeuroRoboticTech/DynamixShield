#include <DynamixelSerial.h>

#define SERVO_ID1 1

DynamixelSerial Dynamixel; // or Dynamixel(&Serial1);

void setup () {
  Serial.begin(57600);
  
  while(!Serial);
  Serial.println("Starting setup");
  Dynamixel.begin(); 

  delay(1000);
  Serial.println("Resetting positions");
  Dynamixel.move2 (SERVO_ID1,512);
  delay (100); 
} 

void loop () { 
  Serial.println("Move to 0");
  Dynamixel.move2(SERVO_ID1, 0);
  delay(1000);
  
  Serial.println("Move to 1023");
  Dynamixel.move2(SERVO_ID1, 1023);
  delay(1000);
} 

