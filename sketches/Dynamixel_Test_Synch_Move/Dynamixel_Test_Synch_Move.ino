#include <DynamixelSerial.h>

#define SERVO_ID1 1
#define SERVO_ID2 2

DynamixelSerial Dynamixel; // or Dynamixel(&Serial1);

void setup () {
  Serial.begin(57600);
  
  while(!Serial);
  Serial.println("Starting setup");
  Dynamixel.begin(); 

  delay(1000);
  Serial.println("Resetting positions");
  Dynamixel.move (SERVO_ID1,512);
  Dynamixel.move (SERVO_ID2,512);
  delay (100); 
} 

void loop () { 
  Serial.println("Move 1 to 100");
  Serial.println("Move 2 to 50");
  Dynamixel.startSyncWrite(true);
  Dynamixel.addServoToSync(SERVO_ID1, 100, 30);
  Dynamixel.addServoToSync(SERVO_ID2, 50, 50);
  Dynamixel.writeSyncData();
  
  delay(5000);

  Serial.println("Move 1 to 1023");
  Serial.println("Move 2 to 950");
  Dynamixel.startSyncWrite(true);
  Dynamixel.addServoToSync(SERVO_ID1, 1023, 600);
  Dynamixel.addServoToSync(SERVO_ID2, 950, 800);
  Dynamixel.writeSyncData();

  delay(1000);
} 

