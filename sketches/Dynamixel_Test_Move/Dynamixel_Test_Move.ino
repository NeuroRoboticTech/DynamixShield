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

void MoveToPos(int servo, int targetPos, int targetSpeed)
{  
  delay(100);

  Dynamixel.moveSpeed (servo, targetPos, targetSpeed);

  int prevPos = -1;
  int pos = Dynamixel.readPosition(servo);
  int vel = 0;
  int notMoved=0;
  //Wait until it has reached the new position, or close to it.
  while(abs(pos-targetPos) > 3)
  {    
    // Read the current servo Position 
    prevPos = pos;
    pos = Dynamixel.readPosition(servo);       
    vel = Dynamixel.readSpeed(servo);

    //If the position is not changing then it is not moving
    if(abs(prevPos-pos) == 0) {
      notMoved++;    
    }

    //If it is not moving then did we get an error in transmission?
    //Send the move command to be sure.
    if(notMoved == 5) {
      Dynamixel.moveSpeed (servo, targetPos, targetSpeed);
    }

    // Print the current servo Position 
    Serial.print("Servo: ");
    Serial.print(servo);
    Serial.print(", Pos: ");
    Serial.print(pos);
    Serial.print(",  Speed: ");
    Serial.println(vel);

    delay(100);
  }
}

void loop () { 
  Serial.println("Move 1 to 450");
  MoveToPos(SERVO_ID1, 450, 30);

  Serial.println("Move 2 to 100");
  MoveToPos(SERVO_ID2, 100, 50);

  Serial.println("Move 1 to 650");
  MoveToPos(SERVO_ID1, 650, 150);

  Serial.println("Move 2 to 850");
  MoveToPos(SERVO_ID2, 850, 80);
} 

