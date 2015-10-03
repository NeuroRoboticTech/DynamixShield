#include <DynamixelSerial.h>

#define SERVO_ID1 1

DynamixelSerial Dynamixel;

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

void MoveToPos(int servo, int targetPos, int targetSpeed)
{  
  delay(100);

  Dynamixel.moveSpeed (servo, targetPos, targetSpeed);

  int prevPos = -1;
  int pos = Dynamixel.readPosition(servo);
  int notMoved=0;
  //Wait until it has reached the new position, or close to it.
  while(abs(pos-targetPos) > 3)
  {    
    // Read the current servo Position 
    prevPos = pos;
    pos = Dynamixel.readPosition(servo);       

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
    Serial.print("Pos: ");
    Serial.println(pos);

    delay(100);
  }
}

void loop () { 
  Serial.println("Move to 450");
  MoveToPos(SERVO_ID1, 450, 30);
  
  Serial.println("Move to 650");
  MoveToPos(SERVO_ID1, 650, 150);
} 

