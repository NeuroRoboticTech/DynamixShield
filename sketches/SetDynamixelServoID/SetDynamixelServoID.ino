#include <DynamixelSerial.h>

int OLD_SERVO_ID = 1;
int NEW_SERVO_ID = 2;

DynamixelSerial Dynamixel;

void setup () {
  Serial.begin(57600);
  
  while(!Serial);
  Serial.println("Starting setup");
    
  Dynamixel.begin (); 

  Serial.println("Press any key and click send to reassign servo");
  while(!Serial.available()) {};

  //Call this method if you want to reset the
  //servo back to its factory default settings.
  //If you do then it will be reset to ID 1
  //Serial.println("Resetting servo to factory defaults");
  //Dynamixel.reset(OLD_SERVO_ID);
  //OLD_SERVO_ID = 1;

  //Now set the servo to the new ID
  Serial.print("Setting servo with ID '");
  Serial.print(OLD_SERVO_ID);
  Serial.print("' to have ID '");
  Serial.print(NEW_SERVO_ID);
  Serial.println("'");
  Dynamixel.setID(OLD_SERVO_ID, NEW_SERVO_ID);
  
  delay(500);

  //Now lets move it a few times to make sure it worked.
  Serial.println("Testing servo movement");
  Dynamixel.move (NEW_SERVO_ID, 250);
  delay(3000);
  Dynamixel.move (NEW_SERVO_ID, 850);
  delay(3000);

  //Then reset it back to its centered position
  Serial.println("Resetting positions");
  Dynamixel.move (NEW_SERVO_ID, 512);

  Serial.println("Servo reassignment finished.");
} 

void loop () { 
  delay (2000); 
} 

