#include <Servo.h>   //include the servo library to control the RobotGeek Servos
#include "CommanderHS.h"
#include <DynamixelSerial.h>
  
//Servo 1: Left, rear wheel
#define LEFT_REAR_WHEEL_ID 1
//Servo 2: Left, front wheel
#define LEFT_FRONT_WHEEL_ID 2
//Servo 3: Right, front wheel
#define RIGHT_FRONT_WHEEL_ID 3
//Servo 4: Right, rear wheel
#define RIGHT_REAR_WHEEL_ID 4
//Servo 5: Claw wrist
#define CLAW_WRIST_ID 5

//Standard Servo Left claw
#define GRIPPER_SERVO_PIN 2

#define GRIPPER_MIN 0
#define GRIPPER_MAX 150

Servo gripperServo;   //create an servo object for the 9g FT-FS90MG micro servo

#define WRIST_POS_MIN 312
#define WRIST_POS_MAX 612

#define LEFT_CLAW_POS_MIN 412
#define LEFT_CLAW_POS_MAX 712

#define RIGHT_CLAW_POS_MIN 312
#define RIGHT_CLAW_POS_MAX 612

//#define ENABLE_DEBUG 1

int wristPos = 512;
int leftClawPos = 512;
int rightClawPos = 512;

int gripperPos = 150;  //Start at 150 degrees
int gripperAdd = 0;  //Start at 0 degrees add

int currentLeftSpeed = 0;
int currentRightSpeed = 0;

float sign(float value) {
  if(value >= 0) return 1;
  else return -1; 
}

CommanderHS command = CommanderHS(&Serial3);
  
DynamixelSerial Dynamixel(&Serial1);

//float toRadians(float degrees) {return ((degrees * 71) / 4068));

//float toDegrees(float radians) {return ((radians * 4068) / 71)};

void setLeftWheels(int speed) {
  if(speed != currentLeftSpeed) {
    currentLeftSpeed = speed;
  
    if(speed > 0) {
      if(speed > 1023)
        speed = 1023;
  
      Dynamixel.turn(LEFT_REAR_WHEEL_ID, 0, speed);
      delay(10);
      Dynamixel.turn(LEFT_FRONT_WHEEL_ID, 0, speed);
      delay(10);  
    }
    else {
      if(speed < -1023)
        speed = -1023;
  
      Dynamixel.turn(LEFT_REAR_WHEEL_ID, 1, -speed);
      delay(10);
      Dynamixel.turn(LEFT_FRONT_WHEEL_ID, 1, -speed);
      delay(10);  
    }
  }
}


void setRightWheels(int speed) {
  if(speed != currentRightSpeed) {
    currentRightSpeed = speed;
     
    if(speed > 0) {
      if(speed > 1023)
        speed = 1023;
        
      Dynamixel.turn(RIGHT_REAR_WHEEL_ID, 1, speed);
      delay(10);
      Dynamixel.turn(RIGHT_FRONT_WHEEL_ID, 1, speed);
      delay(10);  
    }
    else {
      if(speed < -1023)
        speed = -1023;
  
      Dynamixel.turn(RIGHT_REAR_WHEEL_ID, 0, -speed);
      delay(10);
      Dynamixel.turn(RIGHT_FRONT_WHEEL_ID, 0, -speed);
      delay(10);  
    }
  }
}
 
void processWheels() {
  //First lets find the total length of the walkV vector
  //This will control the overall speed
  int speed = sqrt( (command.walkV*command.walkV) + 
                    (command.walkH*command.walkH) );
  float speedNorm = (float) speed / (float) 144.0;
  
  int leftSpeed = 0, rightSpeed = 0;

  //The angle of vertical to horizontal will control how much turn there is
  if(speed > 0) {
    float ratio = (float) (command.walkV)/ speed;
    float leftRatio = 0, rightRatio = 0;
    
    if(command.walkH > 0) {
      leftRatio = sign(ratio) * speedNorm;
      rightRatio = ratio * speedNorm;
    }
    else {
      rightRatio = sign(ratio) * speedNorm;
      leftRatio = ratio * speedNorm;
    }

    //The values given back from the arbotix commander are not circular
    //They are more rectangular. So if you normalize it then at the max
    //forward and reverse settings it is only at about 70% strength. This
    //multiplier helps get max speed when going forward or back.
    float multiplier = 1;
    if( ((ratio >= 0.90) && (ratio <= 1.0)) ||
        ((ratio <= -0.90) && (ratio >= -1.0)) )
        multiplier = 1.4141f;
        
    leftSpeed = 1023 * leftRatio * multiplier;
    rightSpeed = 1023 * rightRatio * multiplier;
    
#ifdef ENABLE_DEBUG
      //Serial.print("ratio: "); Serial.print(ratio); 
      //Serial.print(", left ratio: "); Serial.print(leftRatio); 
      //Serial.print(", right ratio: "); Serial.print(rightRatio); 
      //Serial.print(", speed norm: "); Serial.print(speedNorm); 
      //Serial.print(", left speed: "); Serial.print(leftSpeed); 
      //Serial.print(", right speed: "); Serial.println(rightSpeed); 
#endif    
  }

  setLeftWheels(leftSpeed);
  setRightWheels(rightSpeed);
}

void processWrist() {
  int wristAdd = map(command.lookV, -102, 102, -10, 10);
  
  if( (wristPos+wristAdd >= WRIST_POS_MIN) && (wristPos+wristAdd <= WRIST_POS_MAX) )
    wristPos += wristAdd;

#ifdef ENABLE_DEBUG
  Serial.print("Wrist Pos: "); Serial.println(wristPos); 
#endif

  if(wristAdd != 0) {
    Dynamixel.moveSpeed(CLAW_WRIST_ID, wristPos, 700);
    delay(10);
  }
}

void processGripper() {

  int gripperAdd = map(command.lookH, -102, 102, -10, 10);
  
  if(gripperAdd != 0) {
    gripperPos += gripperAdd;
    if(gripperPos > GRIPPER_MAX) {
      gripperPos = GRIPPER_MAX;
    }
    else if(gripperPos < GRIPPER_MIN) {
      gripperPos = GRIPPER_MIN;
    }
    gripperServo.write(gripperPos);
    
    Serial.print("Grip Add: "); Serial.print(gripperAdd);
    Serial.print("  Grip Pos: "); Serial.println(gripperPos);    
  }
}

bool processFastTurns() {
  if(command.buttons&BUT_R1 || 
     command.buttons&BUT_RT) {
    setLeftWheels(1023);
    setRightWheels(-1023);
    return true;
  }
  else if(command.buttons&BUT_L6 ||
          command.buttons&BUT_LT) {
    setLeftWheels(-1023);
    setRightWheels(1023);
    return true;
  }
  else if(command.buttons&BUT_R2) {
    setLeftWheels(512);
    setRightWheels(-512);
    return true;
  }
  else if(command.buttons&BUT_L5) {
    setLeftWheels(-512);
    setRightWheels(512);
    return true;
  }
  else if(command.buttons&BUT_R3) {
    setLeftWheels(256);
    setRightWheels(-256);
    return true;
  }
  else if(command.buttons&BUT_L4) {
    setLeftWheels(-256);
    setRightWheels(256);
    return true;
  }
        
  return false;
}
void checkCommander() {
  
  if(command.ReadMsgs() > 0) {
      //Serial.print("Commander has messages: ");
      //Serial.println(command.ReadMsgs());

    //If we are turning fast then use it to control
    //the wheels. Otherwise use regular joystick.
    if(!processFastTurns())
      processWheels();
  
    processWrist();
    processGripper();

    //If the commander data has changed then fill out the
    //custom sysex byte array and send it.    
#ifdef ENABLE_DEBUG
     Serial.print("Commander ");
     Serial.print(", WalkV: "); Serial.print(command.walkV); 
     Serial.print(", WalkH: "); Serial.print(command.walkH); 
     Serial.print(", LookV: "); Serial.print(command.lookV); 
     Serial.print(", LookH: "); Serial.print(command.lookH); 
     Serial.print(", Buttons: "); Serial.print(command.buttons); 
     Serial.println ("");
#endif
  }  
}

void configureServos() {

  Dynamixel.setStatusReturnLevel(LEFT_REAR_WHEEL_ID, 1);
  delay(50);
  Dynamixel.setStatusReturnLevel(LEFT_FRONT_WHEEL_ID, 1);
  delay(50);
  Dynamixel.setStatusReturnLevel(RIGHT_REAR_WHEEL_ID, 1);
  delay(50);
  Dynamixel.setStatusReturnLevel(RIGHT_FRONT_WHEEL_ID, 1);
  delay(50);
  Dynamixel.setStatusReturnLevel(CLAW_WRIST_ID, 1);
  delay(50);
  
  //Set the wheels to be in wheel mode
  Dynamixel.setEndless(LEFT_REAR_WHEEL_ID, true);
  delay(50);
  Dynamixel.setEndless(LEFT_FRONT_WHEEL_ID, true);
  delay(50);
  Dynamixel.setEndless(RIGHT_REAR_WHEEL_ID, true);
  delay(50);
  Dynamixel.setEndless(RIGHT_FRONT_WHEEL_ID, true);
  delay(50);
  
  Dynamixel.setCWLimit(CLAW_WRIST_ID, WRIST_POS_MIN);
  delay(50);
  Dynamixel.setCCWLimit(CLAW_WRIST_ID, WRIST_POS_MAX);
  delay(50);
  
  Dynamixel.turn(LEFT_REAR_WHEEL_ID, 1, 0);
  delay(50);
  Dynamixel.turn(LEFT_FRONT_WHEEL_ID, 1, 0);
  delay(50);
  Dynamixel.turn(RIGHT_REAR_WHEEL_ID, 1, 0);
  delay(50);
  Dynamixel.turn(RIGHT_FRONT_WHEEL_ID, 1, 0);
  delay(50);
  
  Dynamixel.moveSpeed(CLAW_WRIST_ID, 512, 0);
  delay(50);
  
  //attach and set gripper and wrist servos  
  gripperServo.attach(GRIPPER_SERVO_PIN);
  gripperServo.write(gripperPos);    // sets the servo position to 150 degress, positioning the servo for the gripper
} 

void configureCommanderOffsets() {
  command.walkHOffset = -1;
  command.lookVOffset = 2;
}
  
void setup() {
  SetSystemCoreClockFor1Mbaud();

#ifdef ENABLE_DEBUG
  Serial.begin(57600);
  while(!Serial);
  Serial.println("Starting setup");
#endif

  command.begin(38400);
  
  Dynamixel.begin (1000000, 22); 
  
  configureServos();  
  configureCommanderOffsets();
}

void loop() {
  checkCommander();
  delay(10);                
}



