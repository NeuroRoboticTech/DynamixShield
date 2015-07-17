#include <DynamixelSerial.h>

DynamixelSerial Dynamixel(&Serial2);

int pose[3][4];
int poseIdx = 0;

void setup() {
  SetSystemCoreClockFor1Mbaud();

  Serial.begin(57600);
  while(!Serial);
  Serial.println("Starting setup");

  Dynamixel.begin (1000000, 22); 

  pose[0][0] = 450;
  pose[1][0] = 450;
  pose[2][0] = 450;

  pose[0][1] = 512;
  pose[1][1] = 512;
  pose[2][1] = 512;

  pose[0][2] = 650;
  pose[1][2] = 650;
  pose[2][2] = 650;

  pose[0][3] = 512;
  pose[1][3] = 512;
  pose[2][3] = 512;
/*
  writePose();
  delay(3000); 
  poseIdx++;

  writePose();
  delay(3000); 
  poseIdx++;

  writePose();
  delay(3000); 
  poseIdx++;


  writePose();
  delay(3000); 
  poseIdx = 0;

  writePose();
  delay(3000); 
  poseIdx++;

  writePose();
  delay(3000); 
  poseIdx++;

  writePose();
  delay(3000); 
  poseIdx = 0;
  */
}


void testPose() {
  Serial.println("Start synchronous write");

  Dynamixel.startSyncWrite();  
  
  Dynamixel.addServoToSync(0, 0x010, 0x150);
  Dynamixel.addServoToSync(1, 0x220, 0x360);
  Dynamixel.addServoToSync(2, 0x030, 0x170);
  Dynamixel.addServoToSync(3, 0x220, 0x380);
        
  Dynamixel.writeSyncData(true);
  Serial.println("Finish synchronous write");
}

void writePose() {
  Serial.print("Start synchronous write Pose: ");
  Serial.println(poseIdx);

  Dynamixel.startSyncWrite();  
  
  for(int i=0; i<3; i++)
    Dynamixel.addServoToSync(i+1, pose[i][poseIdx], 100);
        
  Dynamixel.writeSyncData(true);
  Serial.println("Finish synchronous write");
}

void loop() {
  writePose();
  
  poseIdx++;
  if(poseIdx == 4)
    poseIdx = 0;

  delay(3000);
}
