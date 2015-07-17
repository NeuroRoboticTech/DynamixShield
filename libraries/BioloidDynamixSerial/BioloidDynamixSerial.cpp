/*
  BioloidDynamixSerial - DynamixShield Library for Bioloid Pose Engine
  Copyright (c) 2015 David Cofer.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "BioloidDynamixSerial.h"

/* initializes serial1 transmit at baud, 8-N-1 */
BioloidDynamixSerial::BioloidDynamixSerial(DynamixelSerial *dynamix){
    int i;
    // setup storage
    id_ = (unsigned char *) malloc(AX12_MAX_SERVOS * sizeof(unsigned char));
    pose_ = (unsigned int *) malloc(AX12_MAX_SERVOS * sizeof(unsigned int));
    nextpose_ = (unsigned int *) malloc(AX12_MAX_SERVOS * sizeof(unsigned int));
    speed_ = (int *) malloc(AX12_MAX_SERVOS * sizeof(int));
    // initialize
    for(i=0;i<AX12_MAX_SERVOS;i++){
        id_[i] = i+1;
        pose_[i] = 512;
        nextpose_[i] = 512;
    }
    interpolating = 0;
    playing = 0;
    lastframe_ = millis();
	
	controller = dynamix;
}

/* new-style setup */
void BioloidDynamixSerial::setup(int servo_cnt){
    int i;
    // setup storage
    id_ = (unsigned char *) malloc(servo_cnt * sizeof(unsigned char));
    pose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
    nextpose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
    speed_ = (int *) malloc(servo_cnt * sizeof(int));
    // initialize
    poseSize = servo_cnt;
    for(i=0;i<poseSize;i++){
        id_[i] = i+1;
        pose_[i] = 512;
        nextpose_[i] = 512;
    }
    interpolating = 0;
    playing = 0;
    lastframe_ = millis();
}
void BioloidDynamixSerial::setId(int index, int id){
    id_[index] = id;
}
int BioloidDynamixSerial::getId(int index){
    return id_[index];
}

/* load a named pose from FLASH into nextpose. */
void BioloidDynamixSerial::loadPose( const unsigned int * addr ){
    int i;
    poseSize = pgm_read_word_near(addr); // number of servos in this pose
    for(i=0; i<poseSize; i++)
        nextpose_[i] = pgm_read_word_near(addr+1+i) << BIOLOID_SHIFT;
}
/* read in current servo positions to the pose. */
void BioloidDynamixSerial::readPose(){
    for(int i=0;i<poseSize;i++){
		if(controller) {
            int pos = controller->readRegister(id_[i],AX_PRESENT_POSITION_L,2);
			pose_[i] = pos<<BIOLOID_SHIFT;
            Serial.print("Read: "); Serial.print(id_[i]); Serial.print(" = "); Serial.println(pos);
        }
        delay(25);   
    }
}
/* write pose out to servos using sync write. */
void BioloidDynamixSerial::writePose(){
	
	if(controller) {
        //Serial.println("Start writePose");
        //Do not use speed in the synch settings
		controller->startSyncWrite(false);  
	
		for(int i=0; i<poseSize; i++) {
            int pos = pose_[i] >> BIOLOID_SHIFT;
            //Serial.print("servo: "); Serial.print(id_[i]);
            //Serial.print("pos: "); Serial.println(pos);
            
			controller->addServoToSync(id_[i], pos, 0);
        }
        
        controller->writeSyncData();
        //Serial.println("Finished writePose");
        //Serial.println("");
        //Serial.println("");
       // delay(1000);
	}
}

/* set up for an interpolation from pose to nextpose over TIME 
    milliseconds by setting servo speeds. */
void BioloidDynamixSerial::interpolateSetup(int time){
    int i;
    int frames = (time/BIOLOID_FRAME_LENGTH) + 1;
    lastframe_ = millis();
    // set speed each servo...
    for(i=0;i<poseSize;i++){
        if(nextpose_[i] > pose_[i]){
            speed_[i] = (nextpose_[i] - pose_[i])/frames + 1;
        }else{
            speed_[i] = (pose_[i]-nextpose_[i])/frames + 1;
        }
    }
    interpolating = 1;
}
/* interpolate our pose, this should be called at about 30Hz. */
void BioloidDynamixSerial::interpolateStep(){
    if(interpolating == 0) return;
    int i;
    int complete = poseSize;
    while(millis() - lastframe_ < BIOLOID_FRAME_LENGTH);
    lastframe_ = millis();
    // update each servo
    for(i=0;i<poseSize;i++){
        int diff = nextpose_[i] - pose_[i];
        if(diff == 0){
            complete--;
        }else{
            if(diff > 0){
                if(diff < speed_[i]){
                    pose_[i] = nextpose_[i];
                    complete--;
                }else
                    pose_[i] += speed_[i];
            }else{
                if((-diff) < speed_[i]){
                    pose_[i] = nextpose_[i];
                    complete--;
                }else
                    pose_[i] -= speed_[i];                
            }       
        }
    }
    if(complete <= 0) interpolating = 0;
    writePose();      
}

/* get a servo value in the current pose */
int BioloidDynamixSerial::getCurPose(int id){
    for(int i=0; i<poseSize; i++){
        if( id_[i] == id )
            return ((pose_[i]) >> BIOLOID_SHIFT);
    }
    return -1;
}
/* get a servo value in the next pose */
int BioloidDynamixSerial::getNextPose(int id){
    for(int i=0; i<poseSize; i++){
        if( id_[i] == id )
            return ((nextpose_[i]) >> BIOLOID_SHIFT);
    }
    return -1;
}
/* set a servo value in the next pose */
void BioloidDynamixSerial::setNextPose(int id, int pos){
    for(int i=0; i<poseSize; i++){
        if( id_[i] == id ){
            nextpose_[i] = (pos << BIOLOID_SHIFT);
            return;
        }
    }
}

/* play a sequence. */
void BioloidDynamixSerial::playSeq( const transition_t  * addr ){
    sequence = (transition_t *) addr;
    // number of transitions left to load
    transitions = pgm_read_word_near(&sequence->time);
    sequence++;    
    // load a transition
    loadPose((const unsigned int *)pgm_read_word_near(&sequence->pose));
    interpolateSetup(pgm_read_word_near(&sequence->time));
    transitions--;
    playing = 1;
}
/* keep playing our sequence */
void BioloidDynamixSerial::play(){
    if(playing == 0) return;
    if(interpolating > 0){
        interpolateStep();
    }else{  // move onto next pose
        sequence++;   
        if(transitions > 0){
            loadPose((const unsigned int *)pgm_read_word_near(&sequence->pose));
            interpolateSetup(pgm_read_word_near(&sequence->time));
            transitions--;
        }else{
            playing = 0;
        }
    }
}

