/* ===============================================================================
      Project: Grove Serial MP3 Player overview
       Author: Scott C
      Created: 9th March 2015
  Arduino IDE: 1.6.0
      Website: http://arduinobasics.blogspot.com/p/arduino-basics-projects-page.html

  Description: The following Arduino sketch will allow you to control a Grove Serial MP3 player
               with a Grove Sliding Potentiometer (volume), a Grove button (next song), 
               and a Grove Switch (on/off). It will also show you how to retrieve some useful information from the player. 
               Some functions are not used in this sketch,but have been included for your benefit. 
               
               Additional features and functionality can be found on the WT5001 voice chip datasheet 
               which I retrieved from here: http://goo.gl/ai6oQ9
               
               The Seeedstudio wiki was a very useful resource for getting started with the various Grove modules:
               http://goo.gl/xOiSCl
=============================================================================== */

//#include <SoftwareSerial.h>
//SoftwareSerial mp3(2, 3);      // The Grove MP3 Player is connected to Arduino digital Pin 2 and 3 (Serial communication)
//SoftwareSerial mp3(2, 3);        // The Grove MP3 Player is connected to Arduino digital Pin 2 and 3 (Serial communication)
HardwareSerial &mp3 = Serial3;

int potPin = A11;               // The Sliding Potentiometer is connected to AnalogPin 0
int potVal = 0;                // This is used to hold the value of the Sliding Potentiometer
byte mp3Vol = 0;               // mp3Vol is used to calculate the Current volume of the Grove MP3 player
byte oldVol = 0;               // oldVol is used to remember the previous volume level
int ledPin = A0;               // The Grove sliding potentiometer has an onboard LED attached to Analog pin 1.

int switchPin = 12;             // The Grove Switch(P) is connected to digital Pin 12
int switchStatus = 0;           // This is used to hold the status of the switch 
int switchChangeStatus = 0;     // Used to identify when the switch status has changed

int buttonPin = 6;              // The Grove Button is connected to digital pin 5
int buttonStatus = 0;           // This is used to hold the status of the button  



void setup(){
  Serial.begin(57600);
  Serial.println("setup begin");

  //Initialise the Grove MP3 Module
  delay(2500);
  mp3.begin(9600);
            
  // initialize the pushbutton and switch pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(switchPin, INPUT);
  
  // set ledPin on the sliding potentiometer to OUTPUT
  pinMode(ledPin, OUTPUT);
   
  setPlayMode(0x00);
  playSong(00,01);
  
  //You can view the following demostration output in the Serial Monitor
  demonstrate_GET_FUNCTIONS();    
  Serial.println("Finished Setup");
}


void loop(){
  //switchStatus = digitalRead(switchPin);
  //if(switchStatus==HIGH){
//    if(switchChangeStatus==LOW){             // When Arduino detects a change in the switchStatus (from LOW to HIGH) - play song      
//      setPlayMode(0x02);                     // Automatically cycle to the next song when the current song ends
//      playSong(00,01);                       // Play the 1st song when you switch it on
//      switchChangeStatus=HIGH;
//    }
    
    potVal = analogRead(potPin);                      // Analog read values from the sliding potentiometer range from 0 to 1023
    analogWrite(ledPin, potVal/4);                    // Analog write values range from 0 to 255, and will turn LED ON once potentiometer reaches about half way (or more).
    mp3Vol = map(potVal, 0, 1023, 0,31);              // Convert the potentometer reading (0 - 1023) to fit within the MP3 player's Volume range (0 - 31)
    if((mp3Vol>(oldVol+1))|(mp3Vol<(oldVol-1))){      // Only make a change to the Volume on the Grove MP3 player when the potentiometer value changes
      oldVol = mp3Vol;
      setVolume(mp3Vol);
      delay(10);                               // This delay is necessary with Serial communication to MP3 player
    }

    buttonStatus = digitalRead(buttonPin);
    if(buttonStatus==HIGH){                    // When a button press is detected - play the next song
      playNextSong();
      delay(200);                              // This delay aims to prevent a "skipped" song due to slow button presses - can modify to suit.
    }
  //} else {
  //  if(switchChangeStatus==HIGH){              // When switchStatus changes from HIGH to LOW - stop Song.
  //    stopSong();
  //    switchChangeStatus=LOW;
  //  }
  //} 
}


// demonstrate_GET_FUNCTIONS  will show you how to retrieve some useful information from the Grove MP3 Player (using the Serial Monitor).
void demonstrate_GET_FUNCTIONS(){
        Serial.print("Volume: ");
        Serial.println(getVolume());
        Serial.print("Playing State: ");
        Serial.println(getPlayingState());
        Serial.print("# of Files in SD Card:");
        Serial.println(getNumberOfFiles());
        Serial.println("------------------------------");
}


// writeToMP3: is a generic function that aims to simplify all of the methods that control the Grove MP3 Player

void writeToMP3(byte MsgLEN, byte A, byte B, byte C, byte D, byte E, byte F){
  byte codeMsg[] = {MsgLEN, A,B,C,D,E,F};
  mp3.write(0x7E);                        //Start Code for every command = 0x7E
  for(byte i = 0; i<MsgLEN+1; i++){
    mp3.write(codeMsg[i]);                //Send the rest of the command to the GROVE MP3 player
  }
}


/* The Following functions control the Grove MP3 Player : see datasheet for additional functions--------------------------------------------*/

void setPlayMode(byte playMode){
  /* playMode options:
        0x00 = Single song - played only once ie. not repeated.  (default)
        0x01 = Single song - cycled ie. repeats over and over.
        0x02 = All songs - cycled 
        0x03 = play songs randomly                                           */
        
  writeToMP3(0x03, 0xA9, playMode, 0x7E, 0x00, 0x00, 0x00);  
}


void playSong(byte songHbyte, byte songLbyte){                              // Plays the selected song
  writeToMP3(0x04, 0xA0, songHbyte, songLbyte, 0x7E, 0x00, 0x00);            
  Serial.println("Playing song.");
}


void pauseSong(){                                                           // Pauses the current song
  writeToMP3(0x02, 0xA3, 0x7E, 0x00, 0x00, 0x00, 0x00);
  Serial.println("Pausing song.");
}


void stopSong(){                                                            // Stops the current song
  writeToMP3(0x02, 0xA4, 0x7E, 0x00, 0x00, 0x00, 0x00);
  Serial.println("Stopping song.");
}


void playNextSong(){                                                        // Play the next song
  writeToMP3(0x02, 0xA5, 0x7E, 0x00, 0x00, 0x00, 0x00);
  Serial.println("Playing next song.");
}


void playPreviousSong(){                                                    // Play the previous song
  writeToMP3(0x02, 0xA6, 0x7E, 0x00, 0x00, 0x00, 0x00);
  Serial.println("Playing previous song.");
}


void addSongToPlayList(byte songHbyte, byte songLbyte){
  //Repeat this function for every song you wish to stack onto the playlist (max = 10 songs)
  writeToMP3(0x04, 0xA8, songHbyte, songLbyte, 0x7E, 0x00, 0x00);
}


void setVolume(byte Volume){                                                // Set the volume
  byte tempVol = constrain(Volume, 0, 31);
  //Volume range = 00 (muted) to 31 (max volume)
  writeToMP3(0x03, 0xA7, tempVol, 0x7E, 0x00, 0x00, 0x00); 
  Serial.print("Setting volume: ");
  Serial.println(Volume);
}



/* The following functions retrieve information from the Grove MP3 player : see data sheet for additional functions--------------*/

// getData: is a generic function to simplifly the other functions for retieving information from the Grove Serial MP3 player
byte getData(byte queryVal, int dataPosition){
  byte returnVal = 0x00;
  writeToMP3(0x02, queryVal, 0x7E, 0x00, 0x00, 0x00, 0x00);
  delay(50);
  for(int x = 0; x<dataPosition; x++){
    if(mp3.available()){
      returnVal = mp3.read();
      delay(50);
    }
  }
  return(returnVal);
}

byte getVolume(){                                              //Get the volume of the Grove Serial MP3 player
  //returns value from 0 - 31
  return(getData(0xC1, 4));
}

byte getPlayingState(){                                        //Get the playing state : Play / Stopped / Paused
  //returns 1: Play,   2: Stop,   3:Paused
  return(getData(0xC2, 2));
}


byte getNumberOfFiles(){                                      //Find out how many songs are on the SD card
  //returns the number of MP3 files on SD card
  return(getData(0xC4, 3));
}
