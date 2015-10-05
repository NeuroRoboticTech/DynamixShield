#include <CommanderHS.h>

CommanderHS command = CommanderHS(&Serial3);

void setup(){
  // setup serial
  Serial.begin(57600);
  while(!Serial);
  Serial.println("Starting setup");

  //Start the XBee commander controller.
  command.begin(38400);

  Serial.println ("Finished setup");
}


void loop(){
  // take commands 
  if(command.ReadMsgs() > 0){

    Serial.print("WalkV: ");
    Serial.print(command.walkV);
    Serial.print(", WalkH: ");
    Serial.print(command.walkH);

    Serial.print("LookV: ");
    Serial.print(command.lookV);
    Serial.print(", LookH: ");
    Serial.print(command.lookH);
    
    // select gaits
    if(command.buttons&BUT_R1){ 
      Serial.print(", R1");
    }
    if(command.buttons&BUT_R2){ 
      Serial.print(", R2");
    }
    if(command.buttons&BUT_R3){ 
      Serial.print(", R3");
    }
    if(command.buttons&BUT_L4){ 
      Serial.print(", L4");
    }
    if(command.buttons&BUT_L5){ 
      Serial.print(", L5");
    }
    if(command.buttons&BUT_L6){ 
      Serial.print(", L6");
    }

    Serial.println("");
 }
}
