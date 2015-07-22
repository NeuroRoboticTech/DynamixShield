#include <DynamixelSerial.h>

#define SERVO_ID1 1
#define SERVO_ID2 3

DynamixelSerial Dynamixel(&Serial1);

void setup () {
 
  //#define SYS_BOARD_PLLAR (CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(39UL) | CKGR_PLLAR_PLLACOUNT(100UL) | CKGR_PLLAR_DIVA(3UL))
  //#define SYS_BOARD_MCKR ( PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK)
          
  // Set FWS according to SYS_BOARD_MCKR configuration 
  //EFC0->EEFC_FMR = EEFC_FMR_FWS(4); //4 waitstate flash access
  //EFC1->EEFC_FMR = EEFC_FMR_FWS(4);
  
  // Initialize PLLA to 114MHz 
  //PMC->CKGR_PLLAR = SYS_BOARD_PLLAR;
  //while (!(PMC->PMC_SR & PMC_SR_LOCKA)) {}
  
  //PMC->PMC_MCKR = SYS_BOARD_MCKR;
  //while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) {} 
 
  
  Serial.begin(57600);
  
  while(!Serial);
  Serial.println("Starting setup");
  //delay (5000); 
  
  pinMode(22, OUTPUT);
  Dynamixel.begin (1000000, 22); 

  //Dynamixel.setID(30, SERVO_ID);
  
  //delay(1000);
  //Dynamixel.reset(1);

  //delay(1000);
  //int iError = Dynamixel.setReturnDelayTime(1, 1);
  //Serial.println(iError);
  
  //delay (100); 
  //int ret_delay = Dynamixel.readReturnDelayTime(1);
  //Serial.print("Ret_Delay: ");
  //Serial.println(ret_delay);
  
  delay (100); 
  Dynamixel.move (SERVO_ID1,512);
  delay (100); 
  Dynamixel.move (SERVO_ID2,512);
  delay(1000);
  
  //ref_pos = Dynamixel.readPosition(1);
  //delay (100); 
} 

void ReadPosWithError(int servo, int ref_pos)
{  
  bool done = false;
  int errors = 0;
  int pos = 0, i = 0;
  double avg_time = 0;
  int loops = 10;
  int pos_vals[100];
  int pos_errors = 0;
  unsigned long end_time = 0, diff = 0, start_time = micros();
  
  for(i=0; i<loops; i++)
  {
    pos = Dynamixel.readPosition(servo);       // Request and Print the Position 
    //delayMicroseconds(200);
    delay(50);
    
    if(pos < 0)
      errors++;   
   
    if(pos != ref_pos)
    {
       pos_vals[pos_errors] = pos;
       pos_errors++;   
    }
  }
  
  end_time = micros();
  diff = end_time - start_time;  
  avg_time = (double) diff/ (double) loops;
  
  Serial.print("diff: ");
  Serial.println(diff);
  Serial.print("avg: ");
  Serial.println(avg_time);
  Serial.print("errors: ");
  Serial.println(errors);

  Serial.print("pos_errors: ");
  Serial.println(pos_errors);

  for(i=0; i<pos_errors; i++)
  {
    Serial.println(pos_vals[i]);
  }
}

/*
void ReadPos(int servo, int target)
{  
  bool done = false;
  int iError = 
  //while(!done)
  for(int i=0; i<100; i++)
  {
    int pos = Dynamixel.readPosition(1);       // Request and Print the Position 
    //Serial.print("Position: ");
    //Serial.print(pos);
    //Serial.println(" of 1023 resolution");
    delay(20);
    
    if(pos != Position)
      
    
    //if(abs(pos-target) <= 4)
    //  done = true;      
  }
}
*/

void loop () { 
 // digitalWrite(22, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println("Move 450");
  Dynamixel.moveSpeed (SERVO_ID1, 450, 30);  //.move(1, 450);, 30
  delay(100);
  Dynamixel.moveSpeed(SERVO_ID2, 650, 30);  //.move(1, 450);

  //delay(2000);
  //int Position = Dynamixel.readPosition(1);       // Request and Print the Position 
  //Serial.print("Position: ");
  //Serial.println(Position);

  //ReadPosWithError(SERVO_ID1, 450);
  
  delay (2000); 
  //digitalWrite(22, LOW);   // turn the LED on (HIGH is the voltage level)
  Serial.println("Move 650");
  Dynamixel.moveSpeed(SERVO_ID1, 650, 30); 
  delay(100);
  Dynamixel.moveSpeed(SERVO_ID2, 450, 30); 
   //Dynamixel.move(1, 650);
   
  delay(2000);
  //Position = Dynamixel.readPosition(1);       // Request and Print the Position 
  //Serial.print("Position: ");
  //Serial.println(Position);

  //delay (2000); 
} 

