/*
 Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 Created by Savage on 27/01/11.
 Modified by Cofer on 7/5/15
 
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
 
 *****************************************************************************
 Modifications:
 
 25/07/2011 - Eliminado la modificacion serial para ser modificada dentro del mismo Hardware Serial.
 25/07/2011 - Modificado la funcion setBD() para aceptar todas la velocidades sin PDF.
 25/07/2011 - Agregada la funcion de Rotacion Continua.
 26/07/2011 - Agregada la funcion begin sin seteo de Direction_Pin.
 25/07/2011 - Agregada la funcion Reset.
 26/07/2011 - Agregada la funcion Reg_Write en move y moveSpeed.
 26/07/2011 - Agregada la funcion Action.
 13/12/2011 - Arreglado el manejo y envio de variables.
 22/12/2011 - Compatible con la actualizacion Arduino 1.0.
 10/01/2012 - Utilizacion de Macros y eliminacion codigo no necesario.
 11/01/2012 - Agregadas las funciones:
              int setTempLimit(unsigned char ID, unsigned char Temperature);
              int setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit);
              int setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage);
			  int setMaxTorque(unsigned char ID, int MaxTorque);
              int setSRL(unsigned char ID, unsigned char SRL);
              int setRDT(unsigned char ID, unsigned char RDT);
              int setLEDAlarm(unsigned char ID, unsigned char LEDAlarm);
              int setShutdownAlarm(unsigned char ID, unsigned char SALARM);
              int setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin);
			  int setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope);
 15/01/2012 - Agregadas las funciones:             
              int setPunch(unsigned char ID, int Punch);
              int moving(unsigned char ID);
              int lockRegister(unsigned char ID);
			  int RWStatus(unsigned char ID);
              int readSpeed(unsigned char ID);
              int readLoad(unsigned char ID);
 7/5/15     - Modified library to allow user to pass in serial link direclty instead
              of having multiple files for different serial lines
 TODO:
 
 FUNCION SYNCWRITE.
 
 *****************************************************************************
 
 Contact: savageelectronics@gmail.com 
 Web:     http://savageelectrtonics.blogspot.com/
 Autor:   Josue Alejandro Savage
 
 For Modifications
 Contact: dcofer@NeuroRoboticTech.com 
 Web:     http://NeuroRoboticTech.com/
 Co-Author:  David Cofer
 */

#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "DynamixelSerial.h"

// Macro for Timing

#define delayus(args) (delayMicroseconds(args))  // Delay Microseconds

// Macro for Comunication Flow Control

#define setDPin(DirPin,Mode)   (pinMode(DirPin,Mode))       // Select the Switch to TX/RX Mode Pin
#define switchCom(DirPin,Mode) (digitalWrite(DirPin,Mode))  // Switch to TX/RX Mode

DynamixelSerial::DynamixelSerial(HardwareSerial *ss){
	Checksum = 0; 
	Direction_Pin = 0;
	Time_Counter = 0;
	Incoming_Byte = 0;               
	Position_High_Byte = 0;
	Position_Low_Byte = 0;
	Speed_High_Byte = 0;
	Speed_Low_Byte = 0;
	Load_High_Byte = 0;
	Load_Low_Byte = 0;
	
	Moving_Byte = 0;
	RWS_Byte = 0;
	Speed_Long_Byte = 0;
	Load_Long_Byte = 0;
	Position_Long_Byte = 0;
	Temperature_Byte = 0;
	Voltage_Byte = 0;
	Error_Byte = 0; 
	Return_Delay_Byte = 0;
	stream = ss;
	use_speed_synch = true;
    Status_Return_Level = 2;
    
#ifdef __SAM3X8E__
    SetSystemCoreClockFor1Mbaud();
#endif        

    if(stream == NULL) {
        stream = &Serial1;
    }
}

void DynamixelSerial::sendData(const uint8_t data)
{
	if(stream)
	{
		stream->write(data);
		stream->flush();
	}
}

int DynamixelSerial::availableData()
{
	if(stream)
		return stream->available();
	else
		return 0;
}

int DynamixelSerial::readData()
{
	if(stream)
		return stream->read();
	else
		return 0;
}

int DynamixelSerial::peekData()
{
	if(stream)
		return stream->peek();
	else
		return 0;
}

void DynamixelSerial::flushData()
{
  if(stream) {
    stream->flush();    
  }
}

void DynamixelSerial::beginCom(unsigned long baudRate)
{
	if(stream)
		return stream->begin(baudRate);
}

void DynamixelSerial::endCom()
{
	if(stream)
		return stream->end();
}

// Private Methods //////////////////////////////////////////////////////////////

int DynamixelSerial::read_error(void)
{
	Time_Counter = 0;
	while((availableData() < 5) & (Time_Counter < TIME_OUT)){  // Wait for Data
		Time_Counter++;
		delayus(1000);
	}
	
	while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                                    // Start Bytes
			readData();                                    // Ax-12 ID
			readData();                                    // Length
			Error_Byte = readData();                       // Error
			//Serial.print("Received Error: ");
			//Serial.println(Error_Byte);
            
            if(Time_Counter == TIME_OUT)
                Serial.println("Timed out");
                
			return (Error_Byte);
		}
	}

	//Serial.print("Nothing Recieved: ");
	//Serial.println(-1);
	return (-1);											 // No Ax Response
}

// Public Methods //////////////////////////////////////////////////////////////

void DynamixelSerial::begin(long baud, unsigned char directionPin)
{	
	Direction_Pin = directionPin;
	setDPin(Direction_Pin,OUTPUT);
	beginCom(baud);
}	

void DynamixelSerial::begin(long baud)
{	
	beginCom(baud);
}	

void DynamixelSerial::end()
{
	endCom();
}

int DynamixelSerial::reset(unsigned char ID)
{
	Checksum = (~(ID + AX_RESET_LENGTH + AX_RESET))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	sendData(AX_START);                     
	sendData(AX_START);
	sendData(ID);
	sendData(AX_RESET_LENGTH);
	sendData(AX_RESET);    
	sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);

    return (read_error());  
}

int DynamixelSerial::ping(unsigned char ID)
{
	Checksum = (~(ID + AX_READ_DATA + AX_PING))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	sendData(AX_START);                     
	sendData(AX_START);
	sendData(ID);
	sendData(AX_READ_DATA);
	sendData(AX_PING);    
	sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());              
}

int DynamixelSerial::setID(unsigned char ID, unsigned char newID)
{    
	Checksum = (~(ID + AX_ID_LENGTH + AX_WRITE_DATA + AX_ID + newID))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
	sendData(AX_ID_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_ID);
    sendData(newID);
    sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelSerial::setBD(unsigned char ID, long baud)
{    
	unsigned char Baud_Rate = (2000000/baud) - 1;
    Checksum = (~(ID + AX_BD_LENGTH + AX_WRITE_DATA + AX_BAUD_RATE + Baud_Rate))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                 // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
	sendData(AX_BD_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_BAUD_RATE);
    sendData(Baud_Rate);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelSerial::move(unsigned char ID, int Position)
{
    char Position_H,Position_L;
    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;
	Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                 // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_GOAL_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_GOAL_POSITION_L);
    sendData(Position_L);
    sendData(Position_H);
    sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);

    if(Status_Return_Level == 2)
        return (read_error());                // Return the read error
    else
        return 0;
}

int DynamixelSerial::moveSpeed(unsigned char ID, int Position, int Speed)
{
    char Position_H,Position_L,Speed_H,Speed_L;
    Position_H = Position >> 8;    
    Position_L = Position;                // 16 bits - 2 x 8 bits variables
    Speed_H = Speed >> 8;
    Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables
	Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H))&0xFF;
 
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_GOAL_SP_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_GOAL_POSITION_L);
    sendData(Position_L);
    sendData(Position_H);
    sendData(Speed_L);
    sendData(Speed_H);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
 	switchCom(Direction_Pin,Rx_MODE);
    
    if(Status_Return_Level == 2)
        return (read_error());                // Return the read error
    else
        return 0;
}

int DynamixelSerial::stop(unsigned char ID)
{
  //First set speed to slowest setting.
  setSpeed(ID, 1);
  delay(10);
  
  //Now get the servo position
  int pos = readPosition(ID);
  
  //And set the goal position to that value.
  move(ID, pos);

  //Serial.print("Stop Pos: ");
  //Serial.println(pos);
}

int DynamixelSerial::setEndless(unsigned char ID, bool Status)
{
 if ( Status ) {	
	  char AX_CCW_AL_LT = 0;     // Changing the CCW Angle Limits for Full Rotation.
	  Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L))&0xFF;
	
	  switchCom(Direction_Pin,Tx_MODE);
      sendData(AX_START);                // Send Instructions over Serial
      sendData(AX_START);
      sendData(ID);
      sendData(AX_GOAL_LENGTH);
      sendData(AX_WRITE_DATA);
      sendData(AX_CCW_ANGLE_LIMIT_L );
      sendData(AX_CCW_AL_LT);
      sendData(AX_CCW_AL_LT);
      sendData(Checksum);
      delayus(TX_DELAY_TIME);
	  switchCom(Direction_Pin,Rx_MODE);

      if(Status_Return_Level == 2)
          return (read_error());                // Return the read error
      else
          return 0;
 }
 else
 {
	 turn(ID,0,0);
	 Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + AX_CCW_AL_L + AX_CCW_AL_H))&0xFF;
	
	 switchCom(Direction_Pin,Tx_MODE);
	 sendData(AX_START);                 // Send Instructions over Serial
	 sendData(AX_START);
	 sendData(ID);
	 sendData(AX_GOAL_LENGTH);
	 sendData(AX_WRITE_DATA);
	 sendData(AX_CCW_ANGLE_LIMIT_L);
	 sendData(AX_CCW_AL_L);
	 sendData(AX_CCW_AL_H);
	 sendData(Checksum);
	 delayus(TX_DELAY_TIME);
	 switchCom(Direction_Pin,Rx_MODE);
	 
      if(Status_Return_Level == 2)
          return (read_error());                // Return the read error
      else
          return 0;
  }
 } 

int DynamixelSerial::turn(unsigned char ID, bool SIDE, int Speed)
{		
		if (SIDE == 0){                          // Move Left///////////////////////////
			
			char Speed_H,Speed_L;
			Speed_H = Speed >> 8;
			Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables
			Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H))&0xFF;
			
			switchCom(Direction_Pin,Tx_MODE);
			sendData(AX_START);                // Send Instructions over Serial
			sendData(AX_START);
			sendData(ID);
			sendData(AX_SPEED_LENGTH);
			sendData(AX_WRITE_DATA);
			sendData(AX_GOAL_SPEED_L);
			sendData(Speed_L);
			sendData(Speed_H);
			sendData(Checksum);
			delayus(TX_DELAY_TIME);
			switchCom(Direction_Pin,Rx_MODE);
			
            if(Status_Return_Level == 2)
                return (read_error());                // Return the read error
            else
                return 0;
		}
		else
		{                                            // Move Rigth////////////////////
			char Speed_H,Speed_L;
			Speed_H = (Speed >> 8) + 4;
			Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables
			Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H))&0xFF;
			
			switchCom(Direction_Pin,Tx_MODE);
			sendData(AX_START);                // Send Instructions over Serial
			sendData(AX_START);
			sendData(ID);
			sendData(AX_SPEED_LENGTH);
			sendData(AX_WRITE_DATA);
			sendData(AX_GOAL_SPEED_L);
			sendData(Speed_L);
			sendData(Speed_H);
			sendData(Checksum);
			delayus(TX_DELAY_TIME);
			switchCom(Direction_Pin,Rx_MODE);
			
            if(Status_Return_Level == 2)
                return (read_error());                // Return the read error
            else
                return 0;
		}
}

int DynamixelSerial::moveRW(unsigned char ID, int Position)
{
    char Position_H,Position_L;
    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;
    Checksum = (~(ID + AX_GOAL_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                 // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_GOAL_LENGTH);
    sendData(AX_REG_WRITE);
    sendData(AX_GOAL_POSITION_L);
    sendData(Position_L);
    sendData(Position_H);
    sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    if(Status_Return_Level == 2)
        return (read_error());                // Return the read error
    else
        return 0;
}

int DynamixelSerial::moveSpeedRW(unsigned char ID, int Position, int Speed)
{
    char Position_H,Position_L,Speed_H,Speed_L;
    Position_H = Position >> 8;    
    Position_L = Position;                // 16 bits - 2 x 8 bits variables
    Speed_H = Speed >> 8;
    Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables
    Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_GOAL_SP_LENGTH);
    sendData(AX_REG_WRITE);
    sendData(AX_GOAL_POSITION_L);
    sendData(Position_L);
    sendData(Position_H);
    sendData(Speed_L);
    sendData(Speed_H);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    if(Status_Return_Level == 2)
        return (read_error());                // Return the read error
    else
        return 0;
}

void DynamixelSerial::action()
{	
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                // Send Instructions over Serial
    sendData(AX_START);
    sendData(BROADCAST_ID);
    sendData(AX_ACTION_LENGTH);
    sendData(AX_ACTION);
    sendData(AX_ACTION_CHECKSUM);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
}

int DynamixelSerial::torqueStatus( unsigned char ID, bool Status)
{
    Checksum = (~(ID + AX_TORQUE_LENGTH + AX_WRITE_DATA + AX_TORQUE_ENABLE + Status))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);              // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_TORQUE_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_TORQUE_ENABLE);
    sendData(Status);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());              // Return the read error
}

int DynamixelSerial::ledStatus(unsigned char ID, bool Status)
{    
    Checksum = (~(ID + AX_LED_LENGTH + AX_WRITE_DATA + AX_LED + Status))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);              // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_LED_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_LED);
    sendData(Status);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());              // Return the read error
}

int DynamixelSerial::readTemperature(unsigned char ID)
{	
    Checksum = (~(ID + AX_TEM_LENGTH  + AX_READ_DATA + AX_PRESENT_TEMPERATURE + AX_BYTE_READ))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(ID);
    sendData(AX_TEM_LENGTH);
    sendData(AX_READ_DATA);
    sendData(AX_PRESENT_TEMPERATURE);
    sendData(AX_BYTE_READ);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Temperature_Byte = -1;
    Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			Temperature_Byte = readData();         // Temperature
		}
    }
	return (Temperature_Byte);               // Returns the read temperature
}

int DynamixelSerial::readPosition(unsigned char ID)
{	
    Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS))&0xFF;
  
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(ID);
    sendData(AX_POS_LENGTH);
    sendData(AX_READ_DATA);
    sendData(AX_PRESENT_POSITION_L);
    sendData(AX_BYTE_READ_POS);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Position_Long_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 7) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
    
			Position_Low_Byte = readData();            // Position Bytes
			Position_High_Byte = readData();
			Position_Long_Byte = Position_High_Byte << 8; 
			Position_Long_Byte = Position_Long_Byte + Position_Low_Byte;
		}
    }
	return (Position_Long_Byte);     // Returns the read position
}

/** Read register value(s) */
int DynamixelSerial::readRegister(unsigned char id, int regstart, int length)
{	
    Checksum = (~(id + 6 + regstart + length))&0xFF;

  flushData();
  
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(id);
    sendData(4);
    sendData(AX_READ_DATA);
    sendData(regstart);
    sendData(length);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Position_Long_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 7) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0){
    //Serial.println("Read register data");
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 ) {   // Error
        //Serial.print("Error byte: "); Serial.println(Error_Byte);
				return (Error_Byte*(-1));
			}
         
			Position_Low_Byte = readData();            // Position Bytes
			
			if(length == 2)
				Position_High_Byte = readData();
			else
				Position_High_Byte = 0;
				
			Position_Long_Byte = Position_High_Byte << 8; 
			Position_Long_Byte = Position_Long_Byte + Position_Low_Byte;
		}
    }
	return (Position_Long_Byte);     // Returns the read position
}

/* Set the value of a single-byte register. */
void DynamixelSerial::setRegister(int id, int regstart, int data)
{
    int checksum = ~((id + 4 + AX_WRITE_DATA + regstart + (data&0xff)) % 256);

	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(id);
    sendData(4);    // length
    sendData(AX_WRITE_DATA);
    sendData(regstart);
    sendData(data&0xff);
    // checksum = 
    sendData(checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
}

/* Set the value of a double-byte register. */
void DynamixelSerial::setRegister2(int id, int regstart, int data)
{
    int checksum = ~((id + 5 + AX_WRITE_DATA + regstart + (data&0xFF) + ((data&0xFF00)>>8)) % 256);

	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(id);
    sendData(5);    // length
    sendData(AX_WRITE_DATA);
    sendData(regstart);
    sendData(data&0xff);
    sendData((data&0xff00)>>8);
    sendData(checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
}

int DynamixelSerial::readVoltage(unsigned char ID)
{    
    Checksum = (~(ID + AX_VOLT_LENGTH  + AX_READ_DATA + AX_PRESENT_VOLTAGE + AX_BYTE_READ))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(ID);
    sendData(AX_VOLT_LENGTH);
    sendData(AX_READ_DATA);
    sendData(AX_PRESENT_VOLTAGE);
    sendData(AX_BYTE_READ);
    sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Voltage_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			Voltage_Byte = readData();             // Voltage
		}
    }
	return (Voltage_Byte);               // Returns the read Voltage
}

int DynamixelSerial::setTempLimit(unsigned char ID, unsigned char Temperature)
{
	Checksum = (~(ID + AX_TL_LENGTH +AX_WRITE_DATA+ AX_LIMIT_TEMPERATURE + Temperature))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	sendData(AX_START);                     
	sendData(AX_START);
	sendData(ID);
	sendData(AX_TL_LENGTH);
	sendData(AX_WRITE_DATA);
	sendData(AX_LIMIT_TEMPERATURE);
    sendData(Temperature);
	sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error()); 
}

int DynamixelSerial::setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage)
{
	Checksum = (~(ID + AX_VL_LENGTH +AX_WRITE_DATA+ AX_DOWN_LIMIT_VOLTAGE + DVoltage + UVoltage))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	sendData(AX_START);                     
	sendData(AX_START);
	sendData(ID);
	sendData(AX_VL_LENGTH);
	sendData(AX_WRITE_DATA);
	sendData(AX_DOWN_LIMIT_VOLTAGE);
    sendData(DVoltage);
    sendData(UVoltage);
	sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error()); 
}

int DynamixelSerial::setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit)
{
	char CW_H,CW_L,CCW_H,CCW_L;
    CW_H = CWLimit >> 8;    
    CW_L = CWLimit;                // 16 bits - 2 x 8 bits variables
    CCW_H = CCWLimit >> 8;
    CCW_L = CCWLimit;  
	Checksum = (~(ID + AX_VL_LENGTH +AX_WRITE_DATA+ AX_CW_ANGLE_LIMIT_L + CW_H + CW_L + AX_CCW_ANGLE_LIMIT_L + CCW_H + CCW_L))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	sendData(AX_START);                     
	sendData(AX_START);
	sendData(ID);
	sendData(AX_CCW_CW_LENGTH);
	sendData(AX_WRITE_DATA);
	sendData(AX_CW_ANGLE_LIMIT_L);
    sendData(CW_L);
	sendData(CW_H);
	sendData(AX_CCW_ANGLE_LIMIT_L);
    sendData(CCW_L);
	sendData(CCW_H);
	sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error()); 
}

int DynamixelSerial::setMaxTorque(unsigned char ID, int MaxTorque)
{
    char MaxTorque_H,MaxTorque_L;
    MaxTorque_H = MaxTorque >> 8;           // 16 bits - 2 x 8 bits variables
    MaxTorque_L = MaxTorque;
	Checksum = (~(ID + AX_MT_LENGTH + AX_WRITE_DATA + AX_MAX_TORQUE_L + MaxTorque_L + MaxTorque_H))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                 // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_MT_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_MAX_TORQUE_L);
    sendData(MaxTorque_L);
    sendData(MaxTorque_H);
    sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error());                 // Return the read error
}

int DynamixelSerial::setSRL(unsigned char ID, unsigned char SRL)
{    
	Checksum = (~(ID + AX_SRL_LENGTH + AX_WRITE_DATA + AX_RETURN_LEVEL + SRL))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
	sendData(AX_SRL_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_RETURN_LEVEL);
    sendData(SRL);
    sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelSerial::setReturnDelayTime(unsigned char ID, unsigned char RDT)
{    
	Checksum = (~(ID + AX_RDT_LENGTH + AX_WRITE_DATA + AX_RETURN_DELAY_TIME + RDT))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
	sendData(AX_RDT_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_RETURN_DELAY_TIME);
    sendData(RDT);
    sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}
/*
int DynamixelSerial::setReturnDelayTime(unsigned char ID, unsigned char Time)
{
	Checksum = (~(ID + AX_RDT_LENGTH +AX_WRITE_DATA+ AX_RETURN_DELAY_TIME + Time))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	sendData(AX_START);                     
	sendData(AX_START);
	sendData(ID);
	sendData(AX_RDT_LENGTH);
	sendData(AX_WRITE_DATA);
	sendData(AX_RETURN_DELAY_TIME);
    sendData(Time);
	sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error()); 
}
*/

int DynamixelSerial::readReturnDelayTime(unsigned char ID)
{	
    Checksum = (~(ID + AX_RDT_LENGTH  + AX_READ_DATA + AX_RETURN_DELAY_TIME + AX_BYTE_READ))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(ID);
    sendData(AX_RDT_LENGTH);
    sendData(AX_READ_DATA);
    sendData(AX_RETURN_DELAY_TIME);
    sendData(AX_BYTE_READ);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Return_Delay_Byte = -1;
    Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			Return_Delay_Byte = readData();         // Temperature
		}
    }
	return (Return_Delay_Byte);               // Returns the read temperature
}

int DynamixelSerial::readCWLimit(unsigned char ID)
{
    return readRegister(ID, AX_CW_ANGLE_LIMIT_L, 2);
}

int DynamixelSerial::readCCWLimit(unsigned char ID)
{
    return readRegister(ID, AX_CCW_ANGLE_LIMIT_L, 2);
}
    
int DynamixelSerial::setLEDAlarm(unsigned char ID, unsigned char LEDAlarm)
{    
	Checksum = (~(ID + AX_LEDALARM_LENGTH + AX_WRITE_DATA + AX_ALARM_LED + LEDAlarm))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
	sendData(AX_LEDALARM_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_ALARM_LED);
    sendData(LEDAlarm);
    sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelSerial::setShutdownAlarm(unsigned char ID, unsigned char SALARM)
{    
	Checksum = (~(ID + AX_SALARM_LENGTH + AX_ALARM_SHUTDOWN + AX_ALARM_LED + SALARM))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
	sendData(AX_SALARM_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_ALARM_SHUTDOWN);
    sendData(SALARM);
    sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelSerial::setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin)
{
	Checksum = (~(ID + AX_CM_LENGTH +AX_WRITE_DATA+ AX_CW_COMPLIANCE_MARGIN + CWCMargin + AX_CCW_COMPLIANCE_MARGIN + CCWCMargin))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	sendData(AX_START);                     
	sendData(AX_START);
	sendData(ID);
	sendData(AX_CM_LENGTH);
	sendData(AX_WRITE_DATA);
	sendData(AX_CW_COMPLIANCE_MARGIN);
    sendData(CWCMargin);
	sendData(AX_CCW_COMPLIANCE_MARGIN);
    sendData(CCWCMargin);
	sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error()); 
}

int DynamixelSerial::setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope)
{
	Checksum = (~(ID + AX_CS_LENGTH +AX_WRITE_DATA+ AX_CW_COMPLIANCE_SLOPE + CWCSlope + AX_CCW_COMPLIANCE_SLOPE + CCWCSlope))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	sendData(AX_START);                     
	sendData(AX_START);
	sendData(ID);
	sendData(AX_CS_LENGTH);
	sendData(AX_WRITE_DATA);
	sendData(AX_CW_COMPLIANCE_SLOPE);
    sendData(CWCSlope);
	sendData(AX_CCW_COMPLIANCE_SLOPE);
    sendData(CCWCSlope);
	sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error()); 
}

int DynamixelSerial::setPunch(unsigned char ID, int Punch)
{
    char Punch_H,Punch_L;
    Punch_H = Punch >> 8;           // 16 bits - 2 x 8 bits variables
    Punch_L = Punch;
	Checksum = (~(ID + AX_PUNCH_LENGTH + AX_WRITE_DATA + AX_PUNCH_L + Punch_L + Punch_H))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                 // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_PUNCH_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_PUNCH_L);
    sendData(Punch_L);
    sendData(Punch_H);
    sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error());                 // Return the read error
}

int DynamixelSerial::moving(unsigned char ID)
{	
    Checksum = (~(ID + AX_MOVING_LENGTH  + AX_READ_DATA + AX_MOVING + AX_BYTE_READ))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(ID);
    sendData(AX_MOVING_LENGTH);
    sendData(AX_READ_DATA);
    sendData(AX_MOVING);
    sendData(AX_BYTE_READ);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Moving_Byte = -1;
    Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			Moving_Byte = readData();         // Temperature
		}
    }
	return (Moving_Byte);               // Returns the read temperature
}

int DynamixelSerial::lockRegister(unsigned char ID)
{    
	Checksum = (~(ID + AX_LR_LENGTH + AX_WRITE_DATA + AX_LOCK + AX12_LOCK))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
	sendData(AX_LR_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_LOCK);
    sendData(AX12_LOCK);
    sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelSerial::setStatusReturnLevel(unsigned char ID, unsigned char level)
{   
    if(level >= 0 && level <=2) {
        Checksum = (~(ID + AX_SRL_LENGTH + AX_WRITE_DATA + AX_RETURN_LEVEL + level))&0xFF;
        
        switchCom(Direction_Pin,Tx_MODE);
        sendData(AX_START);                // Send Instructions over Serial
        sendData(AX_START);
        sendData(ID);
        sendData(AX_SRL_LENGTH);
        sendData(AX_WRITE_DATA);
        sendData(AX_RETURN_LEVEL);
        sendData(level);
        sendData(Checksum);
        delayus(TX_DELAY_TIME);
        switchCom(Direction_Pin,Rx_MODE);
        
        Status_Return_Level = level;

        if(Status_Return_Level == 2)
            return (read_error());                // Return the read error
        else
            return 0;
    }
    else
        return 0;
}

int DynamixelSerial::setCWLimit(unsigned char ID, int limit)
{
    setRegister2(ID, AX_CW_ANGLE_LIMIT_L, limit);
    return 0;
}

int DynamixelSerial::setCCWLimit(unsigned char ID, int limit)
{
    setRegister2(ID, AX_CCW_ANGLE_LIMIT_L, limit);
    return 0;
}

int DynamixelSerial::setSpeed(unsigned char ID, int speed)
{
    setRegister2(ID, AX_GOAL_SPEED_L, speed);
    return 0;
}
   
int DynamixelSerial::RWStatus(unsigned char ID)
{	
    Checksum = (~(ID + AX_RWS_LENGTH  + AX_READ_DATA + AX_REGISTERED_INSTRUCTION + AX_BYTE_READ))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(ID);
    sendData(AX_RWS_LENGTH);
    sendData(AX_READ_DATA);
    sendData(AX_REGISTERED_INSTRUCTION);
    sendData(AX_BYTE_READ);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    RWS_Byte = -1;
    Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			RWS_Byte = readData();         // Temperature
		}
    }
	return (RWS_Byte);               // Returns the read temperature
}

int DynamixelSerial::readSpeed(unsigned char ID)
{	
    Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_SPEED_L + AX_BYTE_READ_POS))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(ID);
    sendData(AX_POS_LENGTH);
    sendData(AX_READ_DATA);
    sendData(AX_PRESENT_SPEED_L);
    sendData(AX_BYTE_READ_POS);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Speed_Long_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 7) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			
			Speed_Low_Byte = readData();            // Position Bytes
			Speed_High_Byte = readData();
			Speed_Long_Byte = Speed_High_Byte << 8; 
			Speed_Long_Byte = Speed_Long_Byte + Speed_Low_Byte;
		}
    }
	return (Speed_Long_Byte);     // Returns the read position
}

int DynamixelSerial::readLoad(unsigned char ID)
{	
    Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_LOAD_L + AX_BYTE_READ_POS))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(ID);
    sendData(AX_POS_LENGTH);
    sendData(AX_READ_DATA);
    sendData(AX_PRESENT_LOAD_L);
    sendData(AX_BYTE_READ_POS);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Load_Long_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 7) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0){
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			
			Load_Low_Byte = readData();            // Position Bytes
			Load_High_Byte = readData();
			Load_Long_Byte = Load_High_Byte << 8; 
			Load_Long_Byte = Load_Long_Byte + Load_Low_Byte;
		}
    }
	return (Load_Long_Byte);     // Returns the read position
}

int DynamixelSerial::makeWord(int low, int high) 
{
	return (low + (high<<8));
};

int DynamixelSerial::getLowByte(int val) 
{
	return (val & 0xff);
};

int DynamixelSerial::getHighByte(int val) 
{
	return ((val & 0xff00)>> 8);
};

void DynamixelSerial::startSyncWrite(bool use_speed)
{
	total_sync_servos = 0;
	use_speed_synch = use_speed;
}

int DynamixelSerial::dataSizePerServoSynch()
{
	if(use_speed_synch)
		return AX12_SYNCH_PER_SERVO_WSPEED;
	else
		return AX12_SYNCH_PER_SERVO_WOSPEED;
}

void DynamixelSerial::addServoToSync(int id, int goal_pos, int goal_speed)
{
	int data_size_per_servo = dataSizePerServoSynch();

	int idx = total_sync_servos*data_size_per_servo;

	sync_data[idx] = id;
	sync_data[idx+1] = getLowByte(goal_pos);
	sync_data[idx+2] = getHighByte(goal_pos);

	if(use_speed_synch) {
		sync_data[idx+3] = getLowByte(goal_speed);
		sync_data[idx+4] = getHighByte(goal_speed);
	}
	
	//Serial.print("servos: ");  
	//Serial.print(total_sync_servos);  
	//Serial.print(" idx: ");  
	//Serial.print(idx);  
	//Serial.print(" id: ");  
	//Serial.print(id);  
	//Serial.print(" pos: ");  
	//Serial.print(goal_pos); 
	//Serial.print(" speed: ");  
	//Serial.print(goal_speed); 
	//Serial.print("\n");
		
	//for(int i=0; i<(idx+data_size_per_servo); i++)
	//{
	//	Serial.print(sync_data[i], HEX);  
	//	Serial.print(", ");
	//}
	//Serial.println("\n");

    total_sync_servos++;	
}

void DynamixelSerial::writeSyncData(bool print_data)
{	
    int temp;
	int data_size_per_servo = dataSizePerServoSynch();

	int	length = 4 + (data_size_per_servo * total_sync_servos);		
    int Checksum = 254 + length + AX_SYNC_WRITE + AX_GOAL_POSITION_L + (data_size_per_servo-1);

	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(BROADCAST_ID);
    sendData(length);
    sendData(AX_SYNC_WRITE);
    sendData(AX_GOAL_POSITION_L);
    sendData(data_size_per_servo-1);

	int dataidx=0, id=0;
    for(int servo=0; servo<total_sync_servos; servo++)
    {
		//Now write the data for the servo
		for(int servo_data_idx=0; servo_data_idx<data_size_per_servo; servo_data_idx++)
		{
			temp = sync_data[dataidx];
			Checksum += temp;
			sendData(temp);
			dataidx++;
		}
    } 

	unsigned char sum = (0xff - (Checksum % 256));
    sendData(sum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
	if(print_data)
		printSyncData(length, sum);
	
}

void DynamixelSerial::printSyncData(int length, unsigned char checksum)
{
	int data_size_per_servo = dataSizePerServoSynch();
    int Checksum = BROADCAST_ID + length + AX_SYNC_WRITE + AX_GOAL_POSITION_L + (data_size_per_servo-1);
	
    
	Serial.print(0xFF, HEX);  
	Serial.print(", ");

	Serial.print(0xFF, HEX);  
	Serial.print(", ");

	Serial.print(BROADCAST_ID, HEX);  
	Serial.print(", ");

	Serial.print(length, HEX);  
	Serial.print(", ");

	Serial.print(AX_SYNC_WRITE, HEX);  
	Serial.print(", ");

	Serial.print(AX_GOAL_POSITION_L, HEX);  
	Serial.print(", ");

	Serial.print(data_size_per_servo-1, HEX);  
	Serial.print(", ");
	
	int dataidx=0, id=0, temp=0;
    for(int servo=0; servo<total_sync_servos; servo++)
    {
		//Now write the data for the servo
		for(int servo_data_idx=0; servo_data_idx<data_size_per_servo; servo_data_idx++)
		{
			temp = sync_data[dataidx];
			Serial.print(temp, HEX);  
			Serial.print(", ");
			Checksum += temp;
			dataidx++;
		}
    } 
	unsigned char sum = (0xff - (Checksum % 256));
    
	Serial.print(checksum, HEX);  
	//Serial.print(sum, HEX);  
	Serial.print("\n");
}	

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };
 
    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
 
    return crc_accum;
}
/*
int DynamixelSerial::move2(unsigned char ID, int Position)
{
    unsigned char data[x];
    
    char Position_H,Position_L;
    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;

    data[0] = AX_START;
    data[1] = AX_START;
    data[2] = 0xFD;
    data[3] = 0;
    data[4] = ID;
    data[5] = LengthLow;
    data[6] = LengthHigh;
    data[7] = AX_WRITE_DATA;
    data[8] = AX_GOAL_POSITION_L;
    data[9] = 0x00;  //High byte of goal position
    data[10] = Position_L;
    data[11] = Position_H;
    
    unsigned short checksum = calc_crc16(;
    

	Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                 // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_GOAL_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_GOAL_POSITION_L);
    sendData(Position_L);
    sendData(Position_H);
    sendData(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);

}
*/

int DynamixelSerial::move2(unsigned char id, int value){

    /*Dynamixel 2.0 communication protocol
      used by Dynamixel XL-320 and Dynamixel PRO only.
    */

    // technically i think we need 14bytes for this packet 
    int Address = AX_GOAL_POSITION_L;
    
    const int bufsize = 16;

    byte txbuffer[bufsize];

    Packet p(txbuffer,bufsize,id,0x03,4,
	getLowByte(Address),
	getHighByte(Address),
	getLowByte(value),
	getHighByte(value));

    int size = p.getSize();
	switchCom(Direction_Pin,Tx_MODE);
    stream->write(txbuffer,size);
    stream->flush();
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);

    return bufsize;	
}

DynamixelSerial::Packet::Packet(
	unsigned char *data,
	size_t data_size,
	unsigned char id,
	unsigned char instruction,
	int parameter_data_size,
	...) {


    // [ff][ff][fd][00][id][len1][len2] { [instr][params(parameter_data_size)][crc1][crc2] }
    unsigned int length=3+parameter_data_size;
    if(!data) {
	// [ff][ff][fd][00][id][len1][len2] { [data(length)] }
	this->data_size = 7+length;   
	this->data = (unsigned char*)malloc(data_size);
	this->freeData = true;
    } else {
	this->data = data;
	this->data_size = data_size;
	this->freeData = false;
    }
    this->data[0]=0xFF;
    this->data[1]=0xFF;
    this->data[2]=0xFD;
    this->data[3]=0x00;
    this->data[4]=id;
    this->data[5]=length&0xff;
    this->data[6]=(length>>8)&0xff;
    this->data[7]=instruction;
    va_list args;
    va_start(args, parameter_data_size); 
    for(int i=0;i<parameter_data_size;i++) {
	unsigned char arg = va_arg(args, int);
	this->data[8+i]=arg;
    }
    unsigned short crc = update_crc(0,this->data,this->getSize()-2);
    this->data[8+parameter_data_size]=crc&0xff;
    this->data[9+parameter_data_size]=(crc>>8)&0xff;
    va_end(args);
}

DynamixelSerial::Packet::Packet(unsigned char *data, size_t size) {
    this->data = data;
    this->data_size = size;
    this->freeData = false;
}


DynamixelSerial::Packet::~Packet() {
    if(this->freeData==true) {
	free(this->data);
    }
}

void DynamixelSerial::Packet::toStream(Stream &stream) {
    stream.print("id: ");
    stream.println(this->getId(),DEC);
    stream.print("length: ");
    stream.println(this->getLength(),DEC);
    stream.print("instruction: ");
    stream.println(this->getInstruction(),HEX);
    stream.print("parameter count: ");
    stream.println(this->getParameterCount(), DEC);
    for(int i=0;i<this->getParameterCount(); i++) {
	stream.print(this->getParameter(i),HEX);
	if(i<this->getParameterCount()-1) {
	    stream.print(",");
	}
    }
    stream.println();
    stream.print("valid: ");
    stream.println(this->isValid()?"yes":"no");
}

unsigned char DynamixelSerial::Packet::getId() {
    return data[4];
}

int DynamixelSerial::Packet::getLength() {
    return data[5]+((data[6]&0xff)<<8);
}

int DynamixelSerial::Packet::getSize() {
    return getLength()+7;
}

int DynamixelSerial::Packet::getParameterCount() {
    return getLength()-3;
}

unsigned char DynamixelSerial::Packet::getInstruction() {
    return data[7];
}

unsigned char DynamixelSerial::Packet::getParameter(int n) {
    return data[8+n];
}

bool DynamixelSerial::Packet::isValid() {
    int length = getLength();
    unsigned short storedChecksum = data[length+5]+(data[length+6]<<8);
    return storedChecksum == update_crc(0,data,length+5);
}

#ifdef __SAM3X8E__

void SetSystemCoreClockFor1Mbaud()
{
#define SYS_BOARD_PLLAR     (CKGR_PLLAR_ONE \
                            | CKGR_PLLAR_MULA(39) \
                            | CKGR_PLLAR_PLLACOUNT(100) \
                            | CKGR_PLLAR_DIVA(3))

    // change and update system core clock

    // Configure PLLA
    PMC->CKGR_PLLAR = SYS_BOARD_PLLAR;
    while (!(PMC->PMC_SR & PMC_SR_LOCKA));
    
    SystemCoreClockUpdate();
}

#endif
