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
			Serial.print("Received Error: ");
			Serial.println(Error_Byte);
            
            if(Time_Counter == TIME_OUT)
                Serial.println("Timed out");
                
			return (Error_Byte);
		}
	}

	Serial.print("Nothing Recieved: ");
	Serial.println(-1);
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
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) ){
			readData();                            // Start Bytes
			readData();                            // Ax-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
    
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
	Checksum = (~(ID + AX_LR_LENGTH + AX_WRITE_DATA + AX_LOCK + LOCK))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);                // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
	sendData(AX_LR_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_LOCK);
    sendData(LOCK);
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

	total_sync_servos++;	
	
	//Serial.print("servos: ");  
	//Serial.print(g_total_sync_servos);  
	//Serial.print("idx: ");  
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
	//	Serial.print(g_sync_data[i]);  
	//	Serial.print(", ");
	//}
	//Serial.print("\n");
}

void DynamixelSerial::writeSyncData(bool print_data)
{	
    int temp;
	int data_size_per_servo = dataSizePerServoSynch();

	int	length = 4 + (data_size_per_servo * total_sync_servos);		
    int Checksum = 254 + length + AX_SYNC_WRITE + AX_GOAL_POSITION_L + data_size_per_servo;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(AX_START);
    sendData(AX_START);
    sendData(BROADCAST_ID);
    sendData(length);
    sendData(AX_SYNC_WRITE);
    sendData(AX_GOAL_POSITION_L);
    sendData(data_size_per_servo);

	int dataidx=0, id=0;
    for(int servo=0; servo<total_sync_servos; servo++)
    {
		id = sync_data[dataidx];
		sendData(id);  //Write the id
		Checksum += id;
		dataidx++;
		
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
	
	Serial.print(0xFF);  
	Serial.print(", ");

	Serial.print(0xFF);  
	Serial.print(", ");

	Serial.print(0xFE);  
	Serial.print(", ");

	Serial.print(length);  
	Serial.print(", ");

	Serial.print(AX_SYNC_WRITE);  
	Serial.print(", ");

	Serial.print(AX_GOAL_POSITION_L);  
	Serial.print(", ");

	Serial.print(AX12_SYNC_DATA_PER_SERVO);  
	Serial.print(", ");
	
	int dataidx=0, id=0, temp=0;
    for(int servo=0; servo<total_sync_servos; servo++)
    {
		id = sync_data[dataidx];
		Serial.print(id);  
		Serial.print(", ");

		dataidx++;
		
		//Now write the data for the servo
		for(int servo_data_idx=0; servo_data_idx<data_size_per_servo; servo_data_idx++)
		{
			temp = sync_data[dataidx];
			Serial.print(temp);  
			Serial.print(", ");
			dataidx++;
		}
    } 

	Serial.print(checksum);  
	Serial.print("\n");
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
