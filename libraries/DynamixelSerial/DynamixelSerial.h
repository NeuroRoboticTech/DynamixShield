/*
 Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 Created by Savage on 27/01/11.
 
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
 
 David Cofer
 16/07/2015 - Modified the library to be able to pass in any hardware serial reference at 
              instantiation. This removes the need to have multiple redundant files.
			  Modified the code to work with the new DynamixShield board and added a 
			  number of new functions including:
			  1. Synchronous write to multiple servos simultaneously
			  2. Set/Get return delay time
			  3. Set/Get Status return level
			  4. Set/Get CW and CCW limits
			  5. Generic Set/read register and register2
			  6. Helper functions for makework, and get low and high bytes
 
 FUNCION SYNCWRITE.
 
 *****************************************************************************
 
 Contact: savageelectronics@gmail.com 
 Web:     http://savageelectrtonics.blogspot.com/
 Autor:   Josue Alejandro Savage
 
 Secondary Author: David Cofer
 NeuroRobotic Technologies
 Web: www.NeuroRoboticTech.com 
 */

#ifndef DynamixelSerial1_h
#define DynamixelSerial1_h

	// EEPROM AREA  ///////////////////////////////////////////////////////////
#define AX_MODEL_NUMBER_L            0
#define AX_MODEL_NUMBER_H            1
#define AX_VERSION                   2
#define AX_ID                        3
#define AX_BAUD_RATE                 4
#define AX_RETURN_DELAY_TIME         5 
#define AX_CW_ANGLE_LIMIT_L          6
#define AX_CW_ANGLE_LIMIT_H          7
#define AX_CCW_ANGLE_LIMIT_L         8
#define AX_CCW_ANGLE_LIMIT_H         9
#define AX_SYSTEM_DATA2              10
#define AX_LIMIT_TEMPERATURE         11
#define AX_DOWN_LIMIT_VOLTAGE        12
#define AX_UP_LIMIT_VOLTAGE          13
#define AX_MAX_TORQUE_L              14
#define AX_MAX_TORQUE_H              15
#define AX_RETURN_LEVEL              16
#define AX_ALARM_LED                 17
#define AX_ALARM_SHUTDOWN            18
#define AX_OPERATING_MODE            19
#define AX_DOWN_CALIBRATION_L        20
#define AX_DOWN_CALIBRATION_H        21
#define AX_UP_CALIBRATION_L          22
#define AX_UP_CALIBRATION_H          23

	// RAM AREA  //////////////////////////////////////////////////////////////
#define AX_TORQUE_ENABLE             24
#define AX_LED                       25
#define AX_CW_COMPLIANCE_MARGIN      26
#define AX_CCW_COMPLIANCE_MARGIN     27
#define AX_CW_COMPLIANCE_SLOPE       28
#define AX_CCW_COMPLIANCE_SLOPE      29
#define AX_GOAL_POSITION_L           30
#define AX_GOAL_POSITION_H           31
#define AX_GOAL_SPEED_L              32
#define AX_GOAL_SPEED_H              33
#define AX_TORQUE_LIMIT_L            34
#define AX_TORQUE_LIMIT_H            35
#define AX_PRESENT_POSITION_L        36
#define AX_PRESENT_POSITION_H        37
#define AX_PRESENT_SPEED_L           38
#define AX_PRESENT_SPEED_H           39
#define AX_PRESENT_LOAD_L            40
#define AX_PRESENT_LOAD_H            41
#define AX_PRESENT_VOLTAGE           42
#define AX_PRESENT_TEMPERATURE       43
#define AX_REGISTERED_INSTRUCTION    44
#define AX_PAUSE_TIME                45
#define AX_MOVING                    46
#define AX_LOCK                      47
#define AX_PUNCH_L                   48
#define AX_PUNCH_H                   49

    // Status Return Levels ///////////////////////////////////////////////////////////////
#define AX_RETURN_NONE               0
#define AX_RETURN_READ               1
#define AX_RETURN_ALL                2

    // Instruction Set ///////////////////////////////////////////////////////////////
#define AX_PING                      1
#define AX_READ_DATA                 2
#define AX_WRITE_DATA                3
#define AX_REG_WRITE                 4
#define AX_ACTION                    5
#define AX_RESET                     6
#define AX_SYNC_WRITE                131

	// Specials ///////////////////////////////////////////////////////////////
#define OFF                          0
#define ON                           1
#define LEFT						 0
#define RIGTH                        1
#define AX_BYTE_READ                 1
#define AX_BYTE_READ_POS             2
#define AX_RESET_LENGTH				 2
#define AX_ACTION_LENGTH			 2
#define AX_ID_LENGTH                 4
#define AX_LR_LENGTH                 4
#define AX_SRL_LENGTH                4
#define AX_RDT_LENGTH                4
#define AX_LEDALARM_LENGTH           4
#define AX_SALARM_LENGTH             4
#define AX_TL_LENGTH                 4
#define AX_VL_LENGTH                 6
#define AX_CM_LENGTH                 6
#define AX_CS_LENGTH                 6
#define AX_CCW_CW_LENGTH             8
#define AX_BD_LENGTH                 4
#define AX_TEM_LENGTH                4
#define AX_MOVING_LENGTH             4
#define AX_RWS_LENGTH                4
#define AX_VOLT_LENGTH               4
#define AX_LED_LENGTH                4
#define AX_TORQUE_LENGTH             4
#define AX_POS_LENGTH                4
#define AX_GOAL_LENGTH               5
#define AX_MT_LENGTH                 5
#define AX_PUNCH_LENGTH              5
#define AX_SPEED_LENGTH              5
#define AX_GOAL_SP_LENGTH            7
#define AX_ACTION_CHECKSUM			 250
#define BROADCAST_ID                 254
#define AX_START                     255
#define AX_CCW_AL_L                  255 
#define AX_CCW_AL_H                  3
#define TIME_OUT                     10         // Este parametro depende de la velocidad de transmision
#define TX_DELAY_TIME				 300        // Este parametro depende de la velocidad de transmision - pero pueden ser cambiados para mayor velocidad.
#define Tx_MODE                      1
#define Rx_MODE                      0
#define AX12_LOCK                    1

#define AX12_MAX_SERVOS              50
#define AX12_BUFFER_SIZE             32
#define AX12_SYNC_DATA_PER_SERVO	 4
#define AX12_MAX_SYNCH_BUFFER_SIZE   250
#define AX12_SYNCH_PER_SERVO_WSPEED  5
#define AX12_SYNCH_PER_SERVO_WOSPEED 3


#include <inttypes.h>
#include "HardwareSerial.h"

#ifdef __SAM3X8E__
    void SetSystemCoreClockFor1Mbaud();
#endif

#ifdef __SAMD21G18A__ || __SAMD21G18A__
    #define DYN_CTRL_PIN 2
#else
    #define DYN_CTRL_PIN 22
#endif

class DynamixelSerial {
protected:
	
	HardwareSerial *stream;
	
	unsigned char Checksum; 
	unsigned char Direction_Pin;
	unsigned char Time_Counter;
	unsigned char Incoming_Byte;               
	unsigned char Position_High_Byte;
	unsigned char Position_Low_Byte;
	unsigned char Speed_High_Byte;
	unsigned char Speed_Low_Byte;
	unsigned char Load_High_Byte;
	unsigned char Load_Low_Byte;
	
	int Moving_Byte;
	int RWS_Byte;
	int Speed_Long_Byte;
	int Load_Long_Byte;
	int Position_Long_Byte;
	int Temperature_Byte;
	int Voltage_Byte;
	int Error_Byte; 
	int Return_Delay_Byte;
	bool use_speed_synch;
    unsigned char Status_Return_Level;
	  
	int total_sync_servos;
	unsigned char sync_data[AX12_MAX_SYNCH_BUFFER_SIZE];
	  
	int read_error(void);
	int dataSizePerServoSynch();

	void sendData(const uint8_t data);
	int availableData();
	int readData();
	int peekData();
  void flushData();
	void beginCom(unsigned long baudRate);
	void endCom();
	    
public:
	DynamixelSerial(HardwareSerial *ss = NULL);
	
	void begin(long baud = 1000000, 
        unsigned char directionPin = DYN_CTRL_PIN);
	void begin(long baud);
	void end(void);
	
	int reset(unsigned char ID);
	int ping(unsigned char ID); 
	
	int setID(unsigned char ID, unsigned char newID);
	int setBD(unsigned char ID, long baud);
	
	int move(unsigned char ID, int Position);
	int moveSpeed(unsigned char ID, int Position, int Speed);
  int setSpeed(unsigned char ID, int Speed);
  int stop(unsigned char ID);
	int setEndless(unsigned char ID,bool Status);
	int turn(unsigned char ID, bool SIDE, int Speed);
	int moveRW(unsigned char ID, int Position);
	int moveSpeedRW(unsigned char ID, int Position, int Speed);
	
	void startSyncWrite(bool use_speed = true);
	void addServoToSync(int id, int goal_pos, int goal_speed);
	void writeSyncData(bool print_data = false);
	void printSyncData(int length, unsigned char checksum);
	
	void action(void);
	
	int setTempLimit(unsigned char ID, unsigned char Temperature);
	int setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit);
	int setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage);
	int setMaxTorque(unsigned char ID, int MaxTorque);
	int setSRL(unsigned char ID, unsigned char SRL);
	int setReturnDelayTime(unsigned char ID, unsigned char RDT);
	int setLEDAlarm(unsigned char ID, unsigned char LEDAlarm);
	int setShutdownAlarm(unsigned char ID, unsigned char SALARM);
	int setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin);
	int setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope);
	int setPunch(unsigned char ID, int Punch);
    int setStatusReturnLevel(unsigned char ID, unsigned char level);
	int setCWLimit(unsigned char ID, int limit);
    int setCCWLimit(unsigned char ID, int limit);
    
	int moving(unsigned char ID);
	int lockRegister(unsigned char ID);
	int RWStatus(unsigned char ID);
	
	int readTemperature(unsigned char ID);
	int readVoltage(unsigned char ID);
	int readPosition(unsigned char ID);
	int readSpeed(unsigned char ID);
	int readLoad(unsigned char ID);
	int readReturnDelayTime(unsigned char ID);
    int readCWLimit(unsigned char ID);
    int readCCWLimit(unsigned char ID);
	
	int readRegister(unsigned char ID, int regstart, int length);
    void setRegister(int id, int regstart, int data);
    void setRegister2(int id, int regstart, int data);
	
	int torqueStatus(unsigned char ID, bool Status);
	int ledStatus(unsigned char ID, bool Status);
	
	int makeWord(int low, int high);
	int getLowByte(int val);
	int getHighByte(int val);	
    
    
	int move2(unsigned char id, int value);
    
	class Packet {
	  bool freeData;
	  public:
	    unsigned char *data;
	    size_t data_size;

	    // wrap a received data stream in an Packet object for analysis
	    Packet(unsigned char *data, size_t size);
	    // build a packet into the pre-allocated data array
	    // if data is null it will be malloc'ed and free'd on destruction.
	    
	    Packet(
	      unsigned char *data, 
	      size_t        size,
	      unsigned char id,
	      unsigned char instruction,
	      int           parameter_data_size,
	      ...);
	    ~Packet();
	    unsigned char getId();
	    int getLength();
	    int getSize();
	    int getParameterCount();
	    unsigned char getInstruction();
            unsigned char getParameter(int n);
	    bool isValid();

	    void toStream(Stream &stream);

	}; 
    
};


#endif
