/*
  CommanderHS.cpp - Library for interfacing with ArbotiX CommanderHS
  Copyright (c) 2009-2012 Michael E. Ferguson.  All right reserved.

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

#include <Arduino.h>
#include "CommanderHS.h"

/* Constructor */
CommanderHS::CommanderHS(){
    index = -1;
    status = 0;
	stream = NULL;
    walkVOffset = 0;      
    walkHOffset = 0;      
    lookVOffset = 0;      
    lookHOffset = 0;	
}

CommanderHS::CommanderHS(HardwareSerial *ss){
    index = -1;
    status = 0;
	stream = ss;
}

void CommanderHS::begin(unsigned long baud){
	if(stream != NULL)
		stream->begin(baud);
	else
	{
		stream = &Serial;
		stream->begin(baud);
	}
}

/* SouthPaw Support */
void CommanderHS::UseSouthPaw(){
    status |= 0x01;
}

/* process messages coming from CommanderHS 
 *  format = 0xFF RIGHT_H RIGHT_V LEFT_H LEFT_V BUTTONS EXT CHECKSUM */
int CommanderHS::ReadMsgs(){
	if(stream != NULL) {
		while(stream->available() > 0){
			if(index == -1){         // looking for new packet
				if(stream->read() == 0xff){
					index = 0;
					checksum = 0;
				}
			}else if(index == 0){
				vals[index] = (unsigned char) stream->read();
				if(vals[index] != 0xff){            
					checksum += (int) vals[index];
					index++;
				}
			}else{
				vals[index] = (unsigned char) stream->read();
				checksum += (int) vals[index];
				index++;
				if(index == 7){ // packet complete
					if(checksum%256 != 255){
						// packet error!
						index = -1;
						return 0;
					}else{
						if((status&0x01) > 0){     // SouthPaw
							walkV = (signed char)( (int)vals[0]-128 + walkVOffset );
							walkH = (signed char)( (int)vals[1]-128 + walkHOffset );
							lookV = (signed char)( (int)vals[2]-128 + lookVOffset );
							lookH = (signed char)( (int)vals[3]-128 + lookHOffset );
						}else{
							lookV = (signed char)( (int)vals[0]-128 + lookVOffset );
							lookH = (signed char)( (int)vals[1]-128 + lookHOffset );
							walkV = (signed char)( (int)vals[2]-128 + walkVOffset );
							walkH = (signed char)( (int)vals[3]-128 + walkHOffset );
						}
						pan = (vals[0]<<8) + vals[1];
						tilt = (vals[2]<<8) + vals[3];
						buttons = vals[4];
						ext = vals[5];
					}
					index = -1;
					stream->flush();
					return 1;
				}
			}
		}
	}
		
    return 0;
}
