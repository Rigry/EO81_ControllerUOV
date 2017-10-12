/*
 * MBSlave.h
 *
 * Created: 07.02.2017 14:55:19
 *  Author: dvk
 *
 */ 

#include <stdint.h>
//#include <util/crc16.h>	//для атмеги
#include <stdbool.h>
#include "defines.h"
#include "crc.h"


#ifndef MBSLAVE_H_
#define MBSLAVE_H_
	
	struct MBRegSt {
		volatile uint16_t RegIn[QTY_IN_REG];
		volatile uint16_t RegOut[QTY_OUT_REG];
		uint16_t RegInMinVal[QTY_IN_REG];
		uint16_t RegInMaxVal[QTY_IN_REG];
	};
	
	void MBSlave(struct UartBufSt *Buf,
				 struct MBRegSt *Reg,
				 uint8_t Addr);
//	uint16_t crc16(uint8_t *p, uint8_t len); 		//для атмеги
	uint16_t crc16(uint8_t* data, uint8_t length);	//для arm

#endif /* MBSLAVE_H_ */

