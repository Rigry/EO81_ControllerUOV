#include "crc.h"

uint16_t crc16(uint8_t* data, uint8_t length)
{
	int j;
	uint16_t reg_crc = 0xFFFF;
	while(length--)	{
		reg_crc ^= *data++;
		for(j = 0; j < 8; j++) {
			if (reg_crc & 0x01) {
				reg_crc=(reg_crc>>1) ^ 0xA001; // LSB(b0)=1
			} else {
				reg_crc=reg_crc>>1;
			}	
		}
	}
	return reg_crc;
}