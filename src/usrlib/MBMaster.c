/*
 * MBSlave.c
 *
 * Created: 07.02.2017 15:18:22
 *  Author: dvk
 */ 

#include "MBMaster.h"
#include "stm32f10x.h" //для ARM


/*
#define MBFUNC03	(Buf->Buf[1] == 3)		//из спецификации на модбас чтение регистров мастером
#define MBFUNC16	(Buf->Buf[1] == 16)		//из спецификации на модбас установка регистров мастером
#define OURADDR		(Buf->Buf[0] == Addr)
#define ALLADDR		(Buf->Buf[0] == 0)		//широковещательный запрос
*/

//чтение регистров удаленного устройства (Modbus Master 03 function)
enum MBMFuncOut MBM03					
	(uint8_t DevAddr,			//адрес устройства
	 uint16_t RegAddr,			//адрес первого регистра
	 uint16_t QtyReg,			//количество регистров
	 uint16_t Buf[],			//записываемые данные 
	 struct UartBufSt *UartBuf,	//буфер уарта
	 eTimer_t Timer		        //для таймаута
	)
{
	//Реализована автоматом
	enum MBM03Step{	//состояния автомата - по порядку в switch
		Pack,
		WaitAnswer,
		LengthCheck,
		CRCCheck,
		Unpack,
	};
	//возможно потом придёться вместо статика переделывать на указатель,
	//тк если будет работать на 2 порта в мастере, то не выйдет
	static enum MBM03Step Step = Pack;
	
	uint16_t CRCpack;
    uint8_t QtyByte;
    QtyByte = 0;
	uint8_t i;
	
	while (1) {
		switch (Step) {
		case Pack:
			UartBuf->Buf[0] = DevAddr;
			UartBuf->Buf[1] = 03;
			UartBuf->Buf[2] = RegAddr / 256;
			UartBuf->Buf[3] = RegAddr % 256;
			UartBuf->Buf[4] = QtyReg / 256;
			UartBuf->Buf[5] = QtyReg % 256;
			CRCpack = crc16(&(UartBuf->Buf[0]), 6);
			UartBuf->Buf[6] = CRCpack % 256;
			UartBuf->Buf[7] = CRCpack / 256;
			UartBuf->NeedSend = 8;
			UartBuf->MBEnd = false;
            UartBuf->N = 0;
            TimerSetTimeAndStart (Timer, MB_TIMEOUT);
			Step = WaitAnswer;		
			break;
		
		case WaitAnswer:
			if ( TimerEvent(Timer) ) {
                TimerStop (Timer);
				Step = Pack;
				return (TimeoutErr);
			} else if (UartBuf->MBEnd) {
				UartBuf->MBEnd = false;
				Step = LengthCheck;
				TimerStop (Timer);	
			} else {
				return (FuncInWork);
			}
			break;
			
		case LengthCheck:
			QtyByte = 3+QtyReg*2+2;		//спецификация Модбас: адрес, функция, кол-во байт, данные, CRC
			if ( !((UartBuf->N == QtyByte) || (UartBuf->N == (QtyByte+1)) ) ) {		//КОСТЫЛЬ разрешаю на 1 больше байт в ответе
				UartBuf->N = 0;
				Step = Pack;
				return (WrongAnswer);
			} else {
				UartBuf->N = 0;
				Step = CRCCheck;
			}	
			break;
			
		case CRCCheck:
			CRCpack = (uint16_t)UartBuf->Buf[QtyByte-1] * 256 + UartBuf->Buf[QtyByte-2];
			if (CRCpack != crc16(&(UartBuf->Buf[0]), QtyByte-2)) {
				Step = Pack;
				return (WrongAnswer);			
			} else {
				Step = Unpack;
			}	
			break;
			
		case Unpack:
			if (BitIsSet(UartBuf->Buf[1],7)) {	//спецификация Модбас, признак ошибки
				Step = Pack;
				return (UartBuf->Buf[1] % 2);				
			} else {
				for (i = 0; i < QtyReg; i++) {
					Buf[i] = (uint16_t)UartBuf->Buf[3+2*i] * 256 + UartBuf->Buf[3+2*i+1];
				}
				Step = Pack;
				return (FuncDoneNoErr);
			}	
			break;
		} //switch
	} //while	
} //enum MBMFuncOut MBM03

//запись регистров удаленного устройства (Modbus Master 16 function)
enum MBMFuncOut MBM16					
	(uint8_t DevAddr,			//адрес устройства
	 uint16_t RegAddr,			//адрес первого регистра
	 uint16_t QtyReg,			//количество регистров
	 uint16_t Buf[],			//буфер для считанных данных
	 struct UartBufSt *UartBuf,	//буфер уарта
	 eTimer_t Timer		        //для таймаута
	)
{
	//Реализована автоматом
	enum MBM16Step{	//состояния автомата - по порядку в switch
		Pack,
		WaitAnswer,
		LengthCheck,
		CRCCheck,
		Unpack,
	};
	//возможно потом придёться вместо статика переделывать на указатель,
	//тк если будет работать на 2 порта в мастере, то не выйдет
	static enum MBM16Step Step = Pack;
	
	uint16_t CRCpack;
	uint8_t QtyByte = 0;
	uint8_t i;
	
	while (1) {
		switch (Step) {
		case Pack:
			UartBuf->Buf[0] = DevAddr;
			UartBuf->Buf[1] = 16;
			UartBuf->Buf[2] = RegAddr / 256;
			UartBuf->Buf[3] = RegAddr % 256;
			UartBuf->Buf[4] = QtyReg / 256;
			UartBuf->Buf[5] = QtyReg % 256;
			UartBuf->Buf[6] = QtyReg * 2;
			for (i = 0; i < QtyReg; i++) {
				UartBuf->Buf[7+2*i] = Buf[i] / 256;
				UartBuf->Buf[7+2*i+1] = Buf[i] % 256;
			}
			QtyByte = 7+2*QtyReg;
			CRCpack = crc16(&(UartBuf->Buf[0]), QtyByte);
			UartBuf->Buf[QtyByte] = CRCpack % 256;
			UartBuf->Buf[QtyByte+1] = CRCpack / 256;
			UartBuf->NeedSend = QtyByte+2;
			UartBuf->MBEnd = false;
            UartBuf->N = 0;
            TimerSetTimeAndStart (Timer, MB_TIMEOUT);
			Step = WaitAnswer;		
			break;
		
		case WaitAnswer:
            if ( TimerEvent(Timer) ) {
                TimerStop (Timer);
                Step = Pack;
                return (TimeoutErr);
			} else if (UartBuf->MBEnd) {
				UartBuf->MBEnd = false;
				Step = LengthCheck;
				TimerStop (Timer);	
			} else {
				return (FuncInWork);
			}
			break;
			
		case LengthCheck:
			QtyByte = 8;		//спецификация Модбас: адрес, функция, регистр(2), количество(2), CRC(2)
			if (UartBuf->N != QtyByte) {
				UartBuf->N = 0;
				Step = Pack;
				return (WrongAnswer);
			} else {
				UartBuf->N = 0;
				Step = CRCCheck;
			}	
			break;
			
		case CRCCheck:
			CRCpack = (uint16_t)UartBuf->Buf[QtyByte-1] * 256 + UartBuf->Buf[QtyByte-2];
			if (CRCpack != crc16(&(UartBuf->Buf[0]), QtyByte-2)) {
				Step = Pack;
				return (WrongAnswer);			
			} else {
				Step = Unpack;
			}	
			break;
			
		case Unpack:
			if (BitIsSet(UartBuf->Buf[1],7)) {	//спецификация Модбас, признак ошибки
				Step = Pack;
				return (UartBuf->Buf[1] % 2);				
			} else {
				Step = Pack;
				return (FuncDoneNoErr);
			}	
			break;
		} //switch
	} //while	
} //enum MBMFuncOut MBM03


//это я нашел на просторах инета, для атмеги
/*
uint16_t crc16(uint8_t *p, uint8_t len) 
{	
	uint16_t crc;
	uint8_t n;
	if (len > 256U) return (0);
	n = (unsigned char)len;
	crc = 0xffff;
	do {
		crc = _crc16_update(crc, *p++);  // uint16_t _crc16_update(uint16_t __crc, uint8_t __data)
	}
	while (--n);                         // pass through message buffer (max 256 items)
	return (crc);
}
*/

