/*
 * MBMaster.h
 *
 * Created: 22.05.2017
 *  Author: dvk
 *
 *	в macros.h должны быть определены MB_TIMEOUT - таймаут в мс
 *	#define MB_TIMEOUT		200 		//таймаут в мс
 *	#define TIMER_TICK  	1000*72-1	//1000 мкс для 72МГц
 *
 */ 

#include <stdint.h>
//#include <util/crc16.h>	// для AVR
#include <stdbool.h>
#include "defines.h"
#include "crc.h"



#ifndef MBMASTER_H_
#define MBMASTER_H_

	enum MBMFuncOut{		//функции возвращают
		FuncInWork	= 0,	//функция в работе
		FuncErr 	= 1,	//из спецификации на модбас
		RegErr		= 2,	//из спецификации на модбас
		ValueErr	= 3,	//из спецификации на модбас
		WrongAnswer	= 253,	//битый пакет
		TimeoutErr	= 254,	
		FuncDoneNoErr = 255	//всё норм, можно работать дальше
	};

	/********************
	*	Таймер			*
	********************/
	//структура таймера (пока не создам отдельный хидер), логика, как в ST
	struct TimerSt {		//наименования из стиля Codesys
		bool IN;			//разрешение работы
		uint32_t PT;		//установленное время в мс
		uint32_t ET;		//прошедшее время в мс, сбрасывается En (обязателен вызов функции UpdateTimer), инкрементируется аппаратным таймером
		bool Q;				//флаг, что установленное время прошло
	};
	void InitTimer(void);					//инициализация аппаратного таймера для ARM
	void UpdateTimer(struct TimerSt *);		//обновление выхода (по времени или сброс En)
	void Tick(struct TimerSt *);			//инкремент аппаратным таймером	
	
	/*******************/
	
	//чтение регистров удаленного устройства (Modbus Master 03 function)
	enum MBMFuncOut MBM03					
		(uint8_t DevAddr,			//адрес устройства
		 uint16_t RegAddr,			//адрес первого регистра
		 uint16_t QtyReg,			//количество регистров
		 uint16_t Buf[],				//записываемые данные (массив)
		 struct UartBufSt *UartBuf,	//буфер уарта
		 struct TimerSt *Timer		//для таймаута
		);	

	//запись регистров удаленного устройства (Modbus Master 16 function)
	enum MBMFuncOut MBM16					
		(uint8_t DevAddr,			//адрес устройства
		 uint16_t RegAddr,			//адрес первого регистра
		 uint16_t QtyReg,			//количество регистров
		 uint16_t Buf[],				//записываемые данные (массив)
		 struct UartBufSt *UartBuf,	//буфер уарта
		 struct TimerSt *Timer		//для таймаута
		);		

	//эти функции типа приват, используються
//	uint16_t crc16(uint8_t *p, uint8_t len); 		//avr
	uint16_t crc16(uint8_t* data, uint8_t length);	//arm

#endif // MBMASTER_H_

