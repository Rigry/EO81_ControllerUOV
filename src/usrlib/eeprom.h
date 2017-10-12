/**
 * ОСНОВНЫЕ МОМЕНТЫ:
 * Работа с еепром с помощью структуры struct EEPROMst,
 * которая должна быть создана в файле defines.h.
 * Состав структуры произвольный,
 * минимальный размер данных 16 бит (8 недопустимо)
 * При инициализации прочитать из памяти в экземпляр EEPROMst с помощью функции EEPROMRead().
 * Внутри программы работаем с экземпляром EEPROMst без каких либо ограничений.
 * Периодически вызываем функцию EEPROMUpd() для обновления изменившихся значений в памяти
 * 
 * Реализация для STM32 имитирует еепром, занимая страницу флэш памяти
 * адрес страницы FLASH_PAGE_ADR должен быть определен в defines.h
 *
 */
  
 
 
#include <stdbool.h>
#include "defines.h"
#include "stm32f1_llul.h"


#ifndef EEPROM_H_
#define EEPROM_H_

	typedef struct {
		uint16_t data;
		uint16_t num;
	} NumData_t;
	typedef union {
		volatile uint32_t Page[255];
		volatile NumData_t NumData[255];
	} FLASH_Page_EEPROM_t;

	

	// количество данных в 16 битном формате 
	#define QTY_WORD_IN_EEPROM_BUF		(sizeof(struct EEPROMst) / 2)
	

	// 	буфер для определения измененных данных
	// 	содержит записанные в еепром данные
	// 	обновляется при записи измененных данных
    // 	используется в функциях EEPROMRead() и EEPROMUpd()
    //  ПО МОЕМУ ЭТО У МЕНЯ ЛИШНЕЕ МОЖНО ЧИТАТЬ СРАЗУ ИЗ ПАМЯТИ
	static volatile uint16_t EEPROMbuf[QTY_WORD_IN_EEPROM_BUF];

	// читает структуру из еепром (используется только при инициализации)
    // возвращает false если в памяти пусто, или не все данные записаны
	bool EEPROMRead(volatile struct EEPROMst* st);

	// следит за обнавлением в памяти данных, если они изменились
    void EEPROMUpd(volatile struct EEPROMst* st);
    


#endif	//EEPROM_H_