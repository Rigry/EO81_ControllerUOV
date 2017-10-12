#include "eeprom.h"

static const uint16_t QtyWords = QTY_WORD_IN_EEPROM_BUF;
static volatile FLASH_Page_EEPROM_t* FLASH_PAGE = ((FLASH_Page_EEPROM_t*)FLASH_PAGE_ADR);
//volatile FLASH_SR_type* FLASH_SR;

// текущий оффсет для определения еще не записыных ячеек памяти
// для записи новых данных
volatile uint16_t FLASHOffset = 0;

bool EEPROMRead (volatile struct EEPROMst* st)
{
	// обнуляем буфер перед заполнением
	for (uint16_t i = 0; i < QtyWords; i++) {
		EEPROMbuf[i] = 0xFFFF;
	}
	if (FLASH_PAGE->Page[0] == 0xFFFFFFFF) {
		//еще ничего не записано
		return (false);
	} else {
		// просматриваем весь флеш
		uint16_t MaxWordN = 0;
		for (uint16_t i = 0; i < 256; i++) {
			/**
			 * структура данных во флеше
			 * старшие 16 бит - идентификатор слова WordN
			 * младшие 16 бит - само слово
			 * копируем данные в буфер
			 */
			uint16_t WordN;
			WordN = FLASH_PAGE->NumData[i].num;
			if (WordN < QtyWords) {
				EEPROMbuf[WordN] = FLASH_PAGE->NumData[i].data;
				MaxWordN = (MaxWordN < WordN) ? WordN : MaxWordN;
			} else {
				FLASHOffset = i;
				if ( MaxWordN != (QtyWords - 1) ) {
					return (false);	//не все данные записаны
				}
				break;
			}
		}
		/**
		 * копируем данные в структуру
		 */
		uint16_t* ar = (uint16_t*)st;
		for (uint16_t i = 0; i < QtyWords; i++) {
			ar[i] = EEPROMbuf[i];
		}
		return (true);
	}
}

void EEPROMUpd (volatile struct EEPROMst *st)
{
	static uint16_t WordN = 0;
	uint16_t* ar = (uint16_t*)st;
	enum FuncStep {
		CheckChanges,
		DataStartWrite,
		DataCheckWrite,
		WordNStartWrite,
		WordNCheckWrite,
		ErrasePage,
		CheckErasePage
	};
	static enum FuncStep Step = CheckChanges;
	static uint32_t addr;
	static uint16_t data;

	switch (Step) {

	case CheckChanges:
		if (EEPROMbuf[WordN] == ar[WordN]) {
			WordN++;
			WordN = (WordN == QtyWords) ? 0 : WordN;
		} else {
			data = ar[WordN];
			Step = DataStartWrite;
		}
		break;

	case DataStartWrite:
		if (FLASHOffset == 256) {
			FLASHOffset = 0;
			Step = ErrasePage;
		} else if (FLASH_PAGE->NumData[FLASHOffset].data == 0xFFFF ) {
				FLASHUnlock ();
				FLASHSetProgrammMode ();
				addr = (uint32_t)((uint32_t*)FLASH_PAGE + FLASHOffset); 
				FLASHStartWrite (addr, data);
				Step = DataCheckWrite;
		} else {
			addr = (uint32_t)((uint32_t*)FLASH_PAGE + FLASHOffset);
			Step = WordNStartWrite;
		}
		break;

	case DataCheckWrite:
		if ( FLASHEndProgramm() ) {
			FLASHResetEndProgrammFlag();
			FLASHLock ();
			Step = WordNStartWrite;
		}
		break;
	
	case WordNStartWrite:
		if (FLASH_PAGE->NumData[FLASHOffset].num == 0xFFFF) {
			FLASHUnlock ();
			FLASHSetProgrammMode ();
			addr = (uint32_t)( (uint16_t*)addr + 1 );
			FLASHStartWrite (addr, WordN);
			Step = WordNCheckWrite;
		} else {			
			Step = ErrasePage;
		}
		break;

	case WordNCheckWrite:
		if ( FLASHEndProgramm() ) {
			FLASHResetEndProgrammFlag();
			if (FLASH_PAGE->NumData[FLASHOffset].num == WordN) {
				EEPROMbuf[WordN] = FLASH_PAGE->NumData[FLASHOffset].data;
			}
			FLASHOffset++;
			FLASHLock();
			Step = CheckChanges;
        }
		if ( FLASHProgrammingError() ) {
			Step = ErrasePage;
		}
		break;

	case ErrasePage:
		FLASHUnlock ();
		FLASHSetErrasePageMode();
		FLASHStartErasePage ( (uint32_t)FLASH_PAGE );
		Step = CheckErasePage;
		break;

	case CheckErasePage:
		if ( FLASHEndProgramm() ) {
			FLASHResetEndProgrammFlag();
			uint32_t tmp32 = 0xFFFFFFFF;
			for (uint16_t i = 0; i < 256; i++) {
				tmp32 = tmp32 & FLASH_PAGE->Page[i];
			}
			if (tmp32 == 0xFFFFFFFF) {
				FLASHLock ();
				// обнуляем буфер 
				for (uint16_t i = 0; i < QtyWords; i++) {
					EEPROMbuf[i] = 0xFFFF;
				}
				Step = CheckChanges;
				FLASHOffset = 0;
			} else {
				Step = ErrasePage;
			}
		}
		break;
	}
}



