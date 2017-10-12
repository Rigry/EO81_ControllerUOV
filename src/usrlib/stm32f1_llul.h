/**
 * low layer user library
 * определение структур регистров как битовых полей
 * инлайн функции работы с регистрами
 */
#include "stm32f10x.h"

#ifndef STM32F1_LLUL_H
#define STM32F1_LLUL_H

#define SET_MASK(REG, MASK)		((REG) |= (MASK))
#define CLEAR_MASK(REG, MASK)	((REG) &= ~(MASK))

#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)

inline void FLASHUnlock (void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}
inline void FLASHSetProgrammMode (void)
{
    FLASH->CR = 0;
    SET_MASK (FLASH->CR, FLASH_CR_PG);
}
inline void	FLASHStartWrite (uint32_t addr, uint16_t data)
{
    *(__IO uint16_t*)(addr) = data;
}
inline bool FLASHEndProgramm (void)
{
    return (FLASH->SR & FLASH_SR_EOP);
}
inline void FLASHLock (void)
{
    SET_MASK (FLASH->CR, FLASH_CR_LOCK);
}
inline void FLASHSetErrasePageMode (void)
{
    FLASH->CR = 0;
    SET_MASK (FLASH->CR, FLASH_CR_PER);
}
inline void FLASHStartErasePage (uint32_t addr)
{
    FLASH->AR = addr;
    SET_MASK (FLASH->CR, FLASH_CR_STRT);
}
inline void FLASHResetEndProgrammFlag (void)
{
    SET_MASK (FLASH->SR, FLASH_SR_EOP);
}
// заодно сбрасывает флаг
inline bool FLASHProgrammingError (void)
{
    if (FLASH->SR & FLASH_SR_PGERR) {
        FLASH->SR |= FLASH_SR_PGERR;
        return true;
    } else {
        return false;
    }

}

#endif // STM32F1_LLUL_H