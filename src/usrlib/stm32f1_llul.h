/**
 * low layer user library
 * определение структур регистров как битовых полей
 * инлайн функции работы с регистрами
 */
#include "stm32f10x.h"
#include <stdbool.h>

#ifndef STM32F1_LLUL_H
#define STM32F1_LLUL_H

#define SET_MASK(REG, MASK)		((REG) |= (MASK))
#define CLEAR_MASK(REG, MASK)	((REG) &= ~(MASK))

void makeDebugVar (void);


/******************************************************
    RCC
******************************************************/
typedef struct {
    // Bit 0 HSION: HSI clock enable
    __IO uint32_t HSION  :1;
    // Bit 1 HSIRDY: HSI clock ready flag
    __IO uint32_t HSIRDY :1;
    // Bit 2 Reserved, must be kept at reset value.
    __IO uint32_t dcb1   :1;
    // Bits 7:3 HSITRIM[4:0]: HSI clock trimming
    __IO uint32_t HSITRIM    :5;
    // Bits 15:8 HSICAL[7:0]: HSI clock calibration
    __IO uint32_t HSICAL     :8;
    // Bit 16 HSEON: HSE clock enable
    __IO uint32_t HSEON      :1;
    // Bit 17 HSERDY: HSE clock ready flag
    __IO uint32_t HSERDY     :1;
    // Bit 18 HSEBYP: HSE crystal oscillator bypass
    __IO uint32_t HSEBYP     :1;
    // Bit 19 CSSON: Clock security system enable
    __IO uint32_t CSSON      :1;
    // Bits 23:20 Reserved, must be kept at reset value.
    __IO uint32_t dcb2       :4;
    // Bit 24 PLLON: PLL enable
    __IO uint32_t PLLON      :1;
    // Bit 25 PLLRDY: PLL clock ready flag
    __IO uint32_t PLLRDY     :1;
    // Bits 31:26 Reserved, must be kept at reset value.
    __IO uint32_t dcb3       :6;
} RCC_CR_t;

inline void RCC_HSEon (void)
{
	volatile RCC_CR_t* RCC_CR = (RCC_CR_t*)(&(RCC->CR));
	RCC_CR->HSEON = 1;
}

inline void RCC_WaitHSEready (void)
{
	volatile RCC_CR_t* RCC_CR = (RCC_CR_t*)(&(RCC->CR));
    while (RCC_CR->HSERDY != 1) { }
}

inline void RCC_PLLon (void)
{
	volatile RCC_CR_t* RCC_CR = (RCC_CR_t*)(&(RCC->CR));
	RCC_CR->PLLON = 1;
}

inline void RCC_WaitPLLready (void)
{
	volatile RCC_CR_t* RCC_CR = (RCC_CR_t*)(&(RCC->CR));
	while (RCC_CR->PLLRDY != 1) { }
}

typedef enum {
    SW_HSI = 0b00,
    SW_HSE = 0b01,
    SW_PLL = 0b10
} SystemClock_t;
typedef enum {
    AHBnotdiv   = 0b0000,
    AHBdiv2     = 0b1000,
    AHBdiv4     = 0b1001,
    AHBdiv8     = 0b1010,
    AHBdiv16    = 0b1011,
    AHBdiv64    = 0b1100,
    AHBdiv256   = 0b1101,
    AHBdiv512   = 0b1111
} AHBprescaller_t;
typedef enum {
    APBnotdiv   = 0b000,
    APBdiv2     = 0b100,
    APBdiv4     = 0b101,
    APBdiv8     = 0b110,
    APBdiv16    = 0b111
} APBprescaller_t;
typedef enum {
    sHSIdiv2    = 0b0,
    sHSE        = 0b1
} PLLsource_t;
typedef enum {
    x2      = 0b0000,
    x3      = 0b0001,
    x4      = 0b0010,
    x5      = 0b0011,
    x6      = 0b0100,
    x7      = 0b0101,
    x8      = 0b0110,
    x9      = 0b0111,
    x10     = 0b1000,
    x11     = 0b1001,
    x12     = 0b1010,
    x13     = 0b1011,
    x14     = 0b1100,
    x15     = 0b1101,
    x16     = 0b1110
} PLLmull_t;

typedef struct {
    // Bits 1:0 SW[1:0]: System clock switch
    __IO SystemClock_t SW	:2;
    // Bits 3:2 SWS[1:0]: System clock switch status
    __IO SystemClock_t SWS 	:2;
    // Bits 7:4 HPRE[3:0]: HCLK prescaler
    __IO AHBprescaller_t HPRE	:4;
    // Bits 10:8 PPRE[2:0]: PCLK prescaler
    __IO APBprescaller_t PPRE	:3;
    // Bits 13:11 Reserved, must be kept at reset value.
    __IO uint32_t dcb1		:3;
    // Bit 14 ADCPRE: ADC prescaler
    __IO uint32_t ADCPRE 	:1;
    // Bit 15 Reserved, must be kept at reset value.
    __IO uint32_t dcb2       :1;
    // Bit 16 PLLSRC: PLL entry clock source
    __IO PLLsource_t PLLSRC	:1;
    // Bit 17 PLLXTPRE: HSE divider for PLL input clock
    __IO uint32_t PLLXTPRE	:1;
    // Bits 21:18 PLLMUL[3:0]: PLL multiplication factor
    __IO PLLmull_t PLLMULL	:4;
    // Bits 23:22 Reserved, must be kept at reset value.
    __IO uint32_t dcb3		:2;
    // Bits 27:24 MCO[3:0]: Microcontroller clock output
    __IO uint32_t MCO 		:4;
    // Bits 30:28 MCOPRE[2:0]: Microcontroller Clock Output Prescaler 
    __IO uint32_t MCOPRE		:3;
    // Bit 31 PLLNODIV: PLL clock not divided for MCO (not available on STM32F030x8 devices)
    __IO uint32_t PLLNODIV	:1;
} RCC_CFGR_t;

inline void RCC_SystemClockSwitch (SystemClock_t sw)
{
    volatile RCC_CFGR_t* RCC_CFGR = (RCC_CFGR_t*)(&(RCC->CFGR));
    RCC_CFGR->SW = sw;
}
inline void RCC_SetAHBprescaler (AHBprescaller_t prescaller)
{
    volatile RCC_CFGR_t* RCC_CFGR = (RCC_CFGR_t*)(&(RCC->CFGR));
    RCC_CFGR->HPRE = prescaller;
}
inline void RCC_SetAPBprescaler (APBprescaller_t prescaller)
{
    volatile RCC_CFGR_t* RCC_CFGR = (RCC_CFGR_t*)(&(RCC->CFGR));
    RCC_CFGR->PPRE = prescaller;
}
inline void RCC_SetPLLsource (PLLsource_t s)
{
    volatile RCC_CFGR_t* RCC_CFGR = (RCC_CFGR_t*)(&(RCC->CFGR));
    RCC_CFGR->PLLSRC = s;
}
inline void RCC_SetPLLmultiple (PLLmull_t x)
{
    volatile RCC_CFGR_t* RCC_CFGR = (RCC_CFGR_t*)(&(RCC->CFGR));
    RCC_CFGR->PLLMULL = x;
}



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