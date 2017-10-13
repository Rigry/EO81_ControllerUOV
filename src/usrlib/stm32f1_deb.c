#include "stm32f1_llul.h"

/******************************************************

		RCC

******************************************************/
typedef struct
{
  __IO RCC_CR_t CR;
  __IO RCC_CFGR_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
  __IO uint32_t AHBRSTR;
  __IO uint32_t CFGR2;
  __IO uint32_t CFGR3;
  __IO uint32_t CR2;
} RCC_t;
volatile RCC_t* RCC_bf = (RCC_t*)RCC;


void makeDebugVar (void)
{
  RCC_bf->CR.HSION = 1;
/*  FLASH_ACR->dcb1 = 0;
  FLASH_CR->dcb1 = 0;
  FLASH_SR->dcb1 = 0;
  USART1_bf->CR1.UE = 0;
  GPIOA_bf->MODER.MODER0 = 0;
  DMA_CH2_bf->CCR.EN = 0;
  DMA_CH3_bf->CCR.EN = 0;
  DMA1_ISR->GIF1 = 0;
*/ 
/*	GPIOA_MODER->MODER0 = 0;
	GPIOA_OTYPER->OT0 = 0;
	GPIOB_MODER->MODER0 = 0;
	SysTick_CTRL->CLKSOURCE = 0;
	*SysTick_Val = 0;
  *SysTick_LOAD = 0;
*/
}