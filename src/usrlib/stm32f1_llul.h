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


//////////////////////////////////////////////////////////////////////////////
//    RCC
//////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////
//    USART
//////////////////////////////////////////////////////////////////////////////
typedef struct {
    // Bit 0 PE: Parity error
    volatile bool PE        :1;
    // Bit 1 FE: Framing error
    volatile bool FE        :1;
    // Bit 2 NE: Noise error flag
    volatile bool NE        :1;
    // Bit 3 ORE: Overrun error
    volatile bool ORE       :1;
    // Bit 4 IDLE: IDLE line detected
    volatile bool IDLE      :1;
    // Bit 5 RXNE: Read data register not empty
    volatile bool RXNE      :1;
    // Bit 6 TC: Transmission complete
    volatile bool TC        :1;
    // Bit 7 TXE: Transmit data register empty
    volatile bool TXE       :1;
    // Bit 8 LBD: LIN break detection flag
    volatile bool LBD       :1;
    // Bit 9 CTS: CTS flag
    volatile bool CTS       :1;
    // Bits 31:10 Reserved, forced by hardware to 0.
    volatile uint32_t dcb1  :22;
} USART_SR_t;

typedef enum {
    even = 0,
    odd
} PS_t;
typedef enum {
    bit8 = 0,
    bit9
} M_t;
typedef struct {
    // Bit 0 SBK: Send break
    volatile bool SBK       :1;
    // Bit 1 RWU: Receiver wakeup
    volatile bool RWU       :1;
    // Bit 2 RE: Receiver enable
    volatile bool RE        :1;
    // Bit 3 TE: Transmitter enable
    volatile bool TE        :1;
    // Bit 4 IDLEIE: IDLE interrupt enable
    volatile bool IDLEIE    :1;
    // Bit 5 RXNEIE: RXNE interrupt enable
    volatile bool RXNEIE    :1;
    // Bit 6 TCIE: Transmission complete interrupt enable
    volatile bool TCIE      :1;
    // Bit 7 TXEIE: TXE interrupt enable
    volatile bool TXEIE     :1;
    // Bit 8 PEIE: PE interrupt enable
    volatile bool PEIE      :1;
    // Bit 9 PS: Parity selection
    volatile PS_t PS        :1;
    // Bit 10 PCE: Parity control enable
    volatile bool PCE       :1;
    // Bit 11 WAKE: Wakeup method
    volatile uint32_t WAKE  :1;
    // Bit 12 M: Word length
    volatile M_t M          :1;
    // Bit 13 UE: USART enable
    volatile bool UE        :1;
    // Bits 31:14 Reserved, forced by hardware to 0.
    volatile uint32_t dcb1  :18;
} USART_CR1_t;

typedef enum {
    sb1     = 0,
    sbHalf, sb2, sb1andHalf 
} STOP_t;
typedef struct {
    // Bits 3:0 ADD[3:0]: Address of the USART node
    volatile uint32_t ADD   :4;
    // Bit 4 Reserved, forced by hardware to 0.
    volatile uint32_t dcb1  :1;
    // Bit 5 LBDL: lin break detection length
    volatile uint32_t LBDL  :1;
    // Bit 6 LBDIE: LIN break detection interrupt enable
    volatile bool LBDIE     :1;
    // Bit 7 Reserved, forced by hardware to 0.
    volatile uint32_t dcb2  :1;
    // Bit 8 LBCL: Last bit clock pulse
    volatile uint32_t LBCL  :1;
    // Bit 9 CPHA: Clock phase
    volatile uint32_t CPHA  :1;
    // Bit 10 CPOL: Clock polarity
    volatile uint32_t CPOL  :1;
    // Bit 11 CLKEN: Clock enable
    volatile bool CLKEN     :1;
    // Bits 13:12 STOP: STOP bits
    volatile STOP_t STOP    :2;
    // Bit 14 LINEN: LIN mode enable
    volatile bool LINEN     :1;
    // Bits 31:15 Reserved, forced by hardware to 0.
    volatile uint32_t dcb3  :17;
} USART_CR2_t;

typedef struct {
    // Bit 0 EIE: Error interrupt enable
    volatile bool EIE       :1;
    // Bit 1 IREN: IrDA mode enable
    volatile bool IREN      :1;
    // Bit 2 IRLP: IrDA low-power
    volatile uint32_t IRLP  :1;
    // Bit 3 HDSEL: Half-duplex selection
    volatile bool HDSEL     :1;
    // Bit 4 NACK: Smartcard NACK enable
    volatile bool NACK      :1;
    // Bit 5 SCEN: Smartcard mode enable
    volatile bool SCEN      :1;
    // Bit 6 DMAR: DMA enable receiver
    volatile bool DMAR      :1;
    // Bit 7 DMAT: DMA enable transmitter
    volatile bool DMAT      :1;
    // Bit 8 RTSE: RTS enable
    volatile bool RTSE      :1;
    // Bit 9 CTSE: CTS enable
    volatile bool CTSE      :1;
    // Bit 10 CTSIE: CTS interrupt enable
    volatile bool CTSIE     :1;
    // Bits 31:11 Reserved, forced by hardware to 0.
    volatile uint32_t dcb1  :21;
} USART_CR3_t;

typedef struct {
    // Bits 7:0 PSC[7:0]: Prescaler value
    volatile uint32_t PSC       :8;
    // Bits 15:8 GT[7:0]: Guard time value
    volatile uint32_t GT        :8;
    volatile uint32_t dcb1      :16;

} USART_GTPR_t;


//////////////////////////////////////////////////////////////////////////////
//    DMA
//////////////////////////////////////////////////////////////////////////////
typedef struct {
    uint32_t GIF1	:1; //Channel x global interrupt flag
    uint32_t TCIF1	:1; //Channel x transfer complete flag
    uint32_t HTIF1	:1;	//Channel x half transfer flag
    uint32_t TEIF1	:1;	//Channel x transfer error flag
    uint32_t GIF2	:1; //Channel x global interrupt flag
    uint32_t TCIF2	:1; //Channel x transfer complete flag
    uint32_t HTIF2	:1;	//Channel x half transfer flag
    uint32_t TEIF2	:1;	//Channel x transfer error flag
    uint32_t GIF3	:1; //Channel x global interrupt flag
    uint32_t TCIF3	:1; //Channel x transfer complete flag
    uint32_t HTIF3	:1;	//Channel x half transfer flag
    uint32_t TEIF3	:1;	//Channel x transfer error flag
    uint32_t GIF4	:1; //Channel x global interrupt flag
    uint32_t TCIF4	:1; //Channel x transfer complete flag
    uint32_t HTIF4	:1;	//Channel x half transfer flag
    uint32_t TEIF4	:1;	//Channel x transfer error flag
    uint32_t GIF5	:1; //Channel x global interrupt flag
    uint32_t TCIF5	:1; //Channel x transfer complete flag
    uint32_t HTIF5	:1;	//Channel x half transfer flag
    uint32_t TEIF5	:1;	//Channel x transfer error flag
    uint32_t GIF6	:1; //Channel x global interrupt flag
    uint32_t TCIF6	:1; //Channel x transfer complete flag
    uint32_t HTIF6	:1;	//Channel x half transfer flag
    uint32_t TEIF6	:1;	//Channel x transfer error flag
    uint32_t GIF7	:1; //Channel x global interrupt flag
    uint32_t TCIF7	:1; //Channel x transfer complete flag
    uint32_t HTIF7	:1;	//Channel x half transfer flag
    uint32_t TEIF7	:1;	//Channel x transfer error flag
    uint32_t dcb	:4;
} DMA_ISR_t;

typedef struct {
    uint32_t CGIF1	:1;	//Channel x global interrupt clear
    uint32_t CTCIF1	:1;	//Channel x transfer complete clear
    uint32_t CHTIF1	:1;	//Channel x half transfer clear
    uint32_t CTEIF1	:1;	//Channel x transfer error clear
    uint32_t CGIF2	:1;	//Channel x global interrupt clear
    uint32_t CTCIF2	:1;	//Channel x transfer complete clear
    uint32_t CHTIF2	:1;	//Channel x half transfer clear
    uint32_t CTEIF2	:1;	//Channel x transfer error clear
    uint32_t CGIF3	:1;	//Channel x global interrupt clear
    uint32_t CTCIF3	:1;	//Channel x transfer complete clear
    uint32_t CHTIF3	:1;	//Channel x half transfer clear
    uint32_t CTEIF3	:1;	//Channel x transfer error clear
    uint32_t CGIF4	:1;	//Channel x global interrupt clear
    uint32_t CTCIF4	:1;	//Channel x transfer complete clear
    uint32_t CHTIF4	:1;	//Channel x half transfer clear
    uint32_t CTEIF4	:1;	//Channel x transfer error clear
    uint32_t CGIF5	:1;	//Channel x global interrupt clear
    uint32_t CTCIF5	:1;	//Channel x transfer complete clear
    uint32_t CHTIF5	:1;	//Channel x half transfer clear
    uint32_t CTEIF5	:1;	//Channel x transfer error clear
    uint32_t CGIF6	:1;	//Channel x global interrupt clear
    uint32_t CTCIF6	:1;	//Channel x transfer complete clear
    uint32_t CHTIF6	:1;	//Channel x half transfer clear
    uint32_t CTEIF6	:1;	//Channel x transfer error clear
    uint32_t CGIF7	:1;	//Channel x global interrupt clear
    uint32_t CTCIF7	:1;	//Channel x transfer complete clear
    uint32_t CHTIF7	:1;	//Channel x half transfer clear
    uint32_t CTEIF7	:1;	//Channel x transfer error clear
    uint32_t dcb	:4;	//Bits 31:28 Reserved, must be kept at reset value.
} DMA_IFCR_t;
inline void DMAChannel2ClearTransferCompleteInterruptFlag (void)
{
    SET_MASK (DMA1->IFCR, DMA_IFCR_CTCIF2);
}
inline void DMAChannel3ClearTransferCompleteInterruptFlag (void)
{
    SET_MASK (DMA1->IFCR, DMA_IFCR_CTCIF3);
}
typedef enum {
    DirFromMemory		= 0b1,
    DirFromPeripheral	= 0b0
} eDMA_Direction;
typedef enum {
    Size8Bits	= 0b00,
    Size16Bits 	= 0b01,
    Size32Bits 	= 0b10
} eDMA_Size;
typedef struct {
    uint32_t EN		:1; //Bit 0 EN: Channel enable
    uint32_t TCIE	:1;	//Bit 1 TCIE: Transfer complete interrupt enable
    uint32_t HTIE	:1;	//Bit 2 HTIE: Half transfer interrupt enable
    uint32_t TEIE	:1;	//Bit 3 TEIE: Transfer error interrupt enable
    eDMA_Direction DIR	:1;	//Bit 4 DIR: Data transfer direction
    uint32_t CIRC	:1;	//Bit 5 CIRC: Circular mode
    bool PINC       :1;	//Bit 6 PINC: Peripheral increment mode
    bool MINC       :1;	//Bit 7 MINC: Memory increment mode
    eDMA_Size PSIZE	:2;	//Bits 9:8 PSIZE[1:0]: Peripheral size
    eDMA_Size MSIZE	:2;	//Bits 11:10 MSIZE[1:0]: Memory size
    uint32_t PL		:2;	//Bits 13:12 PL[1:0]: Channel priority level
    uint32_t MEM2MEM	:1;	//Bit 14 MEM2MEM: Memory to memory mode
    uint32_t dcb	:17;	//Bits 31:15 Reserved, must be kept at reset value.
} DMA_CCRx_t;
#define BIT_DMA_CCR_PSIZE	8
#define BIT_DMA_CCR_MSIZE	10
#define BIT_DMA_CCR_DIR		4

inline void DMASetPeripheralSize (DMA_Channel_TypeDef* DMA_Channel, eDMA_Size size)
{
    CLEAR_MASK  (DMA_Channel->CCR, (uint32_t)0b11 << BIT_DMA_CCR_PSIZE);
    SET_MASK	(DMA_Channel->CCR, (uint32_t)size << BIT_DMA_CCR_PSIZE);
}
inline void DMASetMemorySize (DMA_Channel_TypeDef* DMA_Channel, eDMA_Size size)
{
    CLEAR_MASK  (DMA_Channel->CCR, (uint32_t)0b11 << BIT_DMA_CCR_MSIZE);
    SET_MASK	(DMA_Channel->CCR, (uint32_t)size << BIT_DMA_CCR_MSIZE);
}

inline void DMASetDirection (DMA_Channel_TypeDef* DMA_Channel, eDMA_Direction dir)
{
    if (dir == DirFromMemory) {
        SET_MASK  (DMA_Channel->CCR, DMA_CCR1_DIR);
    } else {
        CLEAR_MASK  	(DMA_Channel->CCR, DMA_CCR1_DIR);
    }
}
inline void DMASetIncrementMemoryAddress (DMA_Channel_TypeDef* DMA_Channel)
{
    SET_MASK (DMA_Channel->CCR, DMA_CCR1_MINC);  
}
inline void DMASetCircularMode (DMA_Channel_TypeDef* DMA_Channel)
{
    SET_MASK (DMA_Channel->CCR, DMA_CCR1_CIRC); 
}
inline void DMAEnableInterruptTransferComplete (DMA_Channel_TypeDef* DMA_Channel)
{
    SET_MASK (DMA_Channel->CCR, DMA_CCR1_TCIE);
}

inline void DMASetPeripherAdress (DMA_Channel_TypeDef* DMA_Channel, uint32_t address)
{
    DMA_Channel->CPAR = address;
}
inline void DMASetMemoryAdress (DMA_Channel_TypeDef* DMA_Channel, uint32_t address)
{
    DMA_Channel->CMAR = address;
}
inline void DMASetQtyTransfer (DMA_Channel_TypeDef* DMA_Channel, uint16_t qty)
{
    DMA_Channel->CNDTR = qty;
}
inline uint16_t DMAGetQtyTransferLeft (DMA_Channel_TypeDef* DMA_Channel)
{
    return (DMA_Channel->CNDTR);
}
inline void DMAEnable (DMA_Channel_TypeDef* DMA_Channel)
{
    SET_MASK (DMA_Channel->CCR, DMA_CCR1_EN);
}
inline void DMADisable (DMA_Channel_TypeDef* DMA_Channel)
{
    CLEAR_MASK (DMA_Channel->CCR, DMA_CCR1_EN);
}


#endif // STM32F1_LLUL_H