#include <stdint.h>
#include <stdbool.h>

#ifndef DEFINES_H_
#define DEFINES_H_

#define MB_TIMEOUT 200
#define F_CPU   72000000
//#define TIMER_TICK  		1000*72-1	// 1000 мкс для 72МГц


#define UART_BUF_SIZE 255
//#define QTY_IN_REG	1
//#define QTY_OUT_REG	2

#define	SetBit(reg, bit)		reg |= (1<<bit)
#define	ClearBit(reg, bit)		reg &= (~(1<<bit))
#define	InvBit(reg, bit)        reg ^= (1<<bit)
#define	BitIsSet(reg, bit)      ((reg & (1<<bit)) != 0)
#define	BitIsClear(reg, bit)    ((reg & (1<<bit)) == 0)



struct UartBufSt {					//структура работы с УАРТ на уровне отделенном от работы с регистрами
	uint8_t Buf[UART_BUF_SIZE];		//буффер куда приходят, откуда уходят данные
	uint8_t	N;						//количество переданных, принятых данных
	uint8_t NeedSend;				//количество байт, которыйе необходимо передать из Buf,
									//число отличное от нуля, переводит в режим передачи функцией UARTStartByRec()
	bool MBEnd;					//признак конца пакета для модбаса
};

// Адреса регистров модбаса
// MBInRegE изменяются извне функцией 03
enum MBInRegE {	
    uartsetset 	= 0,	// когда все нули 9600-8-N-1
    mbadrset	= 1,	// адрес (от 1 - 255)
    psswd		= 2,	// пароль для изменения следующего регистра
    DevNset		= 3,	// установка заводского номера устройства
    Tmaxset     = 4,    // Верхний предел температуры
    UFminset    = 5,    // Нижний предел уровня ультрафиолета
    LampsQtyset = 6,    // Количество ламп
    UFmaxset    = 7,    // Максимальное значение с датчика за всё время работы
    Hours1set   = 8,    // Часы наработки лампы 1
                        // далее остальные лампы до 112
    QTY_IN_REG  = 121   
};
// MBOutRegE читаются извне функцией 16
enum MBOutRegE {
    Dev			= 0,	// код типа устройства
    DevN		= 1,	// заводской номер устройства
    uartset 	= 2,    // когда все нули 9600-8-N-1
    mbadr 		= 3,    // адрес (от 1 - 255)
    workFlags   = 4,
    curTemp     = 5,    // температура
    Tmax        = 6,    // Верхний предел температуры
    UFlev       = 7,    // Уровень УФ
    UFmin       = 8,    // Нижний предел уровня ультрафиолета
    LampsQtyMB  = 9,    // Количество ламп (LampsQty уже занято)
    UFmax       = 10,   // Максимальное значение с датчика за всё время работы
    FlagLamps1  = 11,   // Флаги состояния ламп (1 группа)
    FlagLamps2  = 12,
    FlagLamps3  = 13, 
    FlagLamps4  = 14, 
    FlagLamps5  = 15, 
    FlagLamps6  = 16, 
    FlagLamps7  = 17, 
    Hours1      = 18,   // Часы наработки лампы 1
                        // далее остальные лампы до 112
    QTY_OUT_REG = 131
};
// MBCtrlRegE управление извне функцией 05
enum MBCtrlRegE {
    US  = 0,
    UF  = 1,
    QTY_CTRL_REG,
};

// структура данных, хранящейся в еепром
struct EEPROMst{
	uint16_t 	DevN;	    // зав номер устройства
	uint16_t 	uartset;
	uint16_t 	mbadr;
	uint16_t	UFmax;      // Максимальное значение с датчика за всё время работы 
};

// таймеры
typedef enum {
    MBFunc		= 0,
    MBDelay 	= 1,
    EndMesMBM	= 2,
    test,
    QtyTimers	
} eTimer_t;

#define FLASH_PAGE_ADR 0x08007C00

#define  UV_LED_SET			GPIOC->BSRR = GPIO_Pin_6
#define  UV_LED_RESET		GPIOC->BRR = GPIO_Pin_6			//перенес сюда для отладки

#define  ALARM_LED_SET	GPIOC->BSRR = GPIO_Pin_7
#define  ALARM_LED_RESET	GPIOC->BRR = GPIO_Pin_7

#define  US_LED_SET		GPIOB->BSRR = GPIO_Pin_12
#define  US_LED_RESET	GPIOB->BRR = GPIO_Pin_12

#define  US_ON	    	GPIOB->BSRR = GPIO_Pin_11
#define	 US_OFF 		GPIOB->BRR = GPIO_Pin_11

#define  UV_ON			GPIOB->BSRR = GPIO_Pin_10
#define	 UV_OFF 		GPIOB->BRR = GPIO_Pin_10

#define  CS_HIGH 		GPIOA->BSRR=GPIO_Pin_11
#define  CS_LOW			GPIOA->BRR=GPIO_Pin_11



#endif /* DEFINES_H_ */
