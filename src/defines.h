#include <stdint.h>
#include <stdbool.h>

#ifndef DEFINES_H_
#define DEFINES_H_

#define MB_TIMEOUT 200
#define F_CPU   72000000UL
// для этой прошивки и этого устройства, согласно таблице кодов устройств ЭО-76
#define DEVUNIQNUMBER	4

#define VALID_RESET_CODE	1111
#define CONF_PASSWORD   /*0*/208
#define RETURN_FROM_MENU	10

//маска для проверки включенных ламп.
static const int  LAMPS_MASK[10] =
{
	0x01, 0x03,0x07,0x0f,0x1f,0x3f,0x7f,0xff,0x1ff,0x3ff
};

// временно 30 для отладки
#define UART_BUF_SIZE 255
// число не ноль и не 0xFF00
#define NOT_0_AND_NOT_0xFF00 0xFFFF

#define MAX_LAMPS_QTY       122
#define MAX_EXP_BOARD_QTY	12
#define UF100PERCENT_BEGIN	0x0300


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
    uartset 	= 2,    // когда все нули 9600-8-N-1, подробнее далее
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
#define US_ON_WORKFLAGS_POS     0
#define US_ON_WORKFLAGS_MASK    ((uint16_t)1 << US_ON_WORKFLAGS_POS)
#define UV_ON_WORKFLAGS_POS     1
#define UV_ON_WORKFLAGS_MASK    ((uint16_t)1 << UV_ON_WORKFLAGS_POS)

// MBCtrlRegE управление извне функцией 05
enum MBCtrlRegE {
    US  = 0,
    UV  = 1,
    QTY_CTRL_REG
};

// битовое поле uartset
typedef enum {
    bd9600   = 0b000,
    bd14400  = 0b001,
    bd19200  = 0b010,
    bd28800  = 0b011,
    bd38400  = 0b100,
    bd57600  = 0b101,
    bd76800  = 0b110,
    bd115200 = 0b111
} bode_t;
typedef union {
    struct Bits {
        bool    parityEn        : 1;
        bool    parityEven      : 1;
        uint16_t stopBitsMinus1 : 1;
        bode_t  boud            : 3;
        uint16_t dcb            :10;
    } bits;
    uint16_t val;
} uartset_t;

typedef struct {
    bool Board   :1;
    bool Temp    :1;
    bool UV      :1;
    uint16_t dcb :13;
} Exsist_t;

// структура данных, хранящейся в еепром
struct EEPROMst{
	uint16_t 	DevN;	    // зав номер устройства
	uartset_t 	uartset;
    uint16_t 	mbadr;
    uint16_t    Tmax;
    uint16_t    UFmin;
    uint16_t    LampsQty;
    uint16_t	UFmax;      // Максимальное значение с датчика за всё время работы 
    uint16_t    Name;
    uint16_t    Trec;       // температура восстановления
    Exsist_t    Sens;       // флаги наличия датчиков
    uint16_t    LampsQtyMain; // на самой плате
};



// таймеры
typedef enum {
    MBFunc		= 0,
    MBDelay 	= 1,
    EndMesMBM	= 2,
    MenuLedUpd,
    EndMesMBS,
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



#define  CS_HIGH 		GPIOA->BSRR=GPIO_Pin_11
#define  CS_LOW			GPIOA->BRR=GPIO_Pin_11



#endif /* DEFINES_H_ */
