#include "func.h"       // объявления функций, чтоб не засорять
#include "display.h"
#include "menu.h"
#include "keyboard.h"
#include "defines.h"    //дк
#include "timer.h"      //дк
#include "MBMaster.h"	//дк
#include "MBSlave.h"	//дк
#include "eeprom.h"     //дк




bool bTmp = false; 		//для отладки

// флаги для прерываний
bool hourWorked;

uint8_t kb,temperatura,uf_level;//device_status,
uint8_t ctr1=0;//счетчик посылаемых или принимаемых байт по USRAT3
//Счетчик байт пришедших от компа
uint8_t CMD_RX_CTR=0;
volatile bool RxDataReady=false;
volatile bool device_busy=false;
bool TxCompleted=false;
uint16_t lamps;
// счетчик наработки в часах
uint16_t hourcounter[MAX_LAMPS_QTY];
// счетчик для возврата из меню
volatile uint8_t return_counter;

union {
    struct Flags
    {
        uint8_t uf_on		:1;
        uint8_t uzg_on		:1;
        uint8_t alarm_uf	:1;
        uint8_t alarm_lamp 	:1;
        uint8_t alarm_temp 	:1;
        uint8_t alarm_comm 	:1;
        uint8_t disp_upd   	:1;
        uint8_t disp_red	:1;
    } flags;
    uint16_t val;
} DeviceState;	//структура описывает состояние установки

struct control_flags
{
   uint8_t TxDataReady	:1;
   uint8_t PuskOn		:1;
   uint8_t RxDataReady	:1;
   uint8_t RxPacketErr	:1;
   uint8_t  	  		:3;
   uint8_t              :1;
} flags ;

struct servicelog 
{
    uint16_t oncounter;	//количество включений установки
    uint16_t res_all;	//количество единичных сбросов
    uint16_t res_one;	//количество полных сбросов наработки
    uint16_t res_log;	//количество сбросов лога работы
} DeviceLog;	//структура содержит лог работы


/////////////////////////////////////////////////////////////
//  Денис добавил 					 
/////////////////////////////////////////////////////////////
// таймеры
// struct TimerSt Timer[QtyTimer];
// Мастер
struct UartBufSt mbMasterUART;
uint16_t MBUfLevel;
uint16_t MBTemperature;
uint16_t BadLamps[MAX_EXP_BOARD_QTY+1];	//флаги неработающих ламп (+1 на случай, если EXP_BOARD = 0, значение [0] для базовой платы)
uint16_t WorkCount[MAX_EXP_BOARD_QTY+1][10];	//наработка ламп [0][] - для базовой платы
uint16_t LampsQty[MAX_EXP_BOARD_QTY+1];

// Слейв
struct UartBufSt mbSlaveUART;
struct MBRegSt mbSlave;
uint8_t mbSlaveAdr;

// флэш имитация еепром 					 
volatile struct EEPROMst eeprom;

// Значение 100% УФ			
uint16_t UF100Percent;

uint8_t EXP_BOARD;

inline void US_ON (void) {
    GPIOB->BSRR = GPIO_Pin_11;
    US_LED_SET;
    SET_MASK (mbSlave.RegOut[workFlags], US_ON_WORKFLAGS_MASK);
}
inline void US_OFF (void) {
    GPIOB->BRR = GPIO_Pin_11; 
    US_LED_RESET; 
    CLEAR_MASK (mbSlave.RegOut[workFlags], US_ON_WORKFLAGS_MASK);
}
inline void UV_ON (void) {
    GPIOB->BSRR = GPIO_Pin_10; 
    UV_LED_SET; 
    SET_MASK (mbSlave.RegOut[workFlags], UV_ON_WORKFLAGS_MASK);
}
inline void UV_OFF (void) {
    GPIOB->BRR = GPIO_Pin_10;
    UV_LED_RESET;
    CLEAR_MASK (mbSlave.RegOut[workFlags], UV_ON_WORKFLAGS_MASK);
}

int main (void)
{

    makeDebugVar();

    // настройка тактирования на 72мГц
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH_SetLatency (2);
    RCC_HSEon ();
    RCC_WaitHSEready ();
    RCC_SetAHBprescaler (AHBnotdiv);
    RCC_SetAPBprescaler (APBdiv2);
    RCC_SetPLLmultiple (x9);
    RCC_SetPLLsource (sHSE);
    RCC_PLLon ();
    RCC_WaitPLLready ();
    RCC_SystemClockSwitch (SW_PLL);

    //Сбрасываем состояние установки
    DeviceState.val = 0;
 
    RTC_Config();		// настраиваем часы реального времени на прерывание в 1 секунду 
    init_ports();		// настраиваем порты
    UART3_Init();		// Инициализация RS канала (внутренний)

    
    ALARM_LED_SET; US_LED_SET; UV_LED_SET; //проверка испарвности индикаторов
    UV_OFF(); US_OFF();
    delay_func_init( );			//инициализируем таймер задержек для дисплея BUSY флаг не опрашиваем
    lcd_init();					//Инициализируем дисплей
    lcd_clear();				//Очищаем дисплей
    // Выставляем флаг обновления дисплея-вывести главный дисплей при запуске
    DeviceState.flags.disp_red = 1; 	
    ALARM_LED_RESET; US_LED_RESET; UV_LED_RESET;
 

    // значения в еепром по умолчанию
    if (!EEPROMRead(&eeprom)) {
        eeprom.DevN     = 0;
        eeprom.mbadr    = 1;
        eeprom.uartset.bits.parityEn  = false;
        eeprom.uartset.bits.parityEven = false;
        eeprom.uartset.bits.stopBitsMinus1 = 0;
        eeprom.uartset.bits.boud = bd9600;
        eeprom.Tmax     = 55;
        eeprom.UFmin    = 40;
        eeprom.LampsQty = 5;
        eeprom.UFmax    = UF100PERCENT_BEGIN;
        eeprom.Name     = 0;
        eeprom.Trec     = 20;
        eeprom.Sens.Board = true;
        eeprom.Sens.Temp  = true;
        eeprom.Sens.UV    = true;
    }
    EXP_BOARD = eeprom.LampsQty / 10;

    spi_init();
    get_pars();		

    MBslaveInit();


    USART_MODE_RX();	// RS на прием (внутренний)
    CMD_MODE_RX();		// Интерфейс от компьютера на прием (внешний)
    StartSlaveDMA_USART_RX();	//включаем на прием канал ДМА (внешний)

    // аппаратный на 1 мс
    TimerInit();

    TimerSetTime (EndMesMBM, 6); // 4700 мкс для 9600 и 72МГц округлил до большего плюс 1
    TimerSetTimeAndStart (MenuLedUpd, 200);
    
    __enable_irq();

    
    while(1) //Главный говноцикл
    {	
        MBMasterWork();
        
        if (mbMasterUART.NeedSend != 0) {
            PACK_SEND(mbMasterUART.Buf, mbMasterUART.NeedSend);
            mbMasterUART.NeedSend = 0;
        }			
        
        Keyboard();
        KeyboardAction();
        MBSlaveAction();
        
        if ( TimerEvent(MenuLedUpd) ) {
            WorkCheckAndLeds();
            Menu();
            EEPROMUpd(&eeprom);
        }

        if (hourWorked) {
            hourWorked = false;
            volatile uint16_t badlamp;
            volatile uint8_t lcount;
            for (uint8_t board = 0; board <= EXP_BOARD; board++) {
                badlamp = get_lamps(board);//получаем лампы подключенные к базовой плате
                lcount = get_lamps_count(board);
                for (uint8_t i = 0; i < lcount; i++) { // обходим все подключенные лампы
                    if( !(badlamp & 0x01) ) {	//если лампа подключена- 
                        hourcounter[board*10+i]++;
                    }
                    badlamp = (badlamp >> 1) & (LAMPS_MASK[lcount-1]); //массив нумеруется от нуля
                }
            }
            save_pars();
        }


    } // main loop end
}



///////////////////////////////////////////
//
//      прерывания
//
///////////////////////////////////////////

void RTC_IRQHandler(void)//секундное прерывание
{
    static uint16_t sec;
     
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET) {
        RTC_ClearITPendingBit(RTC_IT_SEC);
        RTC_WaitForLastTask();
        return_counter++;//счетчик для автоматического возврата из меню
        //ALARM_LED_SET
        if (DeviceState.flags.uf_on) {
            if (sec<3600) {
                sec++; // тикаем час
            } else {
                sec=0;
                hourWorked = true;
            }
        } 

        DeviceState.flags.disp_upd = 1;
    } 
} 

//ДК: прерывание по приему байта внутренний протокол
void USART3_IRQHandler(void)	
{
    TimerStop (EndMesMBM);
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        mbMasterUART.Buf[mbMasterUART.N] = (USART_ReceiveData(USART3) & 0xFF);
        mbMasterUART.N++;
        if (mbMasterUART.N >= UART_BUF_SIZE-1) {
            mbMasterUART.N = 0;
        }	
    }

    TimerStart(EndMesMBM);

    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    { }
}

// конец приёма пакет модбас слейва (внешний)
void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_IDLE) {
        // сбрасываем флаг IDLE последовательным чтением рег.Статуса и рег.данных
        USART1->DR;
        // пауза
        TimerStart (EndMesMBS);
        // запрет дма на приём
        StopSlaveDMA_USART_RX();
    }
}

// конец передачи пакета модбас слейв (внешний)
void DMA1_Channel4_IRQHandler (void) 
{
    StopSlaveDMA_USART_TX();
    StartSlaveDMA_USART_RX();
    mbSlaveUART.NeedSend = 0;
    mbSlaveUART.N = 0;
    // сброс всех флагов прерываний
    DMA1->IFCR |= DMA_IFCR_CGIF4;                 
}

void SysTick_Handler(void)
{
    TimerInc();
    TimersUpdate();

    if ( TimerEvent(EndMesMBM) ) {
        TimerPause(EndMesMBM);
        mbMasterUART.MBEnd = true;
    }
}


//////////////////////////////////////
//
//      Функции и подпрограммы
//
//////////////////////////////////////



uint16_t get_lamps(uint8_t board_num)
{
    uint16_t lamps = 0;
    DeviceState.flags.alarm_comm = 0;
    if (board_num == 0) { // базовая плата
        lamps = GPIOA->IDR & 0xFF;          //EPRA[7:0]
        lamps |= (GPIOC->IDR & 0x30) << 4;  //EPRA[9:8]
        //возвращаем состояние порта ламп по маске подключенных ламп
        uint8_t lamps_inst = 0;
        lamps_inst = eeprom.LampsQty;
        if (lamps_inst > 10) {
            lamps_inst = 10;
        }
        return  ( lamps & LAMPS_MASK[lamps_inst-1] );
    } else { 
        return BadLamps[board_num];
    }
}

uint8_t get_lamps_count(uint8_t plata)
{
    uint8_t lamps_count = 0;
    DeviceState.flags.alarm_comm = 0;
    if (plata == 0) {
        lamps_count = eeprom.LampsQty;
    } else if (eeprom.LampsQty > plata*10) {
        lamps_count = eeprom.LampsQty - plata*10;
    }
    if (lamps_count > 10) {
        lamps_count = 10;
    }

    return lamps_count;
}

inline uint8_t get_temp(void)
{
    return( (uint8_t)MBTemperature );
} 

inline uint8_t get_uf_level(void)
{
    //	Перевод в проценты
    if (MBUfLevel > eeprom.UFmax) {
        eeprom.UFmax = MBUfLevel;
        return (100);
    } else {
        uint32_t tmp32 = MBUfLevel;
        return (tmp32 * 100 / eeprom.UFmax);
    }
} 


void save_pars(void)
{
    volatile uint8_t i;
    volatile uint16_t addr;
    addr = 0;
    
    __disable_irq();
        
    for(i = 0; i < eeprom.LampsQty; i++) {
        uint16_t tmp16 = hourcounter[i] - 1;
        mbSlave.RegOut[Hours1+i] = hourcounter[i];
        EEWrite(addr, tmp16 & 0xFF );//LSB writing
        addr++;
        EEWrite(addr, (tmp16 >> 8) & 0xFF);//MSB writing
        addr++;
    }
         
    EEWrite(0x100, DeviceLog.oncounter);//OnCNTR LSB
    EEWrite(0x101, (DeviceLog.oncounter >> 8));//OnCNTR MSB
    EEWrite(0x102, DeviceLog.res_all);
    EEWrite(0x103, (DeviceLog.res_all >> 8));
    EEWrite(0x104, DeviceLog.res_one);
    EEWrite(0x105, (DeviceLog.res_one >> 8));
    EEWrite(0x106, DeviceLog.res_log);
    EEWrite(0x107, (DeviceLog.res_log >> 8));

    EEWrite (0x300, (uint8_t)UF100Percent);
    EEWrite (0x301, (uint8_t)(UF100Percent >> 8) );
    
    EEWRDI();
         
    __enable_irq();		
}

void get_pars()		//ДК: читает чтото (наработка ламп точно) из еепрома
{
    volatile uint8_t i;
    volatile uint16_t addr;
    addr=0;
    
    __disable_irq();
    
    for(i = 0; i < eeprom.LampsQty; i++) {
        hourcounter[i] = EERead(addr);//LSB writing
        addr++;
        hourcounter[i] |= (EERead(addr) << 8);//MSB writing
        addr++;
        hourcounter[i]++;
        mbSlave.RegOut[Hours1+i] = hourcounter[i];
    }
    
    DeviceLog.oncounter=EERead(0x100);       //OnCNTR LSB
    DeviceLog.oncounter|=(EERead(0x101)<<8); //OnCNTR MSB
    DeviceLog.res_all=EERead(0x102);
    DeviceLog.res_all|=(EERead(0x103)<<8);
    DeviceLog.res_one=EERead(0x104);
    DeviceLog.res_one|=(EERead(0x105)<<8);
    DeviceLog.res_log=EERead(0x106);
    DeviceLog.res_log|=(EERead(0x107)<<8);

    
    if (UF100Percent == 0) {
        UF100Percent = EERead (0x300);
        UF100Percent |=  EERead (0x301) << 8;
        if (UF100Percent < UF100PERCENT_BEGIN || UF100Percent == 0xFF) {
            UF100Percent = UF100PERCENT_BEGIN;
        }
    }
    
    __enable_irq();		
} //void get_pars()
 

void PACK_SEND(uint8_t *BUF, uint8_t count)
{
    volatile uint8_t Nsend = 0;
    USART_MODE_TX();
    __NOP();__NOP(); 
    do {
         while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
         USART_SendData(USART3,BUF[Nsend]);
    } while (++Nsend < count);
     
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
    
    USART_MODE_RX();				
}

void MBMasterWork(void)
{
    //реализация автоматом
    enum StE {			//состояния автомата
        GetOrder,		//определяем что опростить следующим
        SensTrans,		//опрос платы датчиков
        ExpRead,		//опрос платы расширения с адресом ExpTrasN
        ExpWrite,		//запись значений в плату расширения ExpTrasN
        Delay			//задержка между опросами
    };	
    static enum StE eSt = GetOrder;		//текущее состояние
    static uint8_t ExpTrasN = 0;		//количество опрошенных плат расширения
    static bool ExpNeedWrite = false;	//флаг необходимости записи после чтения для платы расширения
    uint16_t MBBuf[15];					//до 15 регистров опрос
    enum MBMFuncOut eTmp;
    eTmp = FuncInWork;
    
    
    switch (eSt) {
        
        case GetOrder:
            if (ExpTrasN == EXP_BOARD) {
                eSt = SensTrans;
                ExpTrasN = 0;
                // перевод из маски на 10 в маску на 16
                // подумать еще с установкой, тут явно неверно
                // перенести туда, когда пришёл норм ответ от платы расширения
/*                for (uint32_t i = 0; i < (eeprom.LampsQty - 10); i++) {
                    uint8_t board;
                    board = i / 10 + 1;
                    bool tmp;
                    tmp = BitIsClear(BadLamps[board], i % 10);
                    uint8_t reg;
                    reg = (i + 10) / 16;
                    SetBit(mbSlave.RegOut[reg + FlagLamps1], (uint16_t)tmp << ( (i + 10) % 16) );
                }*/

            } else {
                eSt = ExpRead;
                ExpNeedWrite = false;	//после чтения необходима запись (нет)
                ExpTrasN++;
            }			
        break;
        
        case SensTrans:
            eTmp = MBM03 (  UF_T_ADDR,          // адрес устройства
                            0,                  // адрес регистра
                            2,                  // количество регистров
                            MBBuf,              // сюда приходят данные
                            &(mbMasterUART),    // структура уарта
                            MBFunc              // таймер для таймаута
            );
            if (eTmp == FuncDoneNoErr) {
                MBUfLevel = MBBuf[0];
                MBTemperature = MBBuf[1];
                mbSlave.RegOut[curTemp] = MBTemperature;
                mbSlave.RegOut[UFlev] = get_uf_level();
                eSt = Delay;
            } 
            if (eTmp != FuncInWork) {
                eSt = Delay;
            }
        break;
            
        case Delay:
            TimerSetTimeAndStart (MBDelay, 50);
            if ( TimerEvent(MBDelay) ) {
                TimerPause (MBDelay);
                eSt = ExpNeedWrite ? ExpWrite : GetOrder;
            }
            break;
            
        case ExpRead:
            eTmp = MBM03 (ExpTrasN, 0, 2, MBBuf, &(mbMasterUART), MBFunc);
            if (eTmp == FuncDoneNoErr) {
                LampsQty[ExpTrasN] = MBBuf[0];
                BadLamps[ExpTrasN] = MBBuf[1];
    /*				for (i = 0; i < LampsQty[ExpTrasN]; i++) {
                    WorkCount[ExpTrasN][i] = MBBuf[i+2];
                }*/ //наработка считается на основной плате
            } 
            if (eTmp != FuncInWork) {
                eSt = Delay;
            }		
        break;
            
        case ExpWrite:	//в общем то в этом нет необходимости (потому убрал из алгоритма)
            MBBuf[0] = 0;
            if (DeviceState.flags.uf_on) {
                SetBit(MBBuf[0],0);
            }
            eTmp = MBM16 (ExpTrasN, 0, 1, MBBuf, &(mbMasterUART), MBFunc);
            if (eTmp != FuncInWork) {
                ExpNeedWrite = false;
                eSt = Delay;
            }
        break;
            
    } //	switch (eSt) 
} //void MBMWork(void)




void KeyboardAction (void)
{
    switch (kb) {
        case UP_PRESS: {}
        break;
        
        case DOWN_PRESS: {}
        break;
        
        case LEFT_HOLD:  // УФ
            if (DeviceState.flags.uf_on) {
                UV_LED_RESET;
                UV_OFF();
                DeviceState.flags.uf_on=0;
                DeviceState.flags.disp_upd=1;
            /*	/////////  Посылаем сообщение УФ ВЫКЛ /////
                BUF1[0]=0x0B;     //адрес широковещательный
                BUF1[1]=UFOFST;   //Сообщение УФ выключен
                PACK_SEND(BUF1,2);//запаковать и послать
            */	///////////////////////////////////////////
            } else {
                UV_LED_SET;
                UV_ON();
                DeviceState.flags.uf_on=1;
                DeviceState.flags.disp_upd=1;
                DeviceLog.oncounter++;
                save_pars();	//Сохранить параметр количества включений установки
            /*	/////////  Посылаем сообщение УФ ВКЛ ////////
                BUF1[0]=0x0B;//адрес широковещательный
                BUF1[1]=UFONST;		//Сообщение УФ включен
                PACK_SEND(BUF1,2);//запаковать и послать
            */	/////////////////////////////////////////////
            }
        break;
                    
        case RIGHT_HOLD:  // УЗГ
            if (DeviceState.flags.uzg_on) {
                US_LED_RESET;
                US_OFF();
                DeviceState.flags.uzg_on=0;
                DeviceState.flags.disp_upd=1;
            } else {
                US_LED_SET;
                US_ON();
                DeviceState.flags.uzg_on=1; 
                DeviceState.flags.disp_upd=1;
            }
        break;
                    
        case ENTER_HOLD: 		// 
            device_busy = true;
            display_menu();
            DeviceState.flags.disp_red = 1;
            device_busy = false;
        break;
    } // switch KEYBOARD end
}

void MBSlaveAction (void)
{
    if ( TimerEvent (EndMesMBS) ) {
        TimerStop (EndMesMBS);
        mbSlaveUART.MBEnd = true;
        mbSlaveUART.N = UARTGetQtyReceiveBytes ();
        MBSlave (&mbSlaveUART, &mbSlave, mbSlaveAdr);
        if (mbSlaveUART.NeedSend != 0) {
            // не знаю почему, но последние 2 байта не шлёт, потому +2
            StartSlaveDMA_USART_TX(mbSlaveUART.NeedSend + 2);
        } else {
            StartSlaveDMA_USART_RX();
        }
        mbSlaveUART.MBEnd = false;

        // дописать влияние модбаса на систему
        if (eeprom.uartset.val != mbSlave.RegIn[uartsetset]) {
            eeprom.uartset.val = mbSlave.RegIn[uartsetset];
        }
        
    #define EEPROM_NEW_NOT_0(InRegN, EepromMember) \
        if (mbSlave.RegIn[InRegN] != 0) { \
            EepromMember = mbSlave.RegIn[InRegN]; \
            mbSlave.RegIn[InRegN] = 0; \
        }
        EEPROM_NEW_NOT_0(mbadrset, eeprom.mbadr);
        EEPROM_NEW_NOT_0(Tmaxset, eeprom.Tmax);
        EEPROM_NEW_NOT_0(UFminset, eeprom.UFmin);
        EEPROM_NEW_NOT_0(LampsQtyset, eeprom.LampsQty);
        EEPROM_NEW_NOT_0(UFmaxset, eeprom.UFmax);

        if (mbSlave.RegCtrl[US] == NOT_0_AND_NOT_0xFF00) {
        } else if (mbSlave.RegCtrl[US] == 0) {
            US_OFF();
            DeviceState.flags.uzg_on = 0;
            mbSlave.RegCtrl[US] = NOT_0_AND_NOT_0xFF00;
        } else if (mbSlave.RegCtrl[US] == 0xFF00) {
            US_ON();
            DeviceState.flags.uzg_on = 1;
            mbSlave.RegCtrl[US] = NOT_0_AND_NOT_0xFF00;
        }
        if (mbSlave.RegCtrl[UV] == NOT_0_AND_NOT_0xFF00) {
        } else if (mbSlave.RegCtrl[UV] == 0) {
            UV_OFF();
            DeviceState.flags.uf_on = 0;
            mbSlave.RegCtrl[UV] = NOT_0_AND_NOT_0xFF00;
        } else if (mbSlave.RegCtrl[UV] == 0xFF00) {
            UV_ON();
            DeviceState.flags.uf_on = 1;
            mbSlave.RegCtrl[UV] = NOT_0_AND_NOT_0xFF00;
        }

    }
}



void WorkCheckAndLeds (void)
{
    lamps = 0;
    for(uint8_t i = 0; i <= EXP_BOARD; i++) {
        // получаем список ламп подключенных в цикле по или суммируем от плат расширения и затем проверяем
        lamps|=get_lamps(i);
        if (i == 0) {
            uint16_t tmp;
            tmp = ~lamps & 0b1111111111;
            mbSlave.RegOut[FlagLamps1] &= ~(0b1111111111);
            mbSlave.RegOut[FlagLamps1] |= tmp;
        }
    }
    if (DeviceState.flags.uf_on) {
        if (lamps!=0) { //если лампы не горят
            if (!(DeviceState.flags.alarm_lamp)) { //если тревога не установлена -устанавливаем
                ALARM_LED_SET; //сравнивать с 0x3ff - если хоть одна лампа не горит- зажигаем тревогу
                DeviceState.flags.alarm_lamp=1;// 
                DeviceState.flags.disp_upd=1;
            }
        } else {
            if(DeviceState.flags.alarm_lamp) { //если установлен флаг Тревога ламп-гасим его
                DeviceState.flags.alarm_lamp=0;
                DeviceState.flags.disp_upd=1;
            }
        }		
                        
        //если установлена плата датчиков температуры и УФ- проверяем уровни 		
        if (eeprom.Sens.Board)	{	
            //проверяем уровень УФ	
            uf_level=get_uf_level();	//получаем уровень УФ	
            if (uf_level>100) {
                uf_level=100;	//Запрещаем значения УФ>100%
            }	
            if  (uf_level<eeprom.UFmin) { //если уровень УФ меньше порогового уровня 
                if (!(DeviceState.flags.alarm_uf)) { //если тревога не установлена -устанавливаем
                    //если низкий уровень УФ- зажигаем тревогу
                    DeviceState.flags.alarm_uf=1;
                    DeviceState.flags.disp_upd=1;
                    ALARM_LED_SET;
                }
            } else { // если уровень УФ выше порогового на 5 процентов
                if (DeviceState.flags.alarm_uf && (uf_level > (eeprom.UFmin+5)) ) { //если установлен флаг Тревога Ур.УФ-гасим его
                    DeviceState.flags.alarm_uf=0;		//  
                    DeviceState.flags.disp_upd=1;		//  
                }
            }
            temperatura=get_temp();//получаем температуру воды				 
            //проверяем уровень температуры				 
            if (temperatura > eeprom.Tmax) { //если температура больше порога срабатывания защиты
                if (!DeviceState.flags.alarm_temp) { //если тревога не установлена 
                    //  зажигаем тревогу
                    DeviceState.flags.alarm_temp=1;// устанавливаем  флаг температуры
                    DeviceState.flags.disp_upd=1;
                    ALARM_LED_SET;
                }
                //выключаем ультрафиолет	 
                UV_LED_RESET;
                UV_OFF();
                //Выключаем ультразвук
                US_LED_RESET;
                US_OFF();
                //DeviceState.flags.uf_on=0;
                DeviceState.flags.disp_upd=1;
                /////////  Посылаем сообщение УФ ВЫКЛ /////
                //BUF1[0]=0x0B;     //адрес широковещательный
                //BUF1[1]=UFOFST;   //Сообщение УФ включен
                //PACK_SEND(BUF1,2);//запаковать и послать
                ///////////////////////////////////////////
            } else { //если температура меньше порога аварии
                //если установлен флаг Авария по температуре и температура опустилась ниже нижней границы отключения уф
                if (DeviceState.flags.alarm_temp && (temperatura < eeprom.Trec)) {
                    DeviceState.flags.alarm_temp=0;
                    DeviceState.flags.disp_upd=1;
                    UV_LED_SET;
                    UV_ON();
                    if (DeviceState.flags.uzg_on==1) { //Если УЗГ был включен с кнопки,тогда надо восттановить 
                        US_LED_SET;//Включить светодиод УЗГ
                        US_ON();//Включить реле УЗГ
                    }
                }
            }
        } //if (UF_T_BOARD)
                        
        // проверяем статус аварий и если аварий нет-гасим индикатор аварий
        if (!(DeviceState.flags.alarm_lamp | DeviceState.flags.alarm_uf | DeviceState.flags.alarm_temp)) {
            ALARM_LED_RESET;							
        } else {
            ALARM_LED_SET;
        }			
    } else { // if (DeviceState.flags.uf_on)
        ALARM_LED_RESET;
    }	 
}

void Menu (void)
{
    if (DeviceState.flags.disp_upd || DeviceState.flags.disp_red) { // если выставлен флаг обновления дисплея-обновляем дисплей
        if (eeprom.Sens.Board) {
            temperatura = get_temp();
            uf_level = get_uf_level();
            if (uf_level > 100) {
                uf_level = 100; 
            }	
        } //получаем уровень УФ и температуру
        display_main_screen();
        DeviceState.flags.disp_red = 0;
        DeviceState.flags.disp_upd = 0; // Сбрасываем флаг готовности обновления дисплея
    }
}


void MBslaveInit (void)
{
    
    // тут потом выбор с еепрома или заводские будет
    mbSlaveAdr = eeprom.mbadr;

    USART1_Init(eeprom.uartset);

    uint8_t msWait = (eeprom.uartset.bits.boud == bd9600)  ? 35000/9600  + 1 :
                     (eeprom.uartset.bits.boud == bd14400) ? 35000/14400 + 1 :
                     (eeprom.uartset.bits.boud == bd19200) ? 35000/19200 + 1 :
                                                             2; 
    // 3.5 слова или 2 мс ожидания после идле (которое 1 фрейм)
    TimerSetTime (EndMesMBS, msWait);

    mbSlave.RegOut[Dev] = DEVUNIQNUMBER;
    mbSlave.RegOut[DevN] = eeprom.DevN;
    mbSlave.RegOut[uartset] = (eeprom.uartset.val);
    mbSlave.RegOut[mbadr] = mbSlaveAdr;
    mbSlave.RegOut[Tmax] = eeprom.Tmax;
    mbSlave.RegOut[UFmin] = eeprom.UFmin;
    mbSlave.RegOut[LampsQtyMB] = eeprom.LampsQty;
    mbSlave.RegOut[UFmax] = eeprom.UFmax;

    // mbadrset может быть установлен в 0, потому для регистрации запоминаем
    mbSlave.RegIn[mbadrset] = mbSlave.RegOut[mbadr];
    mbSlave.RegInMinVal[mbadrset] = 1;
    for (uint8_t i = Tmaxset; i < QTY_IN_REG; i++) {
        mbSlave.RegInMinVal[i] = 1;
    }
    mbSlave.RegInMaxVal[uartsetset] = 0b111111;
    mbSlave.RegInMaxVal[mbadrset] = 255;
    mbSlave.RegInMaxVal[Tmaxset] = 100;
    mbSlave.RegInMaxVal[UFminset] = 100;
    mbSlave.RegInMaxVal[LampsQtyset] = 112;
    mbSlave.RegInMaxVal[UFmaxset] = 0x0FFF;

    // буду сравнивать с 0x0000 0xFF00 для определения запроса
    for (uint8_t i = 0; i < QTY_CTRL_REG; i++) {
        mbSlave.RegCtrl[i] = NOT_0_AND_NOT_0xFF00;
    }


    USART1_RX_DMA_Init((uint32_t)mbSlaveUART.Buf, UART_BUF_SIZE); 
    USART1_TX_DMA_Init((uint32_t)mbSlaveUART.Buf);
    StartSlaveDMA_USART_RX();
}