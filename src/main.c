#include "func.h"       // объявления функций, чтоб не засорять
#include "device_config.h"
#include "display.h"
#include "menu.h"
#include "keyboard.h"
#include "timer.h"
#include "MBMaster.h"	//для модбаса
#include "MBSlave.h"	//для модбаса
#include "eeprom.h"     //для модбаса




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
uint16_t hourcounter[TOTAL_LAMPS];//счетчик наработки в часах
//uint16_t oncounter;//счетчик включений
uint8_t BUF1[70];//Буфер для приема и передачи сообщений по USART3
//Буфер приемника команд управления контроллером от компьютера
uint8_t CMD_BUF_RX[255];//командный буфер от управляющего RS-485
uint8_t BuffTxd[255];
//Буфер передатчика данных на компьютер
uint8_t CMD_BUF_TX[255];
uint8_t BuffRxd[255];
//Флаг поступления запроса от компьютера
volatile bool RxCMDReady=false;
volatile uint8_t return_counter;//счетчик для возврата из меню
volatile uint8_t uf_threshold;
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
uint16_t BadLamps[EXP_BOARD+1];	//флаги неработающих ламп (+1 на случай, если EXP_BOARD = 0, значение [0] для базовой платы)
uint16_t WorkCount[EXP_BOARD+1][10];	//наработка ламп [0][] - для базовой платы
uint16_t LampsQty[EXP_BOARD+1];

// Слейв
struct UartBufSt mbSlaveUART;
struct MBRegSt mbSlave;
uint8_t mbSlaveAdr;

// флэш имитация еепром 					 
volatile struct EEPROMst eeprom;

// Значение 100% УФ			
uint16_t UF100Percent;


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
    USART_MODE_RX();	// RS на прием (внутренний)
    USART1_RX_DMA_Init((uint32_t)mbSlaveUART.Buf, UART_BUF_SIZE); 
    USART1_TX_DMA_Init((uint32_t)mbSlaveUART.Buf);    
    CMD_MODE_RX();		// Интерфейс от компьютера на прием (внешний)

    StartSlaveDMA_USART_RX();	//включаем на прием канал ДМА (внешний)
    
    ALARM_LED_SET; US_LED_SET; UV_LED_SET; //проверка испарвности индикаторов
    UV_OFF; US_OFF;
    delay_func_init( );			//инициализируем таймер задержек для дисплея BUSY флаг не опрашиваем
    lcd_init();					//Инициализируем дисплей
    lcd_clear();				//Очищаем дисплей
    // Выставляем флаг обновления дисплея-вывести главный дисплей при запуске
    DeviceState.flags.disp_red = 1; 	
    ALARM_LED_RESET; US_LED_RESET; UV_LED_RESET;
 
    temperatura = get_temp();   //получаем температуру воды
    uf_level = get_uf_level();  //получаем уровень УФ
    spi_init();
    get_pars();					
    
    // значения в еепром по умолчанию
    if (!EEPROMRead(&eeprom)) {
        eeprom.DevN     = 0;
        eeprom.mbadr    = 1;
        eeprom.uartset  = 0;
        eeprom.UFmax    = UF100PERCENT_BEGIN;
    }

    MBslaveInit();

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
                sec++;// тикаем час
            } else {
                sec=0;
                hourWorked = true;
            }
        } 

        DeviceState.flags.disp_upd = 1;
    } //if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
} //void RTC_IRQHandler(void)//секундное прерывание

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
    if (USART1->SR&USART_SR_IDLE) {
        // ТУТ ВСЁ ПЕРЕДЕЛАТЬ
        USART1->DR;//сбрасываем флаг IDLE последовательным чтением рег.Статуса и рег.данных
        DMA1_Channel5->CNDTR;
        flags.RxDataReady = 1;
        StartSlaveDMA_USART_RX();
        __NOP();
    }  
    
    // а этот иф странный, наверно удалить надо
    if(USART1->SR&USART_SR_TC) {
        USART1->SR &= ~USART_SR_TC;
        CMD_MODE_RX();//интерфейс на прием 
        __NOP();
    }  
}

// конец передачи пакета модбас слейв (внешний)
void DMA1_Channel4_IRQHandler (void) 
{
    //ЕСЛИ ПРОШЛА ПЕРЕДАЧА В КАНАЛЕ ТО НАДО ПЕРЕКЛЮЧИТЬ ПЕРЕДАТЧИК
    if(DMA1->ISR & DMA_ISR_TCIF4)
    { }             
    //???? ???????? ???????? ??????
    if(DMA1->ISR & DMA_ISR_HTIF4)
    { }             //???-?? ??????
    //???? ????????? ?????? ??? ??????
    if(DMA1->ISR & DMA_ISR_TEIF4) { }             //???-?? ??????
    
    DMA1->IFCR |= DMA_IFCR_CGIF4;                 //??????? ?????? GIF, TEIF, HTIF ? TCIF    
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
        return  ( lamps & LAMPS_MASK[LAMPS_INST-1] );
    } else { 
        return BadLamps[board_num];
    }
}

uint8_t get_lamps_count(uint8_t plata)
{
    uint8_t lamps_count = 0;
    DeviceState.flags.alarm_comm = 0;
    if (plata == 0) { // адрес базовой платы
        lamps_count = LAMPS_INST;
    } else if (plata <= EXP_BOARD) { 
        lamps_count = LampsQty[plata];	
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
    if (MBUfLevel > UF100Percent) {
        UF100Percent = MBUfLevel;
        return (100);
    } else {
        uint32_t tmp32 = MBUfLevel;
        return (tmp32 * 100 / UF100Percent);
    }	
} 


void save_pars(void)
{
    volatile uint8_t i;
    volatile uint16_t addr;
    addr = 0;
    
    __disable_irq();
        
    for(i = 0; i < TOTAL_LAMPS; i++) {
        EEWrite(addr, hourcounter[i] & 0xFF );//LSB writing
        addr++; 
        EEWrite(addr, (hourcounter[i]>>8) & 0xFF);//MSB writing
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
    EEWrite(0x110, uf_threshold);
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
    
    for(i = 0; i < TOTAL_LAMPS; i++) {
        hourcounter[i] = EERead(addr);//LSB writing
        addr++;
        hourcounter[i] |= (EERead(addr) << 8);//MSB writing
        addr++;
    }
    
    DeviceLog.oncounter=EERead(0x100);       //OnCNTR LSB
    DeviceLog.oncounter|=(EERead(0x101)<<8); //OnCNTR MSB
    DeviceLog.res_all=EERead(0x102);
    DeviceLog.res_all|=(EERead(0x103)<<8);
    DeviceLog.res_one=EERead(0x104);
    DeviceLog.res_one|=(EERead(0x105)<<8);
    DeviceLog.res_log=EERead(0x106);
    DeviceLog.res_log|=(EERead(0x107)<<8);
    uf_threshold=EERead(0x110); //порог срабатывания тревоги уф
    
    if (UF100Percent == 0) {
        UF100Percent = EERead (0x300);
        UF100Percent |=  EERead (0x301) << 8;
        if (UF100Percent < UF100PERCENT_BEGIN || UF100Percent == 0xFF) {
            UF100Percent = UF100PERCENT_BEGIN;
        }
    }
    
    __enable_irq();		
} //void get_pars()
 


void PACK_SEND_CMD(uint8_t *BUF, uint8_t count)
{
    //uint8_t TEMP[255];//выходной обработанный массив
    uint8_t i,j;
    for(i = 255; i > 0; i--) BuffTxd[i] = 0;
    j = 1;
    for (i = 0; i < count;i++) {
        if (BUF[i]==SOH || BUF[i]==ETX || BUF[i]==DLE) { //экранируем байты содержащие специальные символы
            BuffTxd[j]=DLE;j++;BuffTxd[j]=~BUF[i];j++;
        } else {
            BuffTxd[j]=BUF[i];      // или просто копируем в буфер
            j++;
        }
    }
    BuffTxd[0]=SOH; BuffTxd[j]=ETX;
    StartSlaveDMA_USART_TX(j+1);    // хуячим по ДМА
}
 

 
void UNPACK(uint8_t *BUF)//функция распаковки пакета На выходе- ADDR KOP BYTE1...BYTEn
{
    uint8_t i,j;
    i=0;//индекс массива
    j=0;//Индекс преобразователь 
    while(BUF[j]!=ETX) {	
        if (BUF[j]==DLE) { //если попался экр.символ-удаляем и на его место-следующий байт инвертировав
            BUF[i]=~BUF[++j];
            j++;i++;
        } else {
            BUF[i]=BUF[j];  // или просто копируем в буфер
            j++;i++;
        }
    }
}

void UNPACK_RECEIVE(unsigned char *BUFRX)
{
    // ЭТО ВСЁ ЗАМЕНЯЕТЬССЯ НА модбас слейв
    unsigned char i,j;
    //?????? ???????? ???????
    for(i = 255; i > 0; i--) {
        CMD_BUF_RX[i]=0;//очищаем приемный буфер
    }	
    
    i=0;
    for(i = 0; BUFRX[i]!=SOH; i++) {
        if (i==255) {
          flags.RxDataReady=0;     //??? ?????? ??? ???????, ?.?. ?? ?????????? ????????? ??????
          flags.RxPacketErr=1;     //?????? ?????? ?????? ??????
          return;
        }
    }
    
    i++;
    j=0;
    //????????? ?????? ?????? ?? ?????????? ???????
    for (;BUFRX[i]!=ETX;i++) {
        if (BUFRX[i]==DLE) { //????????????? ????? ?????????? ??????????? ???????
            i=i+1;
            CMD_BUF_RX[j]=~BUFRX[i];j++;
        } else {
            CMD_BUF_RX[j]=BUFRX[i];  // ??? ?????? ???????? ? ?????
            j++;
        }
        if ((i==255)&&(BUFRX[i]!=ETX)) {
            flags.RxDataReady=0;     //??? ?????? ??? ???????, ?.?. ?? ?????????? ???????? ??????
            flags.RxPacketErr=1;     //?????? ?????? ?????? ??????
            return;
        }
    }
         
    flags.RxDataReady=1;     //??? ?????? ??? ???????, ?.?. ?? ?????????? ???????? ??????
    flags.RxPacketErr=0;     //?????? ?????? ?????? ??????
    RxCMDReady=true;
}




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
            } else {
                eSt = ExpRead;
                ExpNeedWrite = false;	//после чтения необходима запись (нет)
                ExpTrasN++;
            }			
        break;
        
        case SensTrans:
            eTmp = MBM03 (
                            UF_T_ADDR,          // адрес устройства
                            0,                  // адрес регистра
                            2,                  // количество регистров
                            MBBuf,              // сюда приходят данные
                            &(mbMasterUART),    // структура уарта
                            MBFunc              // таймер для таймаута
                         );
            if (eTmp == FuncDoneNoErr) {
                MBUfLevel = MBBuf[0];
                MBTemperature = MBBuf[1];
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
                UV_OFF  ;
                DeviceState.flags.uf_on=0;
                DeviceState.flags.disp_upd=1;
            /*	/////////  Посылаем сообщение УФ ВЫКЛ /////
                BUF1[0]=0x0B;     //адрес широковещательный
                BUF1[1]=UFOFST;   //Сообщение УФ выключен
                PACK_SEND(BUF1,2);//запаковать и послать
            */	///////////////////////////////////////////
            } else {
                UV_LED_SET;
                UV_ON;
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
                US_OFF;
                DeviceState.flags.uzg_on=0;
                DeviceState.flags.disp_upd=1;
            } else {
                US_LED_SET;
                US_ON;
                DeviceState.flags.uzg_on=1; 
                DeviceState.flags.disp_upd=1;
            }
        break;
                    
        case ENTER_HOLD: 		// 
            device_busy=true;
            display_menu();
            DeviceState.flags.disp_red=1;
            device_busy=false;
        break;
    } // switch KEYBOARD end
}

void MBSlaveAction (void)
{

    // дк всё это заменить на модбас слейв

    // надо среагировать на включение/выключение УФ УЗ

}



void WorkCheckAndLeds (void)
{
    if (DeviceState.flags.uf_on) {
        lamps = 0;
        for(uint8_t i = 0; i <= EXP_BOARD; i++) {
            // получаем список ламп подключенных в цикле по или суммируем от плат расширения и затем проверяем
            lamps|=get_lamps(i);
        }
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
        if (UF_T_BOARD)	{	
            //проверяем уровень УФ	
            uf_level=get_uf_level();	//получаем уровень УФ	
            if (uf_level>100) {
                uf_level=100;	//Запрещаем значения УФ>100%
            }	
            if  (uf_level<uf_threshold) { //если уровень УФ меньше порогового уровня 
                if (!(DeviceState.flags.alarm_uf)) { //если тревога не установлена -устанавливаем
                    //если низкий уровень УФ- зажигаем тревогу
                    DeviceState.flags.alarm_uf=1;
                    DeviceState.flags.disp_upd=1;
                    ALARM_LED_SET;
                }
            } else { // если уровень УФ выше порогового на 5 процентов
                if (DeviceState.flags.alarm_uf && (uf_level > (uf_threshold+5)) ) { //если установлен флаг Тревога Ур.УФ-гасим его
                    DeviceState.flags.alarm_uf=0;		//  
                    DeviceState.flags.disp_upd=1;		//  
                }
            }
            temperatura=get_temp();//получаем температуру воды				 
            //проверяем уровень температуры				 
            if (temperatura>UF_OFF_TEMP) { //если температура больше порога срабатывания защиты
                if (!DeviceState.flags.alarm_temp) { //если тревога не установлена 
                    //  зажигаем тревогу
                    DeviceState.flags.alarm_temp=1;// устанавливаем  флаг температуры
                    DeviceState.flags.disp_upd=1;
                    ALARM_LED_SET;
                }
                //выключаем ультрафиолет	 
                UV_LED_RESET;
                UV_OFF  ;
                //Выключаем ультразвук
                US_LED_RESET;
                US_OFF;
                //DeviceState.flags.uf_on=0;
                DeviceState.flags.disp_upd=1;
                /////////  Посылаем сообщение УФ ВЫКЛ /////
                //BUF1[0]=0x0B;     //адрес широковещательный
                //BUF1[1]=UFOFST;   //Сообщение УФ включен
                //PACK_SEND(BUF1,2);//запаковать и послать
                ///////////////////////////////////////////
            } else { //если температура меньше порога аварии
                //если установлен флаг Авария по температуре и температура опустилась ниже нижней границы отключения уф
                if ((DeviceState.flags.alarm_temp)&&(temperatura<UF_ON_TEMP)) {
                    DeviceState.flags.alarm_temp=0;
                    DeviceState.flags.disp_upd=1;
                    UV_LED_SET;
                    UV_ON;
                    if (DeviceState.flags.uzg_on==1) { //Если УЗГ был включен с кнопки,тогда надо восттановить 
                        US_LED_SET;//Включить светодиод УЗГ
                        US_ON;//Включить реле УЗГ
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
        if (UF_T_BOARD) {
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
    uartset_t* set = (uartset_t*) &(eeprom.uartset);
    USART1_Init(*set);

    mbSlave.RegOut[Dev] = DEVUNIQNUMBER;
    mbSlave.RegOut[DevN] = eeprom.DevN;
    mbSlave.RegOut[uartset] = eeprom.uartset;
    mbSlave.RegOut[mbadr] = mbSlaveAdr;
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
    

}