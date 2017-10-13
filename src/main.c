#include <stdbool.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "device_config.h"
#include "display.h"
#include "menu.h"
#include "keyboard.h"
#include "communic.h"
#include "defines.h"	//для модбаса
#include "timer.h"
#include "MBMaster.h"	//для модбаса
#include "MBSlave.h"	//для модбаса
#include "eeprom.h"     //для модбаса
#include "func.h"       // объявления функций, чтоб не засорять



bool bTmp = false; 		//для отладки

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
volatile uint8_t sTimer=0;
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

// подпрограмма работы мастера модбаса
void MBMasterWork(void);

// Слейв
struct UartBufSt mbSlaveUART;
struct MBRegSt mb;

// флэш имитация еепром 					 
volatile struct EEPROMst eeprom;

// Значение 100% УФ			
uint16_t UF100Percent;


int main (void)
{
//    static uint8_t i; 

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
    USART1_Init();
    USART1_RX_DMA_Init();
    USART1_TX_DMA_Init();    
    CMD_MODE_RX();		// Интерфейс от компьютера на прием (внешний)

    StartDMAChannel5();	//включаем на прием канал ДМА (внешний)
    
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

    // аппаратный на 1 мс
    TimerInit();

    TimerSetTime (EndMesMBM, 6); // 4700 мкс для 9600 и 72МГц округлил до большего плюс 1
    
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

                                
        /////////////////////////////////////////////////////////////////////////////////////////				
        //Проверяем включена ли установка и функционирует ли она
        //////////////////////////////////////////////////////////////////////////////////////////
        if (DeviceState.flags.uf_on && (sTimer == 150)) { //если включена УФ
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
            sTimer=0;
        } // if (DeviceState.flags.uf_on && (sTimer == 150))
         
        if (!(DeviceState.flags.uf_on)) {
            ALARM_LED_RESET;//если установка выключена- выключаем тревогу
        }	 
        sTimer++;
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //Конец проверки работоспособности установки
        /////////////////////////////////////////////////////////////////////////////////////////////////	 

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
    volatile uint16_t badlamp;
    volatile uint8_t i,board,lcount;
     
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET) {
        RTC_ClearITPendingBit(RTC_IT_SEC);
        RTC_WaitForLastTask();
        return_counter++;//счетчик для автоматического возврата из меню
        //ALARM_LED_SET
        //ПРОВЕРЯЕМ ЗАНЯТОСТЬ УСТАНОВКИ-если выставлен флаг занятости и пришла команда-то отправляем назад бизи
        if ((device_busy==true) && (RxCMDReady==true)) {
            //адрес нашей установки
            CMD_BUF_TX[0]=BASE_ADDR;
            //КОП Занятости установки
            CMD_BUF_TX[1]=BUSY;
            //Запаковать и послать ответ на пришедшую команду
            PACK_SEND_CMD(CMD_BUF_TX,2);
            RxCMDReady=false;//сбросить флаг
        }
        if (DeviceState.flags.uf_on) {
            if (sec<3600) {
                sec++;// если час не натикал прибавляем секунды
            } else {
                sec=0;//начинаем считать час по новой
                for (board = 0; board <= EXP_BOARD; board++) {
                    badlamp=get_lamps(board);//получаем лампы подключенные к базовой плате
                    lcount=get_lamps_count(board);
                    for (i = board*10; i < (board*10+lcount); i++) { // обходим все подключенные лампы
                        if(!(badlamp&0x01)) {	//если лампа подключена- 
                            hourcounter[i]+=1;// прибавляем счетчик
                        }
                        badlamp= (badlamp>>1) & (LAMPS_MASK[lcount-1]);//массив нумеруется от нуля
                    }
                }
                save_pars();
            }
        } //if (DeviceState.flags.uf_on)
                
        //if(DeviceState.flags.alarm_comm) {					
        //	////////////////////////////////////////////////////////////////////////////	
        //	BUF1[0]=0;//Посылаем пустой пакет для обнуления буфера плат расширения
        //	BUF1[1]=0;		// на том конце после инициализации должен обяз.-но придти ETX и буфер сбросится
        //	PACK_SEND(BUF1,2);//запаковать и послать
        //	////////////////////////////////////////////////////////////////////////////	
        //}
        
        DeviceState.flags.disp_upd=1;
    } //if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
} //void RTC_IRQHandler(void)//секундное прерывание


void USART3_IRQHandler(void)	//ДК: прерывание по приему байта
{
//	STOP_TIMER;
    TimerStop (EndMesMBM);
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        mbMasterUART.Buf[mbMasterUART.N] = (USART_ReceiveData(USART3) & 0xFF);
        mbMasterUART.N++;
        if (mbMasterUART.N >= UART_BUF_SIZE-1) {
            mbMasterUART.N = 0;
        }	
/*		if (mbMasterUART.Buf[0] == board_addr) {
            GPIOB->BSRR = GPIO_Pin_15;	//Включаем Индикатор активности на шине
        }*/
    }

    TimerStart(EndMesMBM);

    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    { }
}

void SysTick_Handler(void)	//Modbus
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
//
//
//
//////////////////////////////////////

void init_ports()
{
    GPIO_InitTypeDef  GPIO_InitStructure;//наша инит структура
/////////////////////////////////////////////////////////////////////////////////////////////////
//  ПОРТЫ и ПИНЫ ДЛЯ ДИСПЛЕЯ																																	 //
/////////////////////////////////////////////////////////////////////////////////////////////////
    /* GPIOB, GPIOD and AFIO clocks enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//фулл жЫтаг наfig,шьемся по SWD(освобождаем PA15,PB3,PB4)

    /* Configure DB4-DB7 lines for LCD */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
  
    /* Configure E line for LCD */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Configure R/W line for LCD */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    /* Configure RS line for LCD */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
 
/////////////////////////////////////////////////////////////////////////////////////////////
// ПОРТ  КЛАВИАТУРЫ
/////////////////////////////////////////////////////////////////////////////////////////////
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
     
/////////////////////////////////////////////////////////////////////////////////////////////
// ПОРТЫ НАГРУЗОК И ИНДИКАТОРОВ
/////////////////////////////////////////////////////////////////////////////////////////////
    /* PB10-реле УФ, PB11-реле УЗГ, PB12-LED1-ИНДИКАТОР УЗГ ВКЛЮЧЕН*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
     
//  PC6-LED2-ИНДИКАТОР УФ      PC7-LED3-ИНДИКАТОР АВАРИЯ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
/////////////////////////////////////////////////////////////////////////////////////////////
// ВХОДЫ ОТ ЭПРА
/////////////////////////////////////////////////////////////////////////////////////////////	 
//EPRA[7:0]
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
//EPRA[9:8]	 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

} //void init_ports()

uint16_t get_lamps(uint8_t board_num)
{
    uint16_t lamps=0;
//	volatile uint32_t delaycnt=0x1FFFF;
    DeviceState.flags.alarm_comm=0;
    if (board_num == 0) { //если запрашиваем базовую плату - читаем порт ламп
        lamps=GPIOA->IDR&0xFF;//EPRA[7:0]
        lamps|=(GPIOC->IDR&0x30)<<4;//EPRA[9:8]
        return  (lamps&(LAMPS_MASK[LAMPS_INST-1]));//возвращаем состояние порта ламп по маске подключенных ламп
    } else { 
        return BadLamps[board_num];
    }
} //uint16_t get_lamps(uint8_t board_num)

uint8_t get_lamps_count(uint8_t plata)
{
    uint8_t lamps_count=0;
//	uint32_t delaycnt=0x1FFFF;
    DeviceState.flags.alarm_comm=0;
    if(plata==0) { //если адрес-это адрес базовой платы
        lamps_count=LAMPS_INST;
    } else if (plata<=EXP_BOARD) { 
        lamps_count = LampsQty[plata];	
    }
    return lamps_count;
} //uint8_t get_lamps_count(uint8_t plata)

uint8_t get_temp(void)
{
    return( (uint8_t)MBTemperature );
} 

uint8_t get_uf_level(void)
{
    /**
     *	Перевод в проценты
     */
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
    addr=0;
    
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
} //void save_pars(void)

void get_pars()		//ДК: читает чтото (наработка ламп точно) из еепрома
{
    volatile uint8_t i;
    volatile uint16_t addr;
    addr=0;
    
    __disable_irq();
    
    for(i=0;i<TOTAL_LAMPS;i++) {
        // while ((CHECK_BSY()&0x01));
        hourcounter[i]=EERead(addr);//LSB writing
        //while ((CHECK_BSY()&0x01));
        addr++;
        hourcounter[i]|=(EERead(addr)<<8);//MSB writing
        //while ((CHECK_BSY()&0x01));
        addr++;
    }
    
    DeviceLog.oncounter=EERead(0x100);//OnCNTR LSB
    //while ((CHECK_BSY()&0x01));
    DeviceLog.oncounter|=(EERead(0x101)<<8);//OnCNTR MSB
    //while ((CHECK_BSY()&0x01));
    DeviceLog.res_all=EERead(0x102);
    //while ((CHECK_BSY()&0x01));
    DeviceLog.res_all|=(EERead(0x103)<<8);
    //while ((CHECK_BSY()&0x01));
    DeviceLog.res_one=EERead(0x104);
    //while ((CHECK_BSY()&0x01));
    DeviceLog.res_one|=(EERead(0x105)<<8);
    //while ((CHECK_BSY()&0x01));
    DeviceLog.res_log=EERead(0x106);
    //while ((CHECK_BSY()&0x01));
    DeviceLog.res_log|=(EERead(0x107)<<8);
    //while ((CHECK_BSY()&0x01));
    uf_threshold=EERead(0x110);//порог срабатывания тревоги уф
    
    if (UF100Percent == 0) {
        UF100Percent = EERead (0x300);
        UF100Percent |=  EERead (0x301) << 8;
        UF100Percent = (UF100Percent < UF100PERCENT_BEGIN || UF100Percent == 0xFF)
                        ? UF100PERCENT_BEGIN : UF100Percent;
    }
    
    __enable_irq();		
} //void get_pars()
 
 void RTC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    PWR_BackupAccessCmd (ENABLE);
    RCC_LSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
    RCC_RTCCLKCmd(ENABLE);

    RTC_WaitForSynchro();
    RTC_WaitForLastTask();
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    RTC_WaitForLastTask();
    RTC_SetPrescaler(39999); /* RTC period = RTCCLK/RTC_PR = (40000Hz)/(39999+1) */
    RTC_WaitForLastTask();
}


void UART3_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    //Ремап пинов на PC10,PC11
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3 , ENABLE);
    
    /* Configure USART3 Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
  
    /* Configure USART3 Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    //конфигурим управляющий пин USART3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
    /* Configure USART3 */
    USART_Init(USART3, &USART_InitStructure);
    // Подключаем прерывание USART3 по готовности буфера приема
    NVIC_EnableIRQ(USART3_IRQn);
    USART_ITConfig  (USART3,USART_IT_RXNE, ENABLE); 

    /* Enable the USART3 */
    USART_Cmd(USART3, ENABLE);
}



void PACK_SEND_CMD(uint8_t *BUF, uint8_t count)
{
    //uint8_t TEMP[255];//выходной обработанный массив
    uint8_t i,j;
    for(i=255;i>0;i--) BuffTxd[i]=0;
    j=1;
    for (i=0;i<count;i++) {
        if (BUF[i]==SOH || BUF[i]==ETX || BUF[i]==DLE) { //экранируем байты содержащие специальные символы
            BuffTxd[j]=DLE;j++;BuffTxd[j]=~BUF[i];j++;
        } else {
            BuffTxd[j]=BUF[i];  // или просто копируем в буфер
            j++;
        }
    }
    BuffTxd[0]=SOH; BuffTxd[j]=ETX;
    StartDMAChannel4(j+1);//хуячим по ДМА
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

/////////////////////////////////////////////////////////////////////////////////
//Функции для работы с EEPROM AT25040SSHL-B
/////////////////////////////////////////////////////////////////////////////////
uint8_t EEWrite(uint16_t addr,uint8_t data)
{
    volatile uint8_t tmp;
    while ((CHECK_BSY()&0x01)) {}
    EEWREN();
    while ((CHECK_BSY()&0x01)) {}
    CS_LOW;
    __NOP();__NOP();__NOP();__NOP();__NOP();
    //шлем инструкцию
    tmp=(addr>>5) & 0x08;
    spi_send(tmp | 0x02);//шлем опкод на запись с установленным 9-м битом адреса
    spi_send(addr & 0xFF);//шлем адрес
    spi_read();
    spi_send(data);
    spi_read();
    while (SPI2->SR & SPI_SR_BSY) {} //ждем окончания передачи BSY=0 - SPI свободен.
    tmp = spi_read();//сбрасываем флаг RXNE
    CS_HIGH;
    while (CHECK_BSY() & 0x01) {}
    return 0;
}

uint8_t EERead(uint16_t addr)
{
    volatile uint8_t tmp;
    CS_LOW;
    __NOP();__NOP();__NOP();__NOP();__NOP();
    tmp=(addr>>5) & 0x08;//выделяем 9-й бит адреса;
    spi_send(tmp | 0x03);//шлем опкод на чтение с установленный 9-м битом адреса
    spi_send(addr & 0xff);// шлем остаток адреса
    tmp=spi_read();//считали хуйню
    spi_send(0x00);//послали хуйню
    tmp=spi_read();//считали хуйню
    while((SPI2->SR & SPI_SR_BSY)) {} //ждем окончания передачи BSY=0 - SPI свободен.
    tmp=spi_read();//принимаем TRUE данные:)
    CS_HIGH;
    return tmp;
}

void EEWREN(void)//разрешение записи в eeprom
{
    CS_LOW;
    __NOP();__NOP();__NOP();__NOP();__NOP();
    //ПОСЫЛАЕМ КОМАНДУ WREN
    spi_send(0x06);
    while((SPI2->SR & SPI_SR_BSY)) {} //ждем окончания передачи BSY=0 - SPI свободен.
    spi_read();//Сбрасываем приемный буфер
    CS_HIGH;
}

void EEWRDI(void)//запрещение записи в eeprom
{
    CS_LOW;
    __NOP();__NOP();__NOP();__NOP();__NOP();
    //ПОСЫЛАЕМ КОМАНДУ WREN
    spi_send(0x04);
    while((SPI2->SR & SPI_SR_BSY));//ждем окончания передачи BSY=0 - SPI свободен.
    spi_read();//Сбрасываем приемный буфер
    CS_HIGH;
}
uint8_t CHECK_BSY()// читаем регистр статуса
{
    volatile uint8_t tmp;
    CS_LOW;
    __NOP();__NOP();__NOP();__NOP();__NOP();
    //ПОСЫЛАЕМ КОМАНДУ RDSR
    spi_send(0x05);//RDSR command
    spi_send(0x00);
    spi_read();//читаем хуйню
    while(SPI2->SR & SPI_SR_BSY) {}//ждем окончания передачи BSY=0 - SPI свободен.
    tmp=spi_read();//принимаем TRUE данные:)
    CS_HIGH;
    return tmp;//выдаем состояние статуса памяти:0b X X X X BP1 BP0 WEN nRDY(nRDY=0-READY,1- BUSY;WEN=0-NOT WREN,1-WRENABLED)
}

void spi_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15  ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
//	SPI_INIT.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_32 ;
//	SPI_INIT.SPI_CPOL=SPI_CPOL_Low;
//	SPI_INIT.SPI_CPHA=SPI_CPHA_2Edge ;
//	SPI_INIT.SPI_DataSize=SPI_DataSize_16b;
//	SPI_INIT.SPI_Direction=SPI_Direction_1Line_Tx;
//	SPI_INIT.SPI_FirstBit=SPI_FirstBit_MSB;
//	SPI_INIT.SPI_Mode=SPI_Mode_Master;
//	SPI_INIT.SPI_NSS=SPI_NSS_Soft;// 
//	SPI_Init(SPI1,  &SPI_INIT);
//	SPI_Cmd(SPI1, ENABLE);
 

    
    CS_HIGH;
    //SPI1->CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_1); //Baud rate = Fpclk/16
    SPI2->CR1 |= (SPI_CR1_BR_1); //Baud rate = Fapb1/8= 36/8=4.5mhz
    SPI2->CR1 &= ~SPI_CR1_CPOL; //клок в idle state на нуле
    SPI2->CR1 &= ~SPI_CR1_CPHA; //по переднему фронту СЦК
    //SPI1->CR1 &=  ~SPI_CR1_CPHA; //по переднему фронту СЦК
    SPI2->CR1 &= ~SPI_CR1_DFF; //8 bit long
    SPI2->CR1 &= ~SPI_CR1_LSBFIRST; //MSB first
    //SPI1->CR1 |= SPI_CR1_LSBFIRST; //LSB first
    SPI2->CR1 |= SPI_CR1_SSM; //Выставляем ССМ для управления режимом мастер через программу
    SPI2->CR1 |= SPI_CR1_SSI; //выставляем ССИ для установки на сигнала NSSINT high-типа мы master 
    SPI2->CR2 &= ~SPI_CR2_SSOE; //Вывод NSS ?
    SPI2->CR1 |= SPI_CR1_MSTR; //  Master
    // SPI1->CR1 |= SPI_CR1_SPE; //ENABLE SPI1
    SPI2->CR1 |= SPI_CR1_SPE; //включить SPI2
}

void spi_send(uint8_t data)
{
    SPI2->DR = data;
    while(!(SPI2->SR & SPI_SR_TXE)) {}
    //SPI2->CR1 &= ~SPI_CR1_SPE; //выключить SPI2
}

uint8_t spi_read()
{
    while(!(SPI2->SR & SPI_SR_RXNE)) {}
    return ((SPI2->DR) & 0xFF);
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
            eTmp = MBM03 (UF_T_ADDR, 0, 2, MBBuf, &(mbMasterUART), MBFunc);
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





























//-------------------------------------------------------------------------
//Вставка для работы уартов через ДМА контроллер
void USART1_Init(void)
{
    //включение тактирования
    RCC->APB2ENR |=   RCC_APB2ENR_IOPAEN;                //???????????? GPIO
    RCC->APB2ENR |=   RCC_APB2ENR_AFIOEN;                //???????????? ?????????????? ??????? GPIO
    RCC->APB2ENR |=   RCC_APB2ENR_USART1EN;              //???????????? USART1
    //настраиваем ноги порта PORTA.9 для TX; PORTA.10 для RX 
    GPIOA->CRH   &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);   //??????????? MODE ? CNF
    GPIOA->CRH   |=   GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;  //??????????? ????? ? ?????????????? ?-??, 50MHz
    GPIOA->CRH   &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);   //??????????? MODE ? CNF
    GPIOA->CRH   |=   GPIO_CRH_CNF10_0;                   //????, ?????? ?????????
    // настраиваем скорость,число стоповых и бит данных
    // ДК: переделать в будущем на значения с еепрома
    USART1->BRR   =   0x1D4C;                            //скорость 9600 72000000/9600
    USART1->CR1  &=  ~USART_CR1_M;                       //8 бит данных
    USART1->CR2  &=  ~USART_CR2_STOP;                    //количество стоп бит:1
                            
    // 
    USART1->CR1  |=   USART_CR1_UE;                      //включить USART1
    USART1->CR1  |=   USART_CR1_TE;                      //включить передатчик
    USART1->CR1  |=   USART_CR1_RE;                      //включить приемник
    USART1->CR1  |=   USART_CR1_IDLEIE|USART_CR1_TCIE;    //включить прерывание по LINE IDLE  и окончанию передачи через усарт |USART_CR1_RXNEIE-было по кой то хуй
    NVIC_SetPriority (USART1_IRQn, 4);
    NVIC_EnableIRQ(USART1_IRQn);                         //Включаем прерывания в контроллере прерываний
        //конфигурим управляющий пин USART1
    //	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    //  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    //  GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIOA->CRH   &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);   //очищаем MODE ? CNF
    GPIOA->CRH   |=   GPIO_CRH_MODE8 ;                   //выход ППулл 50мгц GPIO
}


void USART1_TX_DMA_Init(void)
{
    // ДК: разрешение работы ДМА для УАРТ1 передачи
    USART1->CR3  |=  USART_CR3_DMAT;
    // ДК: разрешение тактирования
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    // ДК: адрес переферии - буфера
    DMA1_Channel4->CPAR  =  (uint32_t) &USART1->DR;
    DMA1_Channel4->CMAR  =  (uint32_t) mbSlaveUART.Buf;
    // ДК: количество байт на отправку буду указывать при отправке
    // DMA1_Channel4->CNDTR =  32;

    // ДК: какието нстройки (не трогаю)
    DMA1_Channel4->CCR   =  0;
    DMA1_Channel4->CCR  &= ~DMA_CCR4_CIRC;
    DMA1_Channel4->CCR  |=  DMA_CCR4_DIR;
    DMA1_Channel4->CCR  &= ~DMA_CCR4_PSIZE;
    DMA1_Channel4->CCR  &= ~DMA_CCR4_PINC;
    DMA1_Channel4->CCR  &= ~DMA_CCR4_MSIZE;
    DMA1_Channel4->CCR  |=  DMA_CCR4_MINC;

    //ПРЕРЫВАНИЕ ПО КОНЦУ ПЕРЕДАЧИ
    DMA1_Channel4->CCR  |=  DMA_CCR4_TCIE;
    NVIC_EnableIRQ (DMA1_Channel4_IRQn);
    
}
void USART1_RX_DMA_Init(void)
{
    // ДК: разрешение работы ДМА для УАРТ1 приёма
    USART1->CR3  |=  USART_CR3_DMAR;  
    // ДК: разрешение тактирования
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    // ДК: адрес переферии - буфера
    DMA1_Channel5->CPAR  =  (uint32_t) &USART1->DR;
    DMA1_Channel5->CMAR  =  (uint32_t) mbSlaveUART.Buf;
    DMA1_Channel5->CNDTR =  UART_BUF_SIZE;
    
    // ДК: какието нстройки (не трогаю)
    DMA1_Channel5->CCR   =  0;
    DMA1_Channel5->CCR  &= ~DMA_CCR5_CIRC;
    DMA1_Channel5->CCR  &= ~DMA_CCR5_DIR;
    DMA1_Channel5->CCR  &= ~DMA_CCR5_PSIZE;
    DMA1_Channel5->CCR  &= ~DMA_CCR5_PINC;
    DMA1_Channel5->CCR  &= ~DMA_CCR5_MSIZE;
    DMA1_Channel5->CCR  |=  DMA_CCR5_MINC;
}
  

// ПЕРЕДАЧА ДАННЫХ С КОНТРОЛЛЕРА В СЕТЬ
void StartDMAChannel4(unsigned int LengthBufer)
{
    CMD_MODE_TX();//включить на передачу интерфейс с компьютером
    __NOP();__NOP();
  DMA1_Channel4->CCR  &= ~DMA_CCR4_EN;          //????????? ?????? ??????
  DMA1_Channel4->CNDTR =  LengthBufer;          //????????? ?????????? ?????? ??? ??????
  DMA1->IFCR          |= DMA_IFCR_CTCIF4;       //???????? ???? ????????? ??????
  DMA1_Channel4->CCR  |=  DMA_CCR4_EN;          //????????? ?????? ??????
}
// ПРИЕМ ДАННЫХ В КОНТРОЛЛЕР ИЗ СЕТИ
void StartDMAChannel5 (void)
{
  DMA1_Channel5->CCR  &= ~DMA_CCR5_EN;          //????????? ?????? ??????
  DMA1_Channel5->CNDTR =  UART_BUF_SIZE;          //????????? ?????????? ?????? ??? ??????
  DMA1->IFCR          |= DMA_IFCR_CTCIF5;       //???????? ???? ????????? ??????
  DMA1_Channel5->CCR  |=  DMA_CCR5_EN;          //????????? ?????? ??????
}

void USART1_IRQHandler(void)
{
    
  if(USART1->SR&USART_SR_IDLE)
  {
    //uint32_t tmp;
    //tmp = 
    USART1->DR;//сбрасываем флаг IDLE последовательным чтением рег.Статуса и рег.данных
    //tmp = 
    DMA1_Channel5->CNDTR;
    //тут обрабатываем массив пришедший от компа
    UNPACK_RECEIVE(BuffRxd);
    //for(i=99;i>0;i--) BuffRxd[i]=0;
    flags.RxDataReady=1;
    StartDMAChannel5();
    
    __NOP();
  }  
    
    if(USART1->SR&USART_SR_TC)
  {
    USART1->SR&=~USART_SR_TC;//сбрасываем флаг IDLE последовательным чтением рег.Статуса и рег.данных
    CMD_MODE_RX();//интерфейс на прием 
    
    __NOP();
  }  
}
//ПРЕРЫВАНИЕ ПО КОНЦУ ПЕРЕДАЧИ
void DMA1_Channel4_IRQHandler (void) 
{
    
  //ЕСЛИ ПРОШЛА ПЕРЕДАЧА В КАНАЛЕ ТО НАДО ПЕРЕКЛЮЧИТЬ ПЕРЕДАТЧИК
  if(DMA1->ISR & DMA_ISR_TCIF4) 
        { 
            
        }             
    
  //???? ???????? ???????? ??????
  if(DMA1->ISR & DMA_ISR_HTIF4) { }             //???-?? ??????
    
  //???? ????????? ?????? ??? ??????
  if(DMA1->ISR & DMA_ISR_TEIF4) { }             //???-?? ??????
    
  DMA1->IFCR |= DMA_IFCR_CGIF4;                 //??????? ?????? GIF, TEIF, HTIF ? TCIF    
}




//----------------Конец вставки для работы через ДМА контроллер------------------

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
    uint16_t badlamps;
    uint8_t  count;

    // ИСПРАВИТЬ проверку адреса устройства!!!
    // Если пришла команда от компьютера - разбираем ее по частям
    // ДК: эту часть пока не трогаю
    // дк всё это заменить на модбас слейв
    if ( RxCMDReady && ( CMD_BUF_RX[0] == BASE_ADDR) ) {
        switch ( CMD_BUF_RX[1])	{
            case GETSTATE: { // Запрос состояния установки
                //адрес нашей установки
                CMD_BUF_TX[0]=BASE_ADDR;
                //КОП На который отвечаем
                CMD_BUF_TX[1]=GETSTATE;
                //Пакуем флаги состояния установки включен ли уф,узг,тревоги по уф,лампам,температуре и RS-485
                CMD_BUF_TX[2]=	(   (DeviceState.flags.uf_on<<7) 
                                    | (DeviceState.flags.uzg_on<<6) 
                                    | (DeviceState.flags.alarm_uf<<5) 
                                    | (DeviceState.flags.alarm_lamp<<4)
                                    | (DeviceState.flags.alarm_temp<<3) 
                                    | (DeviceState.flags.alarm_comm<<2)
                                ) & 0xFF;
                //Добавляем в пакет информацию об уровне ультрафиолета и температуре
                CMD_BUF_TX[3]=uf_level;
                CMD_BUF_TX[4]=temperatura;
                //Запаковать и послать
                PACK_SEND_CMD(CMD_BUF_TX,5);
            }
            break;
                                        
            case SETUFTHR: { // Установить порог тревоги по УФ
                //адрес нашей установки
                CMD_BUF_TX[0]=BASE_ADDR;
                //КОП На который отвечаем
                CMD_BUF_TX[1]=SETUFTHR;
                //Если укладываемся в разрешенные границы значения
                if ((CMD_BUF_RX[2]<99)&&(CMD_BUF_RX[2]>0)) {
                    //Сохранить новое значение порога аварии по уровню УФ
                    uf_threshold=CMD_BUF_RX[2];	save_pars();
                    CMD_BUF_TX[2]=0x0F;//Если операция успешная-посылаем 0x0F В ответ
                } else {
                    CMD_BUF_TX[2]=0x00;//Если значение неверное-шлем ноль
                }	
                //Запаковать и послать ответ на команду
                PACK_SEND_CMD(CMD_BUF_TX,3);
            }
            break;
                                        
            case GETUFTHR: {
                //адрес нашей установки
                CMD_BUF_TX[0]=BASE_ADDR;
                //КОП На который отвечаем
                CMD_BUF_TX[1]=GETUFTHR;
                CMD_BUF_TX[2]=uf_threshold;//Посылаем текущий порог тревоги по уровню уф
                //Запаковать и послать ответ на команду
                PACK_SEND_CMD(CMD_BUF_TX,3);
            }
            break;
                                        
            case SETUFON: { // Включить УФ
                UV_LED_SET;//Включить светодиод УФ
                UV_ON;//Включить реле УФ
                DeviceState.flags.uf_on=1;//Ставим флаг УФ вкл.
                DeviceState.flags.disp_upd=1;//Обновляем дисплей на установке
                DeviceLog.oncounter++;//Увеличиваем счетчик включений
                save_pars();//Сохранить параметр количества включений установки
            }
            break;
                                        
            case SETUFOF: { // Выключить УФ
                UV_LED_RESET;//выключить светодиод УФ
                UV_OFF  ; //Выключить реле УФ
                DeviceState.flags.uf_on=0;//Изменяем статус установки
                DeviceState.flags.disp_upd=1;//Обновить дисплей на установке
            }
            break;
                                        
            case SETUZON: { // Включить УЗ
                US_LED_SET;//Включить светодиод УЗГ
                US_ON;//Включить реле УЗГ
                DeviceState.flags.uzg_on=1;//Обновить статус установки
                DeviceState.flags.disp_upd=1;//Обновить дисплей
            }
            break;
                                        
            case SETUZOF: { // Выключить УЗ
                US_LED_RESET;//Выключить светодиод уdзг
                US_OFF;//выключить реле узг
                DeviceState.flags.uzg_on=0;//Обновить статус установки
                DeviceState.flags.disp_upd=1;//Обновить дисплей на установке
            }
            break;
                                        
            case BLAMPS: { // Запрос списка аварийных ламп  
                //адрес нашей установки
                CMD_BUF_TX[0]=BASE_ADDR;
                //КОП На который отвечаем
                CMD_BUF_TX[1]=BLAMPS;
                //Пакуем флаги состояния установки включен ли уф,узг,тревоги по уф,лампам,температуре и RS-485
                    //CMD_BUF_TX[2]= ;
                count=2; 
                //Запаковать и послать
                for(uint8_t i=0;i<=EXP_BOARD;i++) { // перетряхиваем платы на предмет нерабочих ламп
                    badlamps=get_lamps(i);	//получаем переменную с битами установленными в 1 соотв.нерабочим лампам
                    CMD_BUF_TX[count]=badlamps&0xFF;//Младший байт содержит инф.-ю о лампах 1-8;
                    count=count+1;//счетчик байт
                    CMD_BUF_TX[count]=((badlamps>>8)&0x03) | (i<<4);//СТарший байт содержит адрес платы и лампы 10,9
                    count=count+1;
                }
                PACK_SEND_CMD(CMD_BUF_TX,count);//запаковать нашу  команду
            }
            break;
                                        
            case HCOUNT: { //Запрос наработки ламп установки
                //адрес нашей установки
                CMD_BUF_TX[0]=BASE_ADDR;
                //КОП На который отвечаем
                CMD_BUF_TX[1]=HCOUNT;
                //Пакуем флаги состояния установки включен ли уф,узг,тревоги по уф,лампам,температуре и RS-485
                //CMD_BUF_TX[2]= ;
                count=2; 
                //Запаковать и послать
                for(uint8_t i = 0; i < TOTAL_LAMPS; i++) { // перетряхиваем всю наработку которую насчитали
                    CMD_BUF_TX[count]=hourcounter[i];//&0xFF;//Наработка LSB
                    count=count+1;//счетчик байт
                    CMD_BUF_TX[count]=((hourcounter[i]>>8)&0xFF);//Наработка MSB
                    count=count+1;
                }
                PACK_SEND_CMD(CMD_BUF_TX,count);//запаковать всю хуйню и заслать
            }
            break;
                                        
            case LCOUNT: {
                //адрес нашей установки
                CMD_BUF_TX[0]=BASE_ADDR;
                //КОП На который отвечаем
                CMD_BUF_TX[1]=LCOUNT;
                CMD_BUF_TX[2]=TOTAL_LAMPS;//Посылаем количество ламп в системе
                //Запаковать и послать ответ на команду
                PACK_SEND_CMD(CMD_BUF_TX,3);
            }
            break;
        } // switch
        RxCMDReady=false;//Обработали команду от компа-сбрасываем флаг новой команды
    } //if((RxCMDReady==true)&&(CMD_BUF_RX[0]==BASE_ADDR))
}