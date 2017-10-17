#include "func.h"

void init_ports (void)
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

void USART1_Init (uartset_t set)
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
    if (set.parityEn) {
        USART1->CR1 |= USART_CR1_PCE;
        if (set.parityEven) {
            USART1->CR1 &= ~USART_CR1_PS;
        } else {
            USART1->CR1 |= USART_CR1_PS;
        }
    }
    USART1->CR2 &= ~USART_CR2_STOP;
    if (set.stopBitsMinus1) {
        USART1->CR2 |= USART_CR2_STOP_1;
    }
	
    uint8_t tmp16;
    tmp16 = (set.boud == bd9600)   ? F_CPU/9600  :
            (set.boud == bd14400)  ? F_CPU/14400 :
            (set.boud == bd19200)  ? F_CPU/19200 :
            (set.boud == bd28800)  ? F_CPU/28800 :
            (set.boud == bd38400)  ? F_CPU/38400 :
            (set.boud == bd57600)  ? F_CPU/57600 :
            (set.boud == bd76800)  ? F_CPU/76800 : 
                                     F_CPU/115200; 

    USART1->BRR   =   tmp16;                             //скорость 9600 72000000/9600
    USART1->CR1  &=  ~USART_CR1_M;                       //8 бит данных
                            
    USART1->CR1  |=   USART_CR1_UE;                      //включить USART1
    USART1->CR1  |=   USART_CR1_TE;                      //включить передатчик
    USART1->CR1  |=   USART_CR1_RE;                      //включить приемник
    USART1->CR1  |=   USART_CR1_IDLEIE|USART_CR1_TCIE;    //включить прерывание по LINE IDLE  и окончанию передачи через усарт |USART_CR1_RXNEIE-было по кой то хуй
    NVIC_SetPriority (USART1_IRQn, 4);
    NVIC_EnableIRQ(USART1_IRQn);                         //Включаем прерывания в контроллере прерываний

    GPIOA->CRH   &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);   //очищаем MODE ? CNF
    GPIOA->CRH   |=   GPIO_CRH_MODE8 ;                   //выход ППулл 50мгц GPIO
}

void USART1_TX_DMA_Init (uint32_t memAdr)
{
    // ДК: разрешение работы ДМА для УАРТ1 передачи
    USART1->CR3  |=  USART_CR3_DMAT;
    // ДК: разрешение тактирования
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    // ДК: адрес переферии - буфера
    DMA1_Channel4->CPAR  =  (uint32_t)&USART1->DR;
    DMA1_Channel4->CMAR  =  (uint32_t)memAdr;
//    DMA1_Channel4->CPAR  =  (uint32_t) &USART1->DR;
//    DMA1_Channel4->CMAR  =  (uint32_t) mbSlaveUART.Buf;
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

void USART1_RX_DMA_Init(uint32_t memAdr, uint32_t bufSize)
{
    // ДК: разрешение работы ДМА для УАРТ1 приёма
    USART1->CR3  |=  USART_CR3_DMAR;  
    // ДК: разрешение тактирования
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    // ДК: адрес переферии - буфера
    DMA1_Channel5->CPAR  =  (uint32_t) &USART1->DR;
    DMA1_Channel5->CMAR  =  (uint32_t) memAdr;
    DMA1_Channel5->CNDTR =  bufSize;
    
    // ДК: какието нстройки (не трогаю)
    DMA1_Channel5->CCR   =  0;
    DMA1_Channel5->CCR  &= ~DMA_CCR5_CIRC;
    DMA1_Channel5->CCR  &= ~DMA_CCR5_DIR;
    DMA1_Channel5->CCR  &= ~DMA_CCR5_PSIZE;
    DMA1_Channel5->CCR  &= ~DMA_CCR5_PINC;
    DMA1_Channel5->CCR  &= ~DMA_CCR5_MSIZE;
    DMA1_Channel5->CCR  |=  DMA_CCR5_MINC;
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


//////////////////////////////////////////////////////////////////
// Модбас слейв
//////////////////////////////////////////////////////////////////
// ПЕРЕДАЧА ДАННЫХ С КОНТРОЛЛЕРА В СЕТЬ
void StartSlaveDMA_USART_TX (unsigned int LengthBufer)
{
    CMD_MODE_TX();//включить на передачу интерфейс с компьютером
    __NOP();__NOP();
  DMA1_Channel4->CCR  &= ~DMA_CCR4_EN;          //????????? ?????? ??????
  DMA1_Channel4->CNDTR =  LengthBufer;          //????????? ?????????? ?????? ??? ??????
  DMA1->IFCR          |= DMA_IFCR_CTCIF4;       //???????? ???? ????????? ??????
  DMA1_Channel4->CCR  |=  DMA_CCR4_EN;          //????????? ?????? ??????
}
// ПРИЕМ ДАННЫХ В КОНТРОЛЛЕР ИЗ СЕТИ
void StartSlaveDMA_USART_RX (void)
{
  DMA1_Channel5->CCR  &= ~DMA_CCR5_EN;          //????????? ?????? ??????
  DMA1_Channel5->CNDTR =  UART_BUF_SIZE;          //????????? ?????????? ?????? ??? ??????
  DMA1->IFCR          |= DMA_IFCR_CTCIF5;       //???????? ???? ????????? ??????
  DMA1_Channel5->CCR  |=  DMA_CCR5_EN;          //????????? ?????? ??????
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

