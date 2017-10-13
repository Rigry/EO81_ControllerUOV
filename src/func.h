#ifndef FUNC_H
#define FUNC_H

#include <stdint.h>

uint8_t get_temp(void);
uint8_t get_uf_level(void);

void RTC_Config(void);
void RTC_IRQHandler(void);
void init_ports (void);
// запись в микросхему eeprom
void save_pars(void);
// чтение из микросхемы еепром
void get_pars(void);

// ДК интерфейс с внешним миром
void USART1_Init(void);
void USART1_TX_DMA_Init(void);
void USART1_RX_DMA_Init(void);
void StartDMAChannel5(void);
void StartDMAChannel4(unsigned int LengthBufer);
// ДК собирает пакет по правилам протокола и отправляет
void PACK_SEND_CMD(uint8_t *BUF, uint8_t count);

// ДК: внутрнний интерфейс
void UART3_Init(void);							
void PACK_SEND(uint8_t *BUF, uint8_t count);	// ДК: переписал

// ДК: старые функции обрамления пакета по правилам не модбас
void UNPACK(uint8_t *BUF);						//определить, нужна ли для внешнего интерфейса
void UNPACK_RECEIVE(unsigned char *BUFRX);		//определить, нужна ли для внешнего интерфейса

//////////////////////////////SPI EEPROM///////////////////////////////
void spi_send(uint8_t data);// послать
uint8_t spi_read(void);
void spi_init(void);// инициализация SPI		
uint8_t EEWrite(uint16_t addr,uint8_t data);// записать байт data по адресу addr
uint8_t EERead(uint16_t addr);// считать байт  по адресу addr
void EEWREN(void);//разрешение записи в eeprom
void EEWRDI(void);//запрещение записи в eeprom
uint8_t CHECK_BSY(void);
////////////////////////////////////////////////////////////////////////

uint16_t get_lamps(uint8_t board_num);		//опрашивает работающие лампы плат расширения
uint8_t get_lamps_count(uint8_t plata);		//опрашивает количество установленных ламп

// подпрограмма действия на нажатия клавиш
void KeyboardAction (void);
// подпрограмма действия на запросы по модбас извне
void MBSlaveAction (void);



#endif // FUNC_H