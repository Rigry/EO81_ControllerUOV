//служебные признаки посылок,передаваемых по уарту
#define   SOH 0xFF    //признак начала пакета данных
#define   ETX 0x13		//признак конца  пакета данных
#define 	DLE 0x10		//экранирующий символ, если байт данных совпадает с любым признаком,тогда он инвертируется и 
//перед ним добавляется символ DLE
//Адреса Плат расширения и измерения
#define   BASE_ADDR		0x00
#define		EXP1_ADDR		0x01
#define		EXP2_ADDR		0x02
#define		EXP3_ADDR		0x03
#define		EXP4_ADDR		0x04
#define		EXP5_ADDR		0x05
#define		EXP6_ADDR		0x06
#define		EXP7_ADDR		0x07
#define		EXP8_ADDR		0x08
#define		EXP9_ADDR		0x09
#define		UF_T_ADDR		0x0A
//дуфайны для управления трансивером RS485
#define		USART_MODE_TX()	GPIOA->BSRR = GPIO_Pin_15
#define		USART_MODE_RX()	GPIOA->BRR  = GPIO_Pin_15
#define   CMD_MODE_TX()		GPIOA->BSRR = GPIO_Pin_8
#define 	CMD_MODE_RX()		GPIOA->BRR  = GPIO_Pin_8

//ОпКоды для управления  платами
#define		LCOUNT		0x20	//число ламп
#define		BLAMPS		0x21	//Плохие лампы
#define 	HCOUNT		0x22	//Часы наработки
#define		UFONST		0x23	//сообщение УФ включен
#define		UFOFST		0x24	//сообщение УФ выключен
#define		GETEMP		0x25	//Получить температуру
#define		GETUFL		0x26	//Получить уровень УФ
#define 	HRESET		0x27	//Сбросить наработку лампы или всех ламп
//Опкоды для управления установкой с компьютера
#define		GETSTATE	0x28	//Запрос состояния установки
#define		SETUFTHR	0x29	//Установить порог срабатывания тревоги УФ
#define   SETUFON		0x30	//Включить ультрафиолет
#define   SETUFOF		0x31	//Выключить ультрафиолет
#define		SETUZON		0x32	//Включить УЗГ
#define 	SETUZOF		0x33	//Выключить УЗГ
#define 	GETUFTHR	0x34  //Получить текущее значение порога тревоги УФ
#define 	BUSY			0x35	//Ответ установка занята



