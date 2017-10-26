/**
 *      должна быть в кодировки windows-1251
 */
#include <stdbool.h>
#include "menu.h"
#include "display.h"
#include "keyboard.h"
#include "device_config.h"
#include "communic.h"
#include "eeprom.h"
#include "MBSlave.h"

//Число строк дисплея
#define MAX_LINES				4 
//Флаги обновления дисплея и нажатия кнопки
#define REDRAW				0x02
#define KEY						0x01
#define	MENU_BACK			0x04

struct flags
{
			uint8_t uf_on		   :1;
      uint8_t uzg_on		 :1;
      uint8_t alarm_uf	 :1;
      uint8_t alarm_lamp :1;
      uint8_t alarm_temp :1;
      uint8_t alarm_comm :1;
			uint8_t disp_upd   :1;
			uint8_t disp_red	 :1;
} ;
struct servicelog 
{
	uint16_t oncounter;//количество включений установки
	uint16_t res_all;	 // количество единичных сбросов
	uint16_t res_one;	 // количество полных сбросов наработки
	uint16_t res_log;	 //количество сбросов лога работы
};
extern uint8_t kb;
extern uint16_t oncounter;//счетчик включений
extern volatile struct flags DeviceState;
extern volatile struct servicelog DeviceLog;
extern uint8_t temperatura;
extern uint8_t uf_level;
extern uint16_t get_lamps(uint8_t board_num);
extern uint8_t get_lamps_count(uint8_t plata);
extern uint16_t hourcounter[];
extern volatile bool RxDataReady;
//extern uint8_t BUF1[];
extern void PACK_SEND(uint8_t *BUF, uint8_t count);
extern void UNPACK(uint8_t *BUF);
extern void save_pars(void);
//extern volatile uint8_t uf_threshold;
extern volatile struct EEPROMst eeprom;
extern volatile uint8_t return_counter;//счетчик для возврата из меню
//void narabotka(uint8_t plata,uint16_t *countarray);

// пункты меню
char *menu_items[] = { "Наработка",
                       "Аварии",
                       "Конфигурация",
                       "Лог работы"				
	                 };

struct menu_item {
    char *menu_item;
    char podmenu;
    char prevmenu;
    char (*function) (void);
};	

const struct menu_item  root_menu[] =
{   //menu item	  		podmenu     prevmenu    функция вызываемая данным пунктом
    {"Аварии",          5,          0,          0},
    {"Наработка",       2,          0,          0},
    {"Конфигурация",    3,          0,          0},
    {"Лог работы",      4,          0,          0},
    {0},
};
const struct menu_item nar_menu[]=
{   //menu item	  		podmenu     prevmenu    функция вызываемая данным пунктом
    {"Просмотр",        0,          1,          narab},
    {"Сброс наработки", 6,          1,          0},
    {0},
};
const struct menu_item narreset_menu[] =
{   //menu item         podmenu     prevmenu    функция вызываемая данным пунктом
    {"Сбросить",        0,          2,          0},
    {" Все лампы",      7,          2,          0},
    {" Лампу  номер ",  0,          2,          reset_nar_one},
    {0},
};
const struct menu_item narresal_menu[] =
{   //menu item             podmenu     prevmenu    функция вызываемая данным пунктом
    {"Сбросить все лампы",  0,          6,          0},
    {" Нет",                6,          6,          0},
    {" Да ",                0,          6,          reset_nar_all},
    {0},
};
const struct menu_item config_menu[] =
{   //menu item             podmenu     prevmenu    функция вызываемая данным пунктом
    {"Просмотр конф.-ии",   0,          1,          config},
    {"Настройки",           0,          1,          setup_device},
    {0},
};
const struct menu_item log_menu[] =
{   //menu item         podmenu     prevmenu    функция вызываемая данным пунктом
    {"Просмотреть лог", 0, 			1,          logg},
    {"Сбросить лог",    0,          1,          logg_reset},
    {0},
};
const struct menu_item alarms_menu[] =
{   //menu item             podmenu     prevmenu    функция вызываемая данным пунктом
    {"Нерабочие лампы",     0,          1,          alarms},
    {"Ошибки линии RS485",  0,          1,          alarmsRS},
    {"Сбросить аварии",     0,          1,          reset_alarms},
    {0},
};
// массив указателей на структуру типа menu_item
// обращаться будем так  menu[имя меню][номер пункта].элемент меню
const struct menu_item *menu[] = {  0,
                                    root_menu,      // 1
                                    nar_menu,       // 2
                                    config_menu,    // 3
                                    log_menu,       // 4
                                    alarms_menu,    // 5
                                    narreset_menu,  // 6
                                    narresal_menu   // 7
                                 };

void display_main_screen(void)//Функция отображения главного дисплея
{
    char stroka[20];

    if (DeviceState.disp_red) {	
        lcd_clear();
        // set cursor(x,y);где x-эт столбец,а y-это строка,нумерация от 0 и по х и по y;			
        set_cursor (5,0);	
        lcd_print (UOV_NAME);
        set_cursor (0,1);	
        sprintf(stroka,"Ламп :%d",TOTAL_LAMPS); 
        lcd_print (stroka);
        set_cursor (0,2);
        lcd_print ("Авария:");
    }
			 
    // выводим Тревогу ЛАМПЫ на главный дисплей
    if (DeviceState.alarm_lamp)
        sprintf(stroka,"ЛАМПЫ");
    if ( !(DeviceState.alarm_lamp) )
        sprintf(stroka,"     ");
    set_cursor (7,2);
    lcd_print (stroka);
    // выводим Тревогу НИЗКИЙ УФ на ДИСПЛЕЙ
    if (DeviceState.alarm_uf)
        sprintf(stroka," УФ");
    if (!(DeviceState.alarm_uf))
        sprintf(stroka,"   ");
    lcd_print (stroka);
    // выводим Тревогу ТЕМПЕРАТУРА на ДИСПЛЕЙ
    if (DeviceState.alarm_temp)
        sprintf(stroka," Темп");
    if (!(DeviceState.alarm_temp))
        sprintf(stroka,"     ");
    lcd_print (stroka);
    // если есть плата датчиков тогда выводим информацию с датчиков
    if(UF_T_BOARD!=0) {
        //выводим температуру			
        set_cursor (0,3);
        //lcd_print ("                    ");
        if (T_SENS_PRESENT != 0) {
            set_cursor (0,3);
            if (temperatura<10) {
                sprintf(stroka,"t=%d",temperatura); 
            } else {
                sprintf(stroka,"t=%d ",temperatura); 
            }
        lcd_print (stroka);
		}
        //выводим уровень УФ		
        if (UF_SENS_PRESENT !=0 ) {
            set_cursor (5,3);
            if (uf_level < 10) {
                sprintf(stroka,"УФ=%d%%  ",uf_level);
            } else if (uf_level < 100) {
                sprintf(stroka,"УФ=%d%% ",uf_level);
            } else {
                sprintf(stroka,"УФ=%d%% ",uf_level);
            }
            lcd_print (stroka);
        } else {
            if (DeviceState.uf_on)
                sprintf(stroka,"УФ ВКЛ");
            if (!(DeviceState.uf_on))
                sprintf(stroka,"УФ ВЫК");
                set_cursor (5,3);
                lcd_print (stroka);
		}
    } else {
        //выводим статус УФ
        if (DeviceState.uf_on)
            sprintf(stroka,"УФ ВКЛ");
        if (!(DeviceState.uf_on))
            sprintf(stroka,"УФ ВЫК");
        set_cursor (5,3);
        lcd_print (stroka);
    }
    //выводим статус УЗГ
    if (DeviceState.uzg_on)
        sprintf(stroka,"УЗГ ВКЛ");
    if (!(DeviceState.uzg_on))
        sprintf(stroka,"УЗГ ВЫК");
    set_cursor (13,3);
    lcd_print (stroka);
			
} // display_main_screen


	
//Функция вывода меню передаем указатель на структуру		
void display_menu(void)				
{
    volatile uint8_t temp;
    uint8_t line, menu_num, menu_ind, STATE;
    volatile uint8_t max_item, cursor;
    menu_num = 1;//указываем на root_menu
    menu_ind=0;//стартовать с 0 пункта
    cursor=0;
    lcd_clear();
    STATE = REDRAW | KEY | MENU_BACK;
    //вывод страницы меню 
    return_counter=0;
    
    while(1) {
        if (return_counter>RETURN_FROM_MENU)
            return;
        //если установлен флаг обновления дисплея - выводим меню или подменю
        if (STATE & REDRAW) {
            lcd_clear();
            for(line = 0; line < MAX_LINES; line++) { // Надо вывести в 4 строки
                // если существует пункт с таким номером
                if(menu[menu_num][menu_ind].menu_item) {
                    // максимальное число пунктов в данном меню
                    max_item = menu_ind;
                    // то выводим его											
                    set_cursor (0,line);										
                    lcd_print  ( menu[menu_num][menu_ind++].menu_item);
                } else {
                    menu_ind--;
                    break;
                }
            }
            STATE &= (~REDRAW);
        }
        // выводим курсор
        if (STATE & KEY) {
            set_cursor (19,temp);
            lcd_print(" "); //выводим пробел на старое место курсора
            set_cursor (19,cursor);//устанавливаем курсор на новую строку
            lcd_print("~"); //выводим стрелку
            STATE &= (~KEY);
            return_counter = 0;
        }
        
        Keyboard();
        temp = cursor; // запоминаем предыдущую позицию курсора
        switch (kb) {
            case UP_PRESS:
                if (cursor>0)
                    cursor--;
                // флаг нажатия на кнопку установить
                STATE |= KEY;
                break;
                    
            case DOWN_PRESS:
                if (cursor < max_item)
                    cursor++; 
                STATE|=KEY; // флаг нажатия на кнопку установить
                break;

            case ENTER_PRESS:
                //если мы вернулись из меню и уже давим кнопку,значит мы ее отпустили и не заметили этого-флаг сбросить
                STATE &= (~MENU_BACK);
                break;
                
            case ENTER_RELEASE: // по отусканию кнопки - выбираем пункт
                if ( !(STATE & MENU_BACK) ) { //если вернулись в меню и отпустили кнопку-не обрабатываем
                    // если есть подменю-заходим туда
                    if (menu[menu_num][cursor].podmenu != 0) {
                        menu_num = menu[menu_num][cursor].podmenu;
                        menu_ind = 0;
                        cursor = 0;
                        //флаг нажатия на кнопку и обновл дисплея установить
                        STATE = REDRAW | KEY;
                        //если подменю нет зато есть функция-вызываем
                    } else if (menu[menu_num][cursor].function != 0) {
                        // функция возвращает нам номер меню куда возвращаться
                        menu_num = menu[menu_num][cursor].function();
                        menu_ind = 0;
                        cursor = 0;
                        // //флаг нажатия на кнопку,обн.дисплея и возврата в меню установить
                        STATE = REDRAW | KEY | MENU_BACK;
                    }
                } else {
                    STATE &= (~MENU_BACK);//сбрасываем флаг возврата в меню
                }
                break;
                    
            case ENTER_HOLD: 		// по длительному нажатию - выходим  
                //если вернулись в меню-не обрабатываем
                if ( !(STATE & MENU_BACK) ) {
                    //если есть куда вернуться из подменю
                    if(menu[menu_num][cursor].prevmenu != 0) {
                        //номер меню в массиве- наше предыдущее меню 
                        menu_num = menu[menu_num][cursor].prevmenu; 
                        menu_ind=0;
                        cursor=0;
                        STATE=REDRAW|KEY|MENU_BACK;
                    } else {
                        return;
                    }
                }
                break;
        } // switch end
    } // while(1)
}

/////////////////////////////////////////////////////////////////////////////////
//										ВЫВОДИМ ТРЕВОГИ НА ДИСПЛЕЙ
/////////////////////////////////////////////////////////////////////////////////
		

char alarms(void)
{
    uint8_t  i,j,STATE;
    uint8_t  dead_lamps;
    uint16_t badlamp;
    char string[21];
    
    dead_lamps=0;
    lcd_clear();
    STATE=REDRAW|KEY|MENU_BACK;
    //вывод 
    lcd_clear();
    set_cursor (0,0);
    lcd_print("Нерабочие лампы");
    set_cursor (0,1);
    return_counter=0;
    while(1)
    {
        if (return_counter>RETURN_FROM_MENU)
            return 5;
        if(STATE &  REDRAW) { //если установлен флаг обновления дисплея- выводим меню или подменю
            for(i=0;i<=EXP_BOARD;i++) { // перетряхиваем платы
                badlamp=get_lamps(i);	//получаем переменную с битами установленными в 1 соотв.нерабочим лампам
                for (j=0;j<10;j++) { //обходим эту переменную
                    if(badlamp&0x01) {	//если попалась нерабочая лампа
                        if(dead_lamps<=20) { //если еще не засрали все 3 строки нашего монитора:)
                            if ((dead_lamps==6)||(dead_lamps==13)||(dead_lamps==20)) 
                                sprintf(string,"%02d",(i*10+1+j)); // выводим ее на дисплей
                            else  
                                sprintf(string,"%02d ",(i*10+1+j)); // выводим ее на дисплей
                            lcd_print  (string);
                            dead_lamps++;		//увеличиваем счетчик нерабочих ламп
                        }
                    }// если у нас набралось 7 нерабочих ламп(считали с 0 потому 6) переводим курсор на 2(нум.стр.с 0) строку
                    if (dead_lamps==7) 
                        set_cursor(0,2); 
                    if (dead_lamps==14)
                        set_cursor(0,3);// если у нас набралось 14 нерабочих ламп  переводим курсор на 3 строку
                    
                    badlamp=(badlamp>>1);//&(LAMPS_MASK[LAMPS_INST-1]);
                }
                
            }
            STATE&=(~REDRAW);
        }
        
        if(STATE&KEY)//
        {
                
        }
        
        Keyboard();
        switch (kb)
        {
        
                case UP_PRESS: // 
                    {
                        
                    }
                break;
                    
                case DOWN_PRESS: // 
                    {		
                        
                    }
                break;
                
                case ENTER_RELEASE: 		// по отусканию кнопки - выходим из показа аварий
                    {
                            
                        
                    }
                break;
                    
                case ENTER_HOLD: 		// по длительному нажатию - просто дрочим
                    {
                        return 5; //5-это индекс нашего меню откуда мы пришли	
                    }
                break;
        
        }//switch end
        
    } // while(1)
        
}
/////////////////////////////////////////////////////////////////////////////////////////
//                  КОНЕЦ ТРЕВОГ 
/////////////////////////////////////////////////////////////////////////////////////////
char alarmsRS(void)
	{
			uint8_t   i,STATE;
			volatile	uint32_t delaycnt=0x1FFFF; 
			char string[21];
			DeviceState.alarm_comm=0; 
			lcd_clear();
			
			//вывод 
			lcd_clear();
			set_cursor (0,0);
		  sprintf(string,"RS485: EXP.%d UFT.%d",EXP_BOARD,UF_T_BOARD);
			lcd_print(string);
			set_cursor(0,1);
			
			RxDataReady=false; 
			if(EXP_BOARD>0)//если есть платы расширения
							{
								for(i=1;i<=EXP_BOARD;i++)
									{
										//BUF1[0]= i;//адрес платы расширения
										//BUF1[1]=LCOUNT;		//Запрос ЧИСЛА ЛАМП
										//PACK_SEND(BUF1,2);//запаковать и послать
										while( RxDataReady!=true){delaycnt--;if(delaycnt==0)break;}//ждем прихода пакета,если пакет не пришел-ошибка на шине

										if(delaycnt==0){ sprintf(string,"addr.%#x",i);lcd_print(string);DeviceState.alarm_comm=1;}	//выставляем флаг ошибки шины
										RxDataReady=false;
									}
								 
							}					
			 
								
			return_counter=0; 
			while(1)
			{
					if (return_counter>RETURN_FROM_MENU) return 5;
					if(STATE &  REDRAW) //если установлен флаг обновления дисплея- выводим меню или подменю
					{
					
						STATE&=(~REDRAW);
					}
					
					if(STATE&KEY)//
					{
						 
						STATE&=(~KEY);	 
					}
					
				  Keyboard();
					switch (kb)
					{
					
						 case UP_PRESS: // 
								{
									 
								}
						 break;
						  
								
						 case DOWN_PRESS: // 
								{		
									 
								}
						 break;
						  
						 
						 case ENTER_RELEASE: 		// по отусканию кнопки  
								{
									 
								}
						 break;
								
						 case ENTER_HOLD: 		// по длительному нажатию - выходим в преддущее меню
								{
									return 5;
								}
						 break;
					
					}//switch end
				
			}
				
	}	
char config(void)
	{
			uint8_t   STATE;
			uint8_t  max_lamps=0;
			 
			char string[21];
			max_lamps=LAMPS_INST;//Плюсуем базовую плату с ее лампами
			if(EXP_BOARD>0)//если есть платы расширения
			{
				max_lamps=max_lamps+((EXP_BOARD-1)*10);
				max_lamps=max_lamps+get_lamps_count(EXP_BOARD);
			}
			lcd_clear();
			STATE=KEY;
			//вывод 
			lcd_clear();
			set_cursor (0,0);
			lcd_print("Конфигурация системы");
			set_cursor (0,1);
			sprintf(string,"Всего ламп: %d",max_lamps);
			lcd_print(string);
								
			sprintf(string,"Плат расширения: %d",EXP_BOARD);
			set_cursor (0,2);
			lcd_print(string);
								
			if(UF_T_BOARD) sprintf(string,"Плата УФ и Темп: +");
			else sprintf(string,"Плата УФ и Темп: -");
			set_cursor (0,3);
			lcd_print(string);
			return_counter=0;
			while(1)
			{
					if (return_counter>RETURN_FROM_MENU) return 3;
					if(STATE &  REDRAW) //если установлен флаг обновления дисплея- выводим меню или подменю
					{
					
						STATE&=(~REDRAW);
					}
					
					if(STATE&KEY)//
					{
						 
						STATE&=(~KEY);	 
					}
					
				  Keyboard();
					switch (kb)
					{
					
						 case UP_PRESS: // 
								{
									 
								}
						 break;
						  
								
						 case DOWN_PRESS: // 
								{		
									 
								}
						 break;
						  
						 
						 case ENTER_RELEASE: 		// по отусканию кнопки  
								{
									 
								}
						 break;
								
						 case ENTER_HOLD: 		// по длительному нажатию - выходим в преддущее меню
								{
									return 3;
								}
						 break;
					
					}//switch end
				
			}
			
		 
	}
char narab(void)
	{
			uint8_t  STATE;
			uint8_t  cursor;
			volatile uint8_t  plata=0;//номер платы с которой считываем наработку 
			uint8_t  lamps_in_board=LAMPS_INST;
	//		uint16_t hcounter[10];
			char string[21];
			cursor=0;
			lcd_clear();
			STATE=REDRAW|KEY|MENU_BACK;
			//вывод страницы меню 
			lcd_clear();
			set_cursor (0,0);
			lcd_print("Наработка Плата");
			set_cursor (0,1);
			lcd_print("Лампа ");
			set_cursor (0,2);		// 
			lcd_print("Лампа "); 
		  set_cursor(0,3);
		  lcd_print("Лампа ");
			return_counter=0;
		 	while(1)
			{
					if (return_counter>RETURN_FROM_MENU) return 2;
					if(STATE &  REDRAW) //если установлен флаг обновления дисплея- выводим меню или подменю
					{
							lamps_in_board=get_lamps_count(plata);//получаем количество ламп на плате
						  //narabotka(plata,hcounter); //считываем наработку от платы (когда напишется ф-я заменить в ввыводе хауркаунтер на переменную)
						  set_cursor(15,0);
							sprintf(string,"  %02d",plata);//выводим адрес платы i*10+1+j
							lcd_print(string);
														
							set_cursor(0,1);
							sprintf(string,"Лампа %02d   %05d  ",plata*10 +cursor+1,hourcounter[(plata*10+cursor)]);//hcounter[(cursor)]);//выводим номер лампы и наработку
							lcd_print(string);
							
							set_cursor(0,2);
							if((cursor+1)<(lamps_in_board))
								{
									sprintf(string,"Лампа %02d   %05d  ",plata*10+cursor+2,hourcounter[(plata*10+cursor+1)]);//hcounter[(cursor+1)]);//выводим номер лампы и наработку
								}
								else sprintf(string,"                    ");
							lcd_print(string);
							
							set_cursor(0,3);
							if((cursor+2)<(lamps_in_board))
								{
										sprintf(string,"Лампа %02d   %05d  ",plata*10+cursor+3,hourcounter[(plata*10+cursor+2)]);//hcounter[(cursor+2)]);//выводим номер лампы и наработку
								}
								else sprintf(string,"                    ");
							lcd_print(string);
						
							STATE&=(~REDRAW);
							return_counter=0;
					}
					
					if(STATE&KEY)//выводим курсор
					{
						 
					}
					
				  Keyboard();
					switch (kb)
					{
					
						 case UP_PRESS: // 
								{
									 if (cursor>0) cursor--; //если текущая позиция курсора не ноль- уменьшаем
									 else 
									 {
										 if(plata>0)
										 {
											 plata--;
										  lamps_in_board=get_lamps_count(plata);
											cursor=lamps_in_board-1-2;//lamps_in_board(plata)-2;
										 }
										 
									 }
									 STATE|=REDRAW;//флаг обновл. экрана установить
								}
						 break;
								
						 case DOWN_PRESS: // 
								{		//если позиция курсора не больше максимального числа установленных ламп
										if (cursor<(lamps_in_board-1-2)) cursor++; 
										else 
										{
											if(plata<EXP_BOARD)
											{
												plata++;
												cursor=0;
												lamps_in_board=get_lamps_count(plata);
											}
											
										}
										STATE|=REDRAW;//флаг обновл. экрана  установить
								}
						 break;
						 
						 case ENTER_RELEASE: 		// по отусканию кнопки - выходим из показа аварий
								{
									 if(plata<EXP_BOARD)plata++; //перебираем наши платы
									 else plata=0;
									 cursor=0;
									 STATE|=REDRAW;
								}
						 break;
								
						 case ENTER_HOLD: 		// по длительному нажатию - просто дрочим
								{
										return 2; //2-это индекс нашего меню откуда мы пришли
								}
						 break;
					
					}//switch end
				
			}
	}	
char reset_nar_all(void)//сброс наработки всех подклченных ламп в системе
	{
		uint8_t i;
			for(i=0;i<TOTAL_LAMPS;i++)// обходим все на базовой плате
					{
							hourcounter[i]=0;//гасим наработку
					}
					DeviceLog.res_all++;
					save_pars();//сохраняем параметры наработки и счетчик включений на базовой плате

										
									
			return 6;
	}
	
char reset_nar_one(void)//Сброс наработки одной лампы выбираемой по номеру
	{
			uint8_t   STATE;
			uint8_t  max_lamps=0;
			uint8_t  cursor=1;
			char string[21];
			max_lamps=TOTAL_LAMPS;//максимальное число ламп в списке

			
			lcd_clear();
			STATE=REDRAW;
			//вывод 
			lcd_clear();
			set_cursor (0,0);
			lcd_print("Введите номер");
			set_cursor (0,1);
			lcd_print("Лампа N ");
			set_cursor(0,2);
			lcd_print("Нажатие   ~ Сброс");
			set_cursor(0,3);
			lcd_print("Удержание ~ Отмена");
			return_counter=0;
			while(1)
			{
					if (return_counter>RETURN_FROM_MENU) return 6;
					if(STATE &  REDRAW) //если установлен флаг обновления дисплея- выводим меню или подменю
					{
						 set_cursor(9,1);
						 sprintf(string,"%02d",cursor);
						 lcd_print(string);
						return_counter=0;	
						STATE&=(~REDRAW);
					}
					
					if(STATE&KEY)//
					{
						 
					}
					
				  Keyboard();
					switch (kb)
					{
					
						 case UP_PRESS: // 
								{
									if(cursor>1) cursor--;
									else cursor=1;
									STATE|= REDRAW;
								}
						 break;
						 case UP_HOLD: // 
								{		
									if(cursor>10) cursor=cursor-10;
									else if(cursor>1) cursor--;
									STATE|= REDRAW;
								}
						 break;
								
						 case DOWN_PRESS: // 
								{		
									if (cursor<max_lamps) cursor++;
									else  cursor=max_lamps;
									STATE|= REDRAW;
								}
						 break;
						 case DOWN_HOLD: // 
								{		
									if (cursor<(max_lamps-10)) cursor=cursor+10;
									else if (cursor<max_lamps) cursor++;
									STATE|= REDRAW;
								}
						 break;
						 
						 case ENTER_RELEASE: 		// по отусканию кнопки - отправляем команду на сброс наработки лампы N и выходим
								{
									
									
											hourcounter[cursor-1]=0;
											DeviceLog.res_one++;
											save_pars();
									
									return 6; //6-это индекс нашего меню откуда мы пришли	
								
									
								}
						 break;
								
						 case ENTER_HOLD: 		// по длительному нажатию - выходим в преддущее меню
								{
									return 6;
								}
						 break;
					
					}//switch end
				
			}
			
		 		
	}
	
	
char logg(void)
{
			uint8_t  STATE;
//			uint8_t  cursor;
			 
			char string[21];
//			cursor=0;
			lcd_clear();
			STATE=REDRAW|KEY|MENU_BACK;
			//вывод страницы меню 
			lcd_clear();
			set_cursor (0,0);
			sprintf(string,"Включений: %03d",DeviceLog.oncounter);
			lcd_print(string);
			set_cursor (0,1);
			sprintf(string,"Сбросов полных:%03d",DeviceLog.res_all);
			lcd_print(string);
			set_cursor (0,2);		// 
			sprintf(string,"Сбросов один.: %03d",DeviceLog.res_one);
			lcd_print(string); 
		  set_cursor(0,3);
		  sprintf(string,"Сбросов лога:  %03d",DeviceLog.res_log);
			lcd_print(string);
			return_counter=0;
		 	while(1)
			{
					if (return_counter>RETURN_FROM_MENU) return 4;
					if(STATE &  REDRAW) //если установлен флаг обновления дисплея- выводим меню или подменю
					{
							 
							return_counter=0;
							STATE&=(~REDRAW);
					}
					
					if(STATE&KEY)//выводим курсор
					{
						 
					}
					
				    Keyboard();
					switch (kb)
					{
					
						 case UP_PRESS: // 
								{
									  
									 STATE|=REDRAW;//флаг обновл. экрана установить
								}
						 break;
								
						 case DOWN_PRESS: // 
								{		 
										STATE|=REDRAW;//флаг обновл. экрана  установить
								}
						 break;
						 
						 case ENTER_RELEASE: 		// по отусканию кнопки - выходим из показа аварий
								{
									  
									 STATE|=REDRAW;
								}
						 break;
								
						 case ENTER_HOLD: 		// по длительному нажатию - просто дрочим
								{
										return 4; //4-это индекс нашего меню откуда мы пришли
								}
						 break;
					
					}//switch end
				
			}
	}	
//СБРОС ЛОГА РАБОТЫ
char logg_reset(void)
{
			uint8_t  STATE;
			volatile uint8_t  cursor,cntr;
			volatile uint16_t kod,multy; 
			char string[21];
			cursor=0;kod=0;
			multy=1;cntr=0;
			lcd_clear();
			STATE=REDRAW|KEY|MENU_BACK;
			//вывод страницы меню 
			lcd_clear();
			set_cursor (0,0);
			sprintf(string,"Введите код:" );
			lcd_print(string);
			 return_counter=0;
		 	while(1)
			{
					if (return_counter>RETURN_FROM_MENU) return 4;
					if(STATE &  REDRAW) //если установлен флаг обновления дисплея- выводим меню или подменю
					{
							 return_counter=0; 
							 set_cursor (0,1);
							 sprintf(string,"  %04d",(kod+multy*cursor));
							 lcd_print(string);
						
							STATE&=(~REDRAW);
					}
					
					if(STATE&KEY)//выводим курсор
					{
						 
					}
					
					
				    Keyboard();
					switch (kb)
					{
					
						 case UP_PRESS: // 
								{
									 if(cursor<9)cursor++;
									 STATE|=REDRAW;//флаг обновл. экрана установить
								}
						 break;
								
						 case DOWN_PRESS: // 
								{		 
									if(cursor>0)cursor--;	
									STATE|=REDRAW;//флаг обновл. экрана  установить
								}
						 break;
						 
						 case ENTER_RELEASE: 		// по отусканию кнопки - сохраняем параметры установки и выходим
								{
									 kod+=(cursor*multy);
									 multy=multy*10;
									 cntr++;
									 cursor=0;
									 if (cntr==4)
									 {
										 if(kod==VALID_RESET_CODE)
										 {
												 DeviceLog.oncounter=0;
												 DeviceLog.res_all=0;
												 DeviceLog.res_log++;
												 DeviceLog.res_one=0;
												 save_pars();
												return 4;
										 } 
										 cntr=0;
										 kod=0;
										 multy=1;
										 cursor=0;
									 }
									
									 STATE|=REDRAW;
								}
						 break;
								
						 case ENTER_HOLD: 		// по длительному нажатию - просто дрочим
								{
										return 4; //4-это индекс нашего меню откуда мы пришли
								}
						 break;
					
					}//switch end
				
				
			}
	}	 

	
char setup_device(void)
	{
		  uint8_t  STATE;
			volatile uint8_t  cursor ;
			 
			char string[21];
			cursor=eeprom.UFmin; 
			 
			lcd_clear();
			STATE=REDRAW|KEY|MENU_BACK;
			//вывод страницы меню 
			lcd_clear();
			set_cursor (0,0);
			sprintf(string,"Тревога УФ:" );
			lcd_print(string);
		  set_cursor(0,2);
			lcd_print("Нажатие   ~ Сохран.");
			set_cursor(0,3);
			lcd_print("Удержание ~ Отмена");
			return_counter=0;
		 	while(1)
			{
					if (return_counter>RETURN_FROM_MENU) return 3;
					if(STATE &  REDRAW) //если установлен флаг обновления дисплея- выводим меню или подменю
					{
							 return_counter=0; 
							 set_cursor (0,1);
							 sprintf(string,"  %04d",( cursor));
							 lcd_print(string);
						
							STATE&=(~REDRAW);
					}
					
					if(STATE&KEY)//выводим курсор
					{
						 
					}
					
					
				    Keyboard();
					switch (kb)
					{
					
						 case UP_PRESS: // 
								{
									 if(cursor<99)cursor++;
									 STATE|=REDRAW;//флаг обновл. экрана установить
								}
						 break;
							
						 case UP_HOLD: // 
								{
									 if(cursor<90)cursor=cursor+10;
									 else if (cursor<99) cursor++;
									 STATE|=REDRAW;//флаг обновл. экрана установить
								}
						 break;
								
						 case DOWN_PRESS: // 
								{		 
									if(cursor>0)cursor--;	
									STATE|=REDRAW;//флаг обновл. экрана  установить
								}
						 break;
								
						 case DOWN_HOLD: // 
								{		 
									if(cursor>10)cursor=cursor-10;
									else if(cursor>0) cursor--;
									STATE|=REDRAW;//флаг обновл. экрана  установить
								}
						 break;
						 
						 case ENTER_RELEASE: 		// по отусканию кнопки - выходим из показа аварий
								{
                                    extern struct MBRegSt mbSlave;
                                    eeprom.UFmin = cursor;
                                    mbSlave.RegOut[UFmin] = eeprom.UFmin;
									save_pars();
									return 3; //4-это индекс нашего меню откуда мы пришли
									
									 
								}
						 break;
								
						 case ENTER_HOLD: 		// по длительному нажатию - просто дрочим
								{
									 
									return 3; //4-это индекс нашего меню откуда мы пришли
								}
						 break;
					
					}//switch end
				
				
			}
	}
		
		 
 
char reset_alarms(void)
	{
		DeviceState.alarm_uf=0;
		DeviceState.alarm_lamp=0; 
		DeviceState.alarm_temp=0;
		DeviceState.alarm_comm=0;
		return 5;
	}






// в прерывании по уарту считываем строку символов начиная от стартового признака(SOH) до конечного признака(ETX)
//потом патроним данные и пихаем по регистрам

