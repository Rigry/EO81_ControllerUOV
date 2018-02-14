/**
 *      ������ ���� � ��������� windows-1251
 */
#include <stdbool.h>
#include "menu.h"
#include "display.h"
#include "keyboard.h"
#include "communic.h"
#include "eeprom.h"     // dk
#include "MBSlave.h"    // dk
#include "defines.h"    // dk

//����� ����� �������
#define MAX_LINES   4 
//����� ���������� ������� � ������� ������
#define REDRAW      0x02
#define KEY         0x01
#define	MENU_BACK   0x04

char* nullName[] = { 0 };
char* models[] = {
    "���-��-1   ",     // 0
    "���-��-5   ",
    "���-��-10  ",
    "���-��-15  ",
    "���-��-30  ",
    "���-��-50  ",
    "���-��-100 ",
    "���-��-150 ",
    "���-��-200 ",
    "���-��-300 ",
    "���-��-500 ",
    "���-��-700 ",
    "���-��-1000",
    "���-��-1500",
    "���-��-5   ",
    "���-��-10  ",
    "���-��-15  ",
    "���-��-30  ",
    "���-��-50  ",
    "���-��-100 ",
    "���-��-150 ",
    "���-��-200 ",
    "���-��-300 ",
    "���-��-500 ",
    "���-��-700 ",
    "���-��-1000",
    "���-��-1500"   // 26
};
char* exist[] = {
    "����������� ",
    "������������"
};
char* boudrate[] = {
    "9600  ",
    "14400 ",
    "19200 ",
    "28800 ",
    "38400 ",
    "57600 ",
    "76800 ",
    "115200"
};
char* parity[] = {
    "����������",
    "��������  "  
};



struct flags
{
    uint8_t uf_on       :1;
    uint8_t uzg_on      :1;
    uint8_t alarm_uf    :1;
    uint8_t alarm_lamp  :1;
    uint8_t alarm_temp  :1;
    uint8_t alarm_comm  :1;
    uint8_t disp_upd    :1;
    uint8_t disp_red    :1;
} ;
struct servicelog 
{
	uint16_t oncounter;//���������� ��������� ���������
	uint16_t res_all;	 // ���������� ��������� �������
	uint16_t res_one;	 // ���������� ������ ������� ���������
	uint16_t res_log;	 //���������� ������� ���� ������
};
extern uint8_t kb;
extern uint16_t oncounter;//������� ���������
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
extern struct MBRegSt mbSlave;
extern volatile uint8_t return_counter;//������� ��� �������� �� ����
//void narabotka(uint8_t plata,uint16_t *countarray);
extern uint8_t EXP_BOARD;


// ������ ����
char *menu_items[] = { "���������",
                       "������",
                       "������������",
                       "��� ������"				
	                 };

struct menu_item {
    // ������ �� ��������� ����
    char *menu_item;
    // ����� ����, ���� ��������� ��� ������� �� ����
    char podmenu;
    // ����� ����, ���� ������������
    char prevmenu;
    // �������, ���������� ��� ������� �� ����
    char (*function) (char menu);
};
/*
enum {
    null = 0,
    root = 1,
    config = 3,
    config_change1 = 8
};*/

const struct menu_item  root_menu[] =
{   //menu item	  		podmenu     prevmenu    ������� ���������� ������ �������
    {"������",          5,          0,          0},
    {"���������",       2,          0,          0},
    {"������������",    3,          0,          0},
    {"��� ������",      4,          0,          0},
    {0},
};
const struct menu_item nar_menu[]=
{   //menu item	  		podmenu     prevmenu    ������� ���������� ������ �������
    {"��������",        0,          1,          narab},
    {"����� ���������", 6,          1,          0},
    {0},
};
const struct menu_item narreset_menu[] =
{   //menu item         podmenu     prevmenu    ������� ���������� ������ �������
    {"��������",        0,          2,          0},
    {" ��� �����",      7,          2,          0},
    {" �����  ����� ",  0,          2,          reset_nar_one},
    {0},
};
const struct menu_item narresal_menu[] =
{   //menu item             podmenu     prevmenu    ������� ���������� ������ �������
    {"�������� ��� �����",  0,          6,          0},
    {" ���",                6,          6,          0},
    {" �� ",                0,          6,          reset_nar_all},
    {0},
};
const struct menu_item config_menu[] =
{   //menu item             podmenu     prevmenu    ������� ���������� ������ �������
    {"�������� ����.-��",   0,          1,          config},
    {"���������",           0,          1,          setup_device},
    {"��������� ����.",     0,          1,          logg_reset},
    {"��������� ����",      0,          1,          setupModbus},
    {0},
};
const struct menu_item log_menu[] =
{   //menu item         podmenu     prevmenu    ������� ���������� ������ �������
    {"����������� ���", 0, 			1,          logg},
    {"�������� ���",    0,          1,          logg_reset},
    {0},
};
const struct menu_item alarms_menu[] =
{   //menu item             podmenu     prevmenu    ������� ���������� ������ �������
    {"��������� �����",     0,          1,          alarms},
    {"������ ����� RS485",  0,          1,          alarmsRS},
    {"�������� ������",     0,          1,          reset_alarms},
    {0},
};
const struct menu_item conf_change1_menu[] =
{   //menu item             podmenu     prevmenu    ������� ���������� ������ �������
    {"���-�� ����",         0,          3,          setLampsQty},
    {"����� ��������� ��",  0,          3,          resetMaxUF},
    {"������",              0,          3,          setName},
    {"���",                 9,          8,          0},
    {0},
};
const struct menu_item conf_change2_menu[] =
{   //menu item             podmenu     prevmenu    ������� ���������� ������ �������
    {"���� �����������",    0,          8,          setTmax},
    {"����������� �����",   0,          8,          setTrec},
    {"���",                 10,         8,          0},
    {0},
};
const struct menu_item conf_change3_menu[] =
{   //menu item             podmenu     prevmenu    ������� ���������� ������ �������
    {"����� ��������",      0,          8,          setSensBoard},
    {"������ �����������",  0,          8,          setTempSense},
    {"������ ��",           0,          8,          setUVsense},
    {0},
};
// ������ ���������� �� ��������� ���� menu_item
// ���������� ����� ���  menu[��� ����][����� ������].������� ����
const struct menu_item *menu[] = {  0,
                                    root_menu,      // 1
                                    nar_menu,       // 2
                                    config_menu,    // 3
                                    log_menu,       // 4
                                    alarms_menu,    // 5
                                    narreset_menu,  // 6
                                    narresal_menu,  // 7
                                    conf_change1_menu,   // 8
                                    conf_change2_menu,   // 9
                                    conf_change3_menu    // 10
                                 };

void display_main_screen(void)//������� ����������� �������� �������
{
    char stroka[20];

    if (DeviceState.disp_red) {	
        lcd_clear();
        // set cursor(x,y);��� x-�� �������,� y-��� ������,��������� �� 0 � �� � � �� y;			
        set_cursor (5,0);	
        lcd_print (models[eeprom.Name]);
        set_cursor (0,1);	
        sprintf(stroka,"���� :%d",eeprom.LampsQty); 
        lcd_print (stroka);
        set_cursor (0,2);
        lcd_print ("������:");
    }
			 
    // ������� ������� ����� �� ������� �������
    if (DeviceState.alarm_lamp)
        sprintf(stroka,"�����");
    if ( !(DeviceState.alarm_lamp) )
        sprintf(stroka,"     ");
    set_cursor (7,2);
    lcd_print (stroka);
    // ������� ������� ������ �� �� �������
    if (DeviceState.alarm_uf)
        sprintf(stroka," ��");
    if (!(DeviceState.alarm_uf))
        sprintf(stroka,"   ");
    lcd_print (stroka);
    // ������� ������� ����������� �� �������
    if (DeviceState.alarm_temp)
        sprintf(stroka," ����");
    if (!(DeviceState.alarm_temp))
        sprintf(stroka,"     ");
    lcd_print (stroka);
    // ���� ���� ����� �������� ����� ������� ���������� � ��������
    if (eeprom.Sens.Board) {
        //������� �����������			
        set_cursor (0,3);
        //lcd_print ("                    ");
        if (eeprom.Sens.Temp) {
            set_cursor (0,3);
            if (temperatura<10) {
                sprintf(stroka,"t=%d ",temperatura); 
            } else {
                sprintf(stroka,"t=%d",temperatura); 
            }
        lcd_print (stroka);
		}
        //������� ������� ��		
        if (eeprom.Sens.UV) {
            set_cursor (5,3);
            if (uf_level < 10 || (mbSlave.RegOut[workFlags] & UV_ON_WORKFLAGS_MASK == 0)) {
                sprintf(stroka,"��=%d%%  ",0);
            } else if (uf_level < 100) {
                sprintf(stroka,"��=%d%% ",uf_level);
            } else {
                sprintf(stroka,"��=%d%% ",uf_level);
            }
            lcd_print (stroka);
        } else {
            if (DeviceState.uf_on)
                sprintf(stroka,"�� ���");
            if (!(DeviceState.uf_on))
                sprintf(stroka,"�� ���");
                set_cursor (5,3);
                lcd_print (stroka);
		}
    } else {
        //������� ������ ��
        if (DeviceState.uf_on)
            sprintf(stroka,"�� ���");
        if (!(DeviceState.uf_on))
            sprintf(stroka,"�� ���");
        set_cursor (5,3);
        lcd_print (stroka);
    }
    //������� ������ ���
    if (DeviceState.uzg_on)
        sprintf(stroka,"��� ���");
    if (!(DeviceState.uzg_on))
        sprintf(stroka,"��� ���");
    set_cursor (13,3);
    lcd_print (stroka);
			
} // display_main_screen


	
//������� ������ ���� �������� ��������� �� ���������		
void display_menu(void)				
{
    volatile uint8_t temp;
    uint8_t line, menu_num, menu_ind, STATE;
    volatile uint8_t max_item, cursor;
    menu_num = 1;//��������� �� root_menu
    menu_ind = 0;//���������� � 0 ������
    cursor=0;
    lcd_clear();
    STATE = REDRAW | KEY | MENU_BACK;
    //����� �������� ���� 
    return_counter=0;
    
    while(1) {
        if (return_counter>RETURN_FROM_MENU)
            return;
        //���� ���������� ���� ���������� ������� - ������� ���� ��� �������
        if (STATE & REDRAW) {
            lcd_clear();
            for(line = 0; line < MAX_LINES; line++) { // ���� ������� � 4 ������
                // ���� ���������� ����� � ����� �������
                if(menu[menu_num][menu_ind].menu_item) {
                    // ������������ ����� ������� � ������ ����
                    max_item = menu_ind;
                    // �� ������� ���											
                    set_cursor (0,line);										
                    lcd_print  ( menu[menu_num][menu_ind++].menu_item);
                } else {
                    menu_ind--;
                    break;
                }
            }
            STATE &= (~REDRAW);
        }
        // ������� ������
        if (STATE & KEY) {
            set_cursor (19,temp);
            lcd_print(" "); //������� ������ �� ������ ����� �������
            set_cursor (19,cursor);//������������� ������ �� ����� ������
            lcd_print("~"); //������� �������
            STATE &= (~KEY);
            return_counter = 0;
        }
        
        Keyboard();
        temp = cursor; // ���������� ���������� ������� �������
        switch (kb) {
            case UP_PRESS:
                if (cursor>0)
                    cursor--;
                // ���� ������� �� ������ ����������
                STATE |= KEY;
                break;
                    
            case DOWN_PRESS:
                if (cursor < max_item)
                    cursor++; 
                STATE|=KEY; // ���� ������� �� ������ ����������
                break;

            case ENTER_PRESS:
                //���� �� ��������� �� ���� � ��� ����� ������,������ �� �� ��������� � �� �������� �����-���� ��������
                STATE &= (~MENU_BACK);
                break;
                
            case ENTER_RELEASE: // �� ��������� ������ - �������� �����
                if ( !(STATE & MENU_BACK) ) { //���� ��������� � ���� � ��������� ������-�� ������������
                    // ���� ���� �������-������� ����
                    if (menu[menu_num][cursor].podmenu != 0) {
                        menu_num = menu[menu_num][cursor].podmenu;
                        menu_ind = 0;
                        cursor = 0;
                        //���� ������� �� ������ � ������ ������� ����������
                        STATE = REDRAW | KEY;
                        //���� ������� ��� ���� ���� �������-��������
                    } else if (menu[menu_num][cursor].function != 0) {
                        // ������� ���������� ��� ����� ���� ���� ������������
                        menu_num = menu[menu_num][cursor].function(menu_num);
                        menu_ind = 0;
                        cursor = 0;
                        // //���� ������� �� ������,���.������� � �������� � ���� ����������
                        STATE = REDRAW | KEY | MENU_BACK;
                    }
                } else {
                    STATE &= (~MENU_BACK);//���������� ���� �������� � ����
                }
                break;
                    
            case ENTER_HOLD: 		// �� ����������� ������� - �������  
                //���� ��������� � ����-�� ������������
                if ( !(STATE & MENU_BACK) ) {
                    //���� ���� ���� ��������� �� �������
                    if(menu[menu_num][cursor].prevmenu != 0) {
                        //����� ���� � �������- ���� ���������� ���� 
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
//										������� ������� �� �������
/////////////////////////////////////////////////////////////////////////////////
		

char alarms(char menu)
{
    uint8_t  i,j,STATE;
    uint8_t  dead_lamps;
    uint16_t badlamp;
    char string[21];
    
    dead_lamps=0;
    lcd_clear();
    STATE=REDRAW|KEY|MENU_BACK;
    //����� 
    lcd_clear();
    set_cursor (0,0);
    lcd_print("��������� �����");
    set_cursor (0,1);
    return_counter=0;
    while(1)
    {
        if (return_counter>RETURN_FROM_MENU)
            return 5;
        if(STATE &  REDRAW) { //���� ���������� ���� ���������� �������- ������� ���� ��� �������
            for(i=0;i<=EXP_BOARD;i++) { // ������������� �����
                badlamp=get_lamps(i);	//�������� ���������� � ������ �������������� � 1 �����.��������� ������
                for (j=0;j<10;j++) { //������� ��� ����������
                    if(badlamp&0x01) {	//���� �������� ��������� �����
                        if(dead_lamps<=20) { //���� ��� �� ������� ��� 3 ������ ������ ��������:)
                            if ((dead_lamps==6)||(dead_lamps==13)||(dead_lamps==20)) 
                                sprintf(string,"%02d",(i*10+1+j)); // ������� �� �� �������
                            else  
                                sprintf(string,"%02d ",(i*10+1+j)); // ������� �� �� �������
                            lcd_print  (string);
                            dead_lamps++;		//����������� ������� ��������� ����
                        }
                    }// ���� � ��� ��������� 7 ��������� ����(������� � 0 ������ 6) ��������� ������ �� 2(���.���.� 0) ������
                    if (dead_lamps==7) 
                        set_cursor(0,2); 
                    if (dead_lamps==14)
                        set_cursor(0,3);// ���� � ��� ��������� 14 ��������� ����  ��������� ������ �� 3 ������
                    
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
                
                case ENTER_RELEASE: 		// �� ��������� ������ - ������� �� ������ ������
                    {
                            
                        
                    }
                break;
                    
                case ENTER_HOLD: 		// �� ����������� ������� - ������ ������
                    {
                        return 5; //5-��� ������ ������ ���� ������ �� ������	
                    }
                break;
        
        }//switch end
        
    } // while(1)
        
}
/////////////////////////////////////////////////////////////////////////////////////////
//                  ����� ������ 
/////////////////////////////////////////////////////////////////////////////////////////
char alarmsRS(char menu)
	{
            uint8_t   i,STATE;
            STATE = 0;
			volatile	uint32_t delaycnt=0x1FFFF; 
			char string[21];
			DeviceState.alarm_comm=0; 
			lcd_clear();
			
			//����� 
			lcd_clear();
			set_cursor (0,0);
		  sprintf(string,"RS485: EXP.%d UFT.%d",EXP_BOARD,eeprom.Sens.Board);
			lcd_print(string);
			set_cursor(0,1);
			
			RxDataReady=false; 
			if(EXP_BOARD>0)//���� ���� ����� ����������
							{
								for(i=1;i<=EXP_BOARD;i++)
									{
										//BUF1[0]= i;//����� ����� ����������
										//BUF1[1]=LCOUNT;		//������ ����� ����
										//PACK_SEND(BUF1,2);//���������� � �������
										while( RxDataReady!=true){delaycnt--;if(delaycnt==0)break;}//���� ������� ������,���� ����� �� ������-������ �� ����

										if(delaycnt==0){ sprintf(string,"addr.%#x",i);lcd_print(string);DeviceState.alarm_comm=1;}	//���������� ���� ������ ����
										RxDataReady=false;
									}
								 
							}					
			 
								
			return_counter=0; 
			while(1)
			{
					if (return_counter>RETURN_FROM_MENU) return 5;
					if(STATE &  REDRAW) //���� ���������� ���� ���������� �������- ������� ���� ��� �������
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
						  
						 
						 case ENTER_RELEASE: 		// �� ��������� ������  
								{
									 
								}
						 break;
								
						 case ENTER_HOLD: 		// �� ����������� ������� - ������� � ��������� ����
								{
									return 5;
								}
						 break;
					
					}//switch end
				
			}
				
	}	
char config(char menu)
{
    uint8_t   STATE;
    uint8_t expBoard = eeprom.LampsQty / 10;

    char string[21];


    lcd_clear();
    STATE=KEY;
    //����� 
    lcd_clear();
    set_cursor (0,0);
    lcd_print("������������ �������");
    set_cursor (0,1);
    sprintf(string,"����� ����: %d",eeprom.LampsQty);
    lcd_print(string);
                        
    sprintf(string,"���� ����������: %d",expBoard);
    set_cursor (0,2);
    lcd_print(string);
    
    set_cursor (0,3);
    if (eeprom.Sens.Board)
        lcd_print("����� �� � ����: +");
    else
        lcd_print("����� �� � ����: -");

    return_counter=0;
    while(1)
    {
            if (return_counter>RETURN_FROM_MENU) return 3;
            if(STATE &  REDRAW) //���� ���������� ���� ���������� �������- ������� ���� ��� �������
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
                    
                    
                    case ENTER_RELEASE: 		// �� ��������� ������  
                        {
                                
                        }
                    break;
                        
                    case ENTER_HOLD: 		// �� ����������� ������� - ������� � ��������� ����
                        {
                            return 3;
                        }
                    break;
            
            }//switch end
        
    }
        
        
}
char narab(char menu)
{
    uint8_t  STATE;
    uint8_t  cursor;
    volatile uint8_t  plata=0;//����� ����� � ������� ��������� ��������� 
    uint8_t  lamps_in_board;
    lamps_in_board = eeprom.LampsQty;
    if (lamps_in_board > 10) {
        lamps_in_board = 0;
    }
    char string[21];
    cursor=0;
    lcd_clear();
    STATE=REDRAW|KEY|MENU_BACK;
    //����� �������� ���� 
    lcd_clear();
    set_cursor (0,0);
    lcd_print("��������� �����");
    set_cursor (0,1);
    lcd_print("����� ");
    set_cursor (0,2);		// 
    lcd_print("����� "); 
    set_cursor(0,3);
    lcd_print("����� ");
    return_counter=0;
    while(1) {
        if (return_counter>RETURN_FROM_MENU)
            return 2;
        if (STATE &  REDRAW) { //���� ���������� ���� ���������� �������- ������� ���� ��� �������
                lamps_in_board=get_lamps_count(plata);//�������� ���������� ���� �� �����
                //narabotka(plata,hcounter); //��������� ��������� �� ����� (����� ��������� �-� �������� � ������� ����������� �� ����������)
                set_cursor(15,0);
                sprintf(string,"  %02d",plata);//������� ����� ����� i*10+1+j
                lcd_print(string);
                                            
                set_cursor(0,1);
                sprintf(string,"����� %02d   %05d  ",plata*10 +cursor+1,hourcounter[(plata*10+cursor)]);//hcounter[(cursor)]);//������� ����� ����� � ���������
                lcd_print(string);
                
                set_cursor(0,2);
                if((cursor+1)<(lamps_in_board))
                    {
                        sprintf(string,"����� %02d   %05d  ",plata*10+cursor+2,hourcounter[(plata*10+cursor+1)]);//hcounter[(cursor+1)]);//������� ����� ����� � ���������
                    }
                    else sprintf(string,"                    ");
                lcd_print(string);
                
                set_cursor(0,3);
                if((cursor+2)<(lamps_in_board))
                    {
                            sprintf(string,"����� %02d   %05d  ",plata*10+cursor+3,hourcounter[(plata*10+cursor+2)]);//hcounter[(cursor+2)]);//������� ����� ����� � ���������
                    }
                    else sprintf(string,"                    ");
                lcd_print(string);
            
                STATE&=(~REDRAW);
                return_counter=0;
        }
        
        if(STATE&KEY)//������� ������
        {
                
        }
        
        Keyboard();
        switch (kb)
        {
        
                case UP_PRESS: // 
                    {
                            if (cursor>0) cursor--; //���� ������� ������� ������� �� ����- ���������
                            else 
                            {
                                if(plata>0)
                                {
                                    plata--;
                                lamps_in_board=get_lamps_count(plata);
                                cursor=lamps_in_board-1-2;//lamps_in_board(plata)-2;
                                }
                                
                            }
                            STATE|=REDRAW;//���� ������. ������ ����������
                    }
                break;
                    
                case DOWN_PRESS: // 
                    {		//���� ������� ������� �� ������ ������������� ����� ������������� ����
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
                            STATE|=REDRAW;//���� ������. ������  ����������
                    }
                break;
                
                case ENTER_RELEASE: 		// �� ��������� ������ - ������� �� ������ ������
                    {
                            if(plata<EXP_BOARD)plata++; //���������� ���� �����
                            else plata=0;
                            cursor=0;
                            STATE|=REDRAW;
                    }
                break;
                    
                case ENTER_HOLD: 		// �� ����������� ������� - ������ ������
                    {
                            return 2; //2-��� ������ ������ ���� ������ �� ������
                    }
                break;
        
        }//switch end
    }
}	
char reset_nar_all(char menu)//����� ��������� ���� ����������� ���� � �������
	{
		uint8_t i;
			for(i=0;i<eeprom.LampsQty;i++)// ������� ��� �� ������� �����
					{
							hourcounter[i]=0;//����� ���������
					}
					DeviceLog.res_all++;
					save_pars();//��������� ��������� ��������� � ������� ��������� �� ������� �����

										
									
			return 6;
	}
	
char reset_nar_one(char menu)//����� ��������� ����� ����� ���������� �� ������
	{
			uint8_t   STATE;
			uint8_t  max_lamps=0;
			uint8_t  cursor=1;
			char string[21];
			max_lamps=eeprom.LampsQty;//������������ ����� ���� � ������

			
			lcd_clear();
			STATE=REDRAW;
			//����� 
			lcd_clear();
			set_cursor (0,0);
			lcd_print("������� �����");
			set_cursor (0,1);
			lcd_print("����� N ");
			set_cursor(0,2);
			lcd_print("�������   ~ �����");
			set_cursor(0,3);
			lcd_print("��������� ~ ������");
			return_counter=0;
			while(1)
			{
					if (return_counter>RETURN_FROM_MENU) return 6;
					if(STATE &  REDRAW) //���� ���������� ���� ���������� �������- ������� ���� ��� �������
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
						 
						 case ENTER_RELEASE: 		// �� ��������� ������ - ���������� ������� �� ����� ��������� ����� N � �������
								{
									
									
											hourcounter[cursor-1]=0;
											DeviceLog.res_one++;
											save_pars();
									
									return 6; //6-��� ������ ������ ���� ������ �� ������	
								
									
								}
						 break;
								
						 case ENTER_HOLD: 		// �� ����������� ������� - ������� � ��������� ����
								{
									return 6;
								}
						 break;
					
					}//switch end
				
			}
			
		 		
	}
	
	
char logg(char menu)
{
			uint8_t  STATE;
//			uint8_t  cursor;
			 
			char string[21];
//			cursor=0;
			lcd_clear();
			STATE=REDRAW|KEY|MENU_BACK;
			//����� �������� ���� 
			lcd_clear();
			set_cursor (0,0);
			sprintf(string,"���������: %03d",DeviceLog.oncounter);
			lcd_print(string);
			set_cursor (0,1);
			sprintf(string,"������� ������:%03d",DeviceLog.res_all);
			lcd_print(string);
			set_cursor (0,2);		// 
			sprintf(string,"������� ����.: %03d",DeviceLog.res_one);
			lcd_print(string); 
		  set_cursor(0,3);
		  sprintf(string,"������� ����:  %03d",DeviceLog.res_log);
			lcd_print(string);
			return_counter=0;
		 	while(1)
			{
					if (return_counter>RETURN_FROM_MENU) return 4;
					if(STATE &  REDRAW) //���� ���������� ���� ���������� �������- ������� ���� ��� �������
					{
							 
							return_counter=0;
							STATE&=(~REDRAW);
					}
					
					if(STATE&KEY)//������� ������
					{
						 
					}
					
				    Keyboard();
					switch (kb)
					{
					
						 case UP_PRESS: // 
								{
									  
									 STATE|=REDRAW;//���� ������. ������ ����������
								}
						 break;
								
						 case DOWN_PRESS: // 
								{		 
										STATE|=REDRAW;//���� ������. ������  ����������
								}
						 break;
						 
						 case ENTER_RELEASE: 		// �� ��������� ������ - ������� �� ������ ������
								{
									  
									 STATE|=REDRAW;
								}
						 break;
								
						 case ENTER_HOLD: 		// �� ����������� ������� - ������ ������
								{
										return 4; //4-��� ������ ������ ���� ������ �� ������
								}
						 break;
					
					}//switch end
				
			}
	}	
//����� ���� ������
char logg_reset(char menu)
{
    uint8_t  STATE;
    volatile uint8_t  cursor,cntr;
    volatile uint16_t kod,multy; 
    char string[21];
    cursor=0;kod=0;
    multy=1;cntr=0;
    lcd_clear();
    STATE=REDRAW|KEY|MENU_BACK;
    //����� �������� ���� 
    lcd_clear();
    set_cursor (0,0);
    sprintf(string,"������� ���:" );
    lcd_print(string);
    return_counter=0;
    while(1)
    {
        if ( return_counter > RETURN_FROM_MENU )
            return menu;
        //���� ���������� ���� ���������� �������- ������� ���� ��� �������
        if (STATE &  REDRAW) { 
            return_counter=0; 
            set_cursor (0,1);
            sprintf(string,"  %04d",(kod+multy*cursor));
            lcd_print(string);
        
            STATE &= (~REDRAW);
        }
        
        if (STATE & KEY) //������� ������
        { }
        
        Keyboard();
        switch (kb) {
        
            case UP_PRESS: // 
                if (cursor < 9)
                    cursor++;
                    STATE |= REDRAW;//���� ������. ������ ����������
                break;

            case DOWN_PRESS: // 
                if (cursor>0)
                    cursor--;	
                STATE |= REDRAW; // ���� ������. ������  ����������
                break;
                
            case ENTER_RELEASE: // �� ��������� ������ - ��������� ��������� ��������� � �������
                kod += (cursor * multy);
                multy = multy * 10;
                cntr++;
                cursor = 0;
                if (cntr == 4) {
                    if ((kod == VALID_RESET_CODE) && (menu == 4))  {
                        DeviceLog.oncounter = 0;
                        DeviceLog.res_all = 0;
                        DeviceLog.res_log++;
                        DeviceLog.res_one = 0;
                        save_pars();
                        return menu;
                    } else if ((kod == CONF_PASSWORD) && (menu == 3)) {
                        return 8;
                    }
                    cntr = 0;
                    kod = 0;
                    multy = 1;
                    cursor = 0;
                }
                STATE|=REDRAW;
                break;
                    
            case ENTER_HOLD: // �� ����������� ������� 
                //4-��� ������ ������ ���� ������ �� ������
                return menu; 
                break;
        }//switch end
    }
}	 

	
char setup_device(char menu)
{
    char string[21];

    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    sprintf(string,"������� ��:" );
    lcd_print(string);

    menu = set(menu, &(eeprom.UFmin), 0, 99, nullName);
    extern struct MBRegSt mbSlave;
    mbSlave.RegOut[UFmin] = eeprom.UFmin;
    
    return menu;
}

char set (char menu, volatile uint16_t* what, uint16_t min, uint16_t max, char* name[]) {
    uint8_t STATE;
    STATE = REDRAW | KEY | MENU_BACK;
    char string[21];
    volatile uint8_t cursor;
    cursor = *what;
    set_cursor(0,2);
    lcd_print("�������   ~ ������.");
    set_cursor(0,3);
    lcd_print("��������� ~ ������");
    return_counter = 0;
    while(1) {
        if (return_counter > RETURN_FROM_MENU)
            return menu;
        // ���� ���������� ���� ���������� ������� - ������� ���� ��� �������
        if(STATE &  REDRAW) {
            return_counter=0; 
            set_cursor (0,1);
            if (name[0] == 0) {
                sprintf(string,"  %04d",( cursor));
                lcd_print(string);
            } else {
                lcd_print(name[cursor]);
            }
            
            STATE&=(~REDRAW);
        }

        if(STATE&KEY)//������� ������
        { }

        Keyboard();
        switch (kb) {
            case UP_PRESS: // 
                if (cursor < max)
                    cursor++;
                STATE|=REDRAW;//���� ������. ������ ����������
                break;
            
            case UP_HOLD: // 
                if ( cursor < (max-9) ) 
                    cursor = cursor + 10;
                else if (cursor < max)
                    cursor++;
                STATE |= REDRAW;//���� ������. ������ ����������
                break;
                
            case DOWN_PRESS: // 
                if (cursor > min)
                    cursor--;	
                STATE |= REDRAW; //���� ������. ������  ����������
                break;
                
            case DOWN_HOLD: // 
                if (cursor > min + 10)
                    cursor = cursor - 10;
                else if (cursor>min)
                    cursor--;
                STATE|=REDRAW;//���� ������. ������  ����������
                break;
            
            case ENTER_RELEASE: 		// �� ��������� ������ - ������� �� ������ ������
                *what = cursor;
                return menu; 
                break;
                
            case ENTER_HOLD:
                return menu;
                break;
        } // switch end
    }
}



		
		 
 
char reset_alarms(char menu) {
    DeviceState.alarm_uf=0;
    DeviceState.alarm_lamp=0; 
    DeviceState.alarm_temp=0;
    DeviceState.alarm_comm=0;
    return 5;
}
    

char resetMaxUF(char menu) {
    eeprom.UFmax = UF100PERCENT_BEGIN;
    mbSlave.RegOut[UFmax] = UF100PERCENT_BEGIN;
    return 3;
}

char setLampsQty (char menu)
{
    char string[21];
    
    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    sprintf(string,"���������� ����:" );
    lcd_print(string);

    menu = set(menu, &(eeprom.LampsQty), 1, 112, nullName);
    mbSlave.RegOut[LampsQtyMB] = eeprom.LampsQty;
    
    return menu;    
}

char setName (char menu)
{
    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    lcd_print("������:");

    menu = set(menu, &(eeprom.Name), 0, 26, models);
    
    return menu;    
}

char setTmax (char menu)
{
    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    lcd_print("���� �����������:");

    menu = set(menu, &(eeprom.Tmax), 1, 99, nullName);
    mbSlave.RegOut[Tmax] = eeprom.Tmax;
    
    return menu;        
}
char setTrec (char menu)
{
    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    lcd_print("T���������� �������:");

    menu = set(menu, &(eeprom.Trec), 1, 99, nullName);
    
    return menu;        
}

char setSensBoard (char menu)
{
    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    lcd_print("����� ��������:");
    uint16_t tmp;
    tmp = eeprom.Sens.Board;
    menu = set(menu, &(tmp), 0, 1, exist);
    eeprom.Sens.Board = tmp;
    if (!eeprom.Sens.Board) {
        eeprom.Sens.Temp = false;
        eeprom.Sens.UV = false;
    };
    
    return menu;      
}
char setTempSense (char menu)
{
    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    lcd_print("������ �����������:");
    uint16_t tmp;
    tmp = eeprom.Sens.Temp;
    menu = set(menu, &(tmp), 0, 1, exist);
    eeprom.Sens.Temp = tmp;
    if (eeprom.Sens.Temp) {
        eeprom.Sens.Board = true;
    };
    return menu;
}
char setUVsense (char menu)
{
    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    lcd_print("������ ��:");
    uint16_t tmp;
    tmp = eeprom.Sens.UV;
    menu = set(menu, &(tmp), 0, 1, exist);
    eeprom.Sens.UV = tmp;
    if (eeprom.Sens.UV) {
        eeprom.Sens.Board = true;
    };
    return menu;
}

char setupModbus (char menu)
{
    setAdr(menu);
    setBoudrate(menu);
    setParityEn(menu);
    if (eeprom.uartset.bits.parityEn) {
        setParity(menu);
    }
    setStopBits(menu);
    return menu;
}

char setAdr (char menu)
{
    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    lcd_print("����� � ����:");
    uint16_t tmp;
    tmp = eeprom.mbadr;
    menu = set(menu, &(tmp), 1, 255, nullName);
    eeprom.mbadr = tmp;
    mbSlave.RegOut[mbadr] = tmp;
    return menu;
}
char setBoudrate (char menu)
{
    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    lcd_print("�������� (����):");
    uint16_t tmp;
    tmp = eeprom.uartset.bits.boud;
    menu = set(menu, &(tmp), 0, 7, boudrate);
    eeprom.uartset.bits.boud = tmp;
    mbSlave.RegOut[uartset] &= ~(0b111 << 3);
    mbSlave.RegOut[uartset] |= (tmp << 2);
    return menu;
}
char setParityEn (char menu)
{
    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    lcd_print("�������� ����.:");
    uint16_t tmp;
    tmp = eeprom.uartset.bits.parityEn;
    menu = set(menu, &(tmp), 0, 1, exist);
    eeprom.uartset.bits.parityEn = tmp;
    mbSlave.RegOut[uartset] &= ~(0b1 << 0);
    mbSlave.RegOut[uartset] |= (tmp << 0);
    return menu;
}
char setParity (char menu)
{
    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    lcd_print("�������� ����.:");
    uint16_t tmp;
    tmp = eeprom.uartset.bits.parityEven;
    menu = set(menu, &(tmp), 0, 1, parity);
    eeprom.uartset.bits.parityEven = tmp;
    mbSlave.RegOut[uartset] &= ~(0b1 << 1);
    mbSlave.RegOut[uartset] |= (tmp << 1);
    return menu;
}
char setStopBits (char menu)
{
    lcd_clear();
    //����� �������� ����
    lcd_clear();
    set_cursor(0,0);
    lcd_print("���-�� ���� ���:");
    uint16_t tmp;
    tmp = eeprom.uartset.bits.stopBitsMinus1 + 1;
    menu = set(menu, &(tmp), 1, 2, nullName);
    eeprom.uartset.bits.stopBitsMinus1 = tmp - 1;
    mbSlave.RegOut[uartset] &= ~(0b1 << 2);
    mbSlave.RegOut[uartset] |= ( (tmp - 1) << 2);
    return menu;
}

