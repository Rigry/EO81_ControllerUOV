#include "stm32f10x.h"
#include <stdio.h>
void display_main_screen(void);
void display_menu(void);	

char narab(char menu);
char alarms(char menu);
char reset_alarms(char menu);
char config(char menu);
char reset_nar_all(char menu);
char reset_nar_one(char menu);
char logg(char menu);
char logg_reset(char menu);
char setup_device(char menu);
char alarmsRS(char menu);

char set (char menu, volatile uint16_t* what, uint16_t min, uint16_t max, char* name[]);
char setLampsQty(char menu);
char resetMaxUF(char menu);
char setName (char menu);
char setTmax (char menu);
char setTrec (char menu);
char setSensBoard (char menu);
char setTempSense (char menu);
char setUVsense (char menu);




