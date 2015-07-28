/*
 * language.cpp
 *
 * Created: 28/07/2015 10:01:26
 *  Author: xgomez
 *
 * In this file is controlled the control languages, the functions to change the language and update the texts depending on the selected language.
 * To add new sentences is necessary add in every language in the updateLanguage() function, and add in the language.h the #define to save the position in the matrix of idiom where is saved the sentences
 * 
 */ 

/*
#include "genieArduino.h"
#include "Touch_Screen_Definitions.h"
#include "Marlin.h"
#include "Configuration.h"
#include "stepper.h"
#include "temperature.h"
#include "language.h"

 char  idiom[50][50]   = {0};

void changeLanguage(int i){
	language = i;
	enquecommand_P(PSTR("M500"));
	updateLanguage();
}

void updateLanguage(){
	Serial.println(language);
	
	if(language == 1){
		//idiom[IDIOM_CURRENT_LANGUAGE] = "English";
		//strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "English");
		//strcpy(idiom[2], LANGUAGE);
		/*strcpy(idiom[3],WELCOME_MSG MACHINE_NAME);
		strcpy(idiom[4],MSG_SD_INSERTED);
		strcpy(idiom[5],MSG_SD_REMOVED);
		strcpy(idiom[6],MSG_MAIN);
		strcpy(idiom[7],MSG_AUTOSTART);
		strcpy(idiom[8],MSG_DISABLE_STEPPERS);
		strcpy(idiom[9],MSG_AUTO_HOME);
		strcpy(idiom[10],MSG_SET_HOME_OFFSETS);
		strcpy(idiom[11],MSG_SET_ORIGIN);
		strcpy(idiom[12],MSG_PREHEAT_PLA);
		strcpy(idiom[13],MSG_PREHEAT_PLA0);
		strcpy(idiom[14],MSG_PREHEAT_PLA1);
		strcpy(idiom[15],MSG_PREHEAT_PLA2);
		strcpy(idiom[16],MSG_PREHEAT_PLA012);
		strcpy(idiom[17],MSG_PREHEAT_PLA_BEDONLY);
		strcpy(idiom[18],MSG_PREHEAT_PLA_SETTINGS);
		strcpy(idiom[19],MSG_PREHEAT_ABS);
		strcpy(idiom[20],MSG_PREHEAT_ABS0);
		strcpy(idiom[21],MSG_PREHEAT_ABS1);
		strcpy(idiom[22],MSG_PREHEAT_ABS2);
		strcpy(idiom[23],MSG_PREHEAT_ABS012);
		strcpy(idiom[24],MSG_PREHEAT_ABS_BEDONLY);
		strcpy(idiom[25],MSG_PREHEAT_ABS_SETTINGS);
		strcpy(idiom[26],MSG_COOLDOWN);
		strcpy(idiom[27],MSG_SWITCH_PS_ON);
		strcpy(idiom[28],MSG_SWITCH_PS_OFF);
		strcpy(idiom[29],MSG_EXTRUDE);
		strcpy(idiom[30],MSG_RETRACT);
		strcpy(idiom[31],MSG_MOVE_AXIS);
		strcpy(idiom[32],MSG_MOVE_X);
		strcpy(idiom[33],MSG_MOVE_Y);*/
	
	/*	}
	else if (language == 2){
		/*strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "Polish");
		strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "English");
		strcpy(idiom[2],LANGUAGE);
		strcpy(idiom[3],WELCOME_MSG MACHINE_NAME);
		strcpy(idiom[4],MSG_SD_INSERTED);
		strcpy(idiom[5],MSG_SD_REMOVED);
		strcpy(idiom[6],MSG_MAIN);
		strcpy(idiom[7],MSG_AUTOSTART);
		strcpy(idiom[8],MSG_DISABLE_STEPPERS);
		strcpy(idiom[9],MSG_AUTO_HOME);
		strcpy(idiom[10],MSG_SET_HOME_OFFSETS);
		strcpy(idiom[11],MSG_SET_ORIGIN);
		strcpy(idiom[12],MSG_PREHEAT_PLA);
		strcpy(idiom[13],MSG_PREHEAT_PLA0);
		strcpy(idiom[14],MSG_PREHEAT_PLA1);
		strcpy(idiom[15],MSG_PREHEAT_PLA2);
		strcpy(idiom[16],MSG_PREHEAT_PLA012);
		strcpy(idiom[17],MSG_PREHEAT_PLA_BEDONLY);
		strcpy(idiom[18],MSG_PREHEAT_PLA_SETTINGS);
		strcpy(idiom[19],MSG_PREHEAT_ABS);
		strcpy(idiom[20],MSG_PREHEAT_ABS0);
		strcpy(idiom[21],MSG_PREHEAT_ABS1);
		strcpy(idiom[22],MSG_PREHEAT_ABS2);
		strcpy(idiom[23],MSG_PREHEAT_ABS012);
		strcpy(idiom[24],MSG_PREHEAT_ABS_BEDONLY);
		strcpy(idiom[25],MSG_PREHEAT_ABS_SETTINGS);
		strcpy(idiom[26],MSG_COOLDOWN);
		strcpy(idiom[27],MSG_SWITCH_PS_ON);
		strcpy(idiom[28],MSG_SWITCH_PS_OFF);
		strcpy(idiom[29],MSG_EXTRUDE);
		strcpy(idiom[30],MSG_RETRACT);
		strcpy(idiom[31],MSG_MOVE_AXIS);
		strcpy(idiom[32],MSG_MOVE_X);
		strcpy(idiom[33],MSG_MOVE_Y);*/
	/*	}
	else if (language == 3){
		//strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "French");	
		
		}
		
	else if (language == 4){
		/*strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "German");	
		strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "English");
		strcpy(idiom[2],LANGUAGE);
		strcpy(idiom[3],WELCOME_MSG MACHINE_NAME);
		strcpy(idiom[4],MSG_SD_INSERTED);
		strcpy(idiom[5],MSG_SD_REMOVED);
		strcpy(idiom[6],MSG_MAIN);
		strcpy(idiom[7],MSG_AUTOSTART);
		strcpy(idiom[8],MSG_DISABLE_STEPPERS);
		strcpy(idiom[9],MSG_AUTO_HOME);
		strcpy(idiom[10],MSG_SET_HOME_OFFSETS);
		strcpy(idiom[11],MSG_SET_ORIGIN);
		strcpy(idiom[12],MSG_PREHEAT_PLA);
		strcpy(idiom[13],MSG_PREHEAT_PLA0);
		strcpy(idiom[14],MSG_PREHEAT_PLA1);
		strcpy(idiom[15],MSG_PREHEAT_PLA2);
		strcpy(idiom[16],MSG_PREHEAT_PLA012);
		strcpy(idiom[17],MSG_PREHEAT_PLA_BEDONLY);
		strcpy(idiom[18],MSG_PREHEAT_PLA_SETTINGS);
		strcpy(idiom[19],MSG_PREHEAT_ABS);
		strcpy(idiom[20],MSG_PREHEAT_ABS0);
		strcpy(idiom[21],MSG_PREHEAT_ABS1);
		strcpy(idiom[22],MSG_PREHEAT_ABS2);
		strcpy(idiom[23],MSG_PREHEAT_ABS012);
		strcpy(idiom[24],MSG_PREHEAT_ABS_BEDONLY);
		strcpy(idiom[25],MSG_PREHEAT_ABS_SETTINGS);
		strcpy(idiom[26],MSG_COOLDOWN);
		strcpy(idiom[27],MSG_SWITCH_PS_ON);
		strcpy(idiom[28],MSG_SWITCH_PS_OFF);
		strcpy(idiom[29],MSG_EXTRUDE);
		strcpy(idiom[30],MSG_RETRACT);
		strcpy(idiom[31],MSG_MOVE_AXIS);
		strcpy(idiom[32],MSG_MOVE_X);
		strcpy(idiom[33],MSG_MOVE_Y);*/
/*		}
	else if (language == 5){
//		strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "Spanish");
		
		}
	else if (language == 6){
		//strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "Russian");
		
		}
	else if (language == 7){
		//strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "Italian");
		
		}
	else if (language == 8){
		//strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "Portuguese");
		
	}
}*/

