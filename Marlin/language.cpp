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
#include "genieArduino.h"
#include "Touch_Screen_Definitions.h"
#include "Marlin.h"
#include "Configuration.h"
#include "stepper.h"
#include "temperature.h"
#include "language.h"

char idiom[9][50]= {0};

void changeLanguage(int i){
	language = i;
	enquecommand_P(PSTR("M500"));
	updateLanguage();
}

void updateLanguage(){
	Serial.println(language);
	
	if(language == 1){
		strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "English");
		
		}
	else if (language == 2){
		strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "Polish");
		
		}
	else if (language == 3){
		strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "French");	
		
		}
		
	else if (language == 4){
		strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "German");	
		
		}
	else if (language == 5){
		strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "Spanish");
		
		}
	else if (language == 6){
		strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "Russian");
		
		}
	else if (language == 7){
		strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "Italian");
		
		}
	else if (language == 8){
		strcpy(idiom[IDIOM_CURRENT_LANGUAGE], "Portuguese");
		
	}
}

