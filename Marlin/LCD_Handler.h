/*
LCD_Handler.h - A place to hold all interactions between LCD and printer. It is called from Marlin_main.cpp when using genie.DoEvents().
Last Update: 15/01/2018
Author: Alejandro Garcia (S3mt0x)
*/

#ifdef SIGMA_TOUCH_SCREEN

#ifndef LCD_HANDLER_H_
#define LCD_HANDLER_H_


#include "genieArduino.h"
#include "Touch_Screen_Definitions.h"
#include "Marlin.h"
#include "Configuration.h"
#include "stepper.h"
#include "temperature.h"
#include "SD_ListFiles.h"
#include "LCD_FSM.h"

void myGenieEventHandler();

void myGenieEventHandler(void) //Handler for the do.Events() function
{
	genieFrame Event;
	genie.DequeueEvent(&Event);
	//static long waitPeriod = millis();
	
	//If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
	if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
	{
		
		if (Event.reportObject.object == GENIE_OBJ_USERBUTTON) //Userbuttons to select GCODE from SD
		{
			if(!lcd_busy)LCD_FSM_input_buton_flag = Event.reportObject.index;
		}
		//USERBUTTONS------------------------------------------------------

		
		//FORMS--------------------------------------------------------
		if (Event.reportObject.object == GENIE_OBJ_FORM)
		{
			char buffer[256];
			switch(Event.reportObject.index){
				
				case FORM_SDLIST:
					genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_SDLIST_FOLDERBACK,0);
					genie.WriteObject(GENIE_OBJ_USERIMAGES, USERIMAGE_SDLIST_FOLDERFILE,0);
					screen_sdcard = true;
					flag_sdlist_goinit = true;
				break;
				
				case FORM_SDPRINTING:
					
					is_on_printing_screen = true;
					surfing_utilities = false;
					genie.WriteStr(STRING_SDPRINTING_GCODE,namefilegcode);
					flag_sdprinting_dararefresh = true;
				break;
				
				case FORM_MAIN:
					screen_sdcard = false;
					surfing_utilities=false;
					surfing_temps = false;
					processing_z_set = 255;
					touchscreen_update();
					SERIAL_PROTOCOLPGM("Surfing 0\n");
				break;
				
				case FORM_UTILITIES:
					
					surfing_utilities=true;
					SERIAL_PROTOCOLPGM("Surfing 1\n");
				break;
				
				case FORM_TEMP:					
					surfing_temps = true;
				break;
				
				case FORM_UTILITIES_FILAMENT_PURGE:
					
					SERIAL_PROTOCOLPGM("Purge Filament\n");
					if(purge_extruder_selected == 0) {
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,0);
					}
					else {
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,1);
					}
					
					sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
					genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_LEFTTARGET,buffer);	Serial.println(buffer);
					sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
					genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_RIGHTTARGET,buffer);	Serial.println(buffer);
				break;
				
				case FORM_INFO_UI:
					sprintf(buffer, "%s%s",VERSION_STRING,BUILD_DATE);
					genie.WriteStr(STRING_INFO_UI_VERSION,buffer);
					if(UI_SerialID0 || UI_SerialID1 || UI_SerialID2){
						sprintf(buffer, "%03d.%03d%03d.%04d",UI_SerialID0, (int)(UI_SerialID1/1000),(int)(UI_SerialID1%1000), UI_SerialID2);
						//sprintf(buffer, "%03d.%03d%03d.%04d",1020, 1151,1021, 10002);
						genie.WriteStr(STRING_INFO_UI_SERIALID,buffer);
						}else{
						genie.WriteStr(STRING_INFO_UI_SERIALID,UI_SerialID);
					}
					sprintf(buffer, "%d h",log_hours_print);
					//Serial.println(buffer);
					genie.WriteStr(STRING_INFO_PRINTINGTIME,buffer);
				break;
			}
		}
	}
	else if (Event.reportObject.cmd == GENIE_PING) {

		if (Event.reportObject.object == GENIE_DISCONNECTED) {
			// If the display gets disconnected, turn on LED
			Serial.println("Display got disconnected");

			} else if (Event.reportObject.object == GENIE_READY) {
			// If the display gets disconnected, turn off LED
			Serial.println("Display is ready");

			} else if (Event.reportObject.object == GENIE_ACK) {
			// manual ping response - Display is Connected - Do something if required

			} else if (Event.reportObject.object == GENIE_NAK) {
			// manually ping response - Display is NOT Connected - Do something if required

		}

	}
}

#endif
#endif /* INCLUDE */

