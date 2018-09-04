/*
LCD_Handler.h - A place to hold all interactions between LCD and printer. It is called from Marlin_main.cpp when using genie.DoEvents().
Last Update: 01/08/2018
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
	display.DequeueEvent(&Event);
	//static long waitPeriod = millis();
	
	//If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
	if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
	{
		
		//USERBUTTONS------------------------------------------------------
		if (Event.reportObject.object == GENIE_OBJ_USERBUTTON) //Userbuttons 
		{
			if(!lcd_busy)LCD_FSM_input_buton_flag = Event.reportObject.index;
		}
		//4DBUTTONS------------------------------------------------------
		if (Event.reportObject.object == GENIE_OBJ_4DBUTTON) //4Dbuttons 
		{
			if(!lcd_busy)LCD_FSM_input_buton_flag = 300 + Event.reportObject.index;
		}
		//SLIDER------------------------------------------------------
		if (Event.reportObject.object == GENIE_OBJ_ISMARTSLIDER) //SLIDER
		{	
			led_brightness = (Event.reportObject.data_msb << 8) + Event.reportObject.data_lsb;	
					
			switch(Event.reportObject.index){
								
				case SMARTSLIDER_PRINTERSETUP_LED_BRIGHTNESS:
				
				Serial.print(F("BRIGHTNESS: "));
				Serial.println(led_brightness);
				analogWrite(RED,led_brightness);
				analogWrite(GREEN,led_brightness);
				analogWrite(BLUE,led_brightness);
				
				break;
				
				default:
				break;
			}
		}
		
		//FORMS--------------------------------------------------------
		if (Event.reportObject.object == GENIE_OBJ_FORM)
		{
			
			switch(Event.reportObject.index){
				/*
				case FORM_SDLIST:
					display_ButtonState( BUTTON_SDLIST_FOLDERBACK,0);
					display.WriteObject(GENIE_OBJ_USERIMAGES, USERIMAGE_SDLIST_FOLDERFILE,0);
					screen_sdcard = true;
					bitSet(flag_sdlist_resgiter,flag_sdlist_resgiter_goinit);
				break;
				*/
				case FORM_SDPRINTING:
					
					is_on_printing_screen = true;
					surfing_utilities = false;
					display.WriteStr(STRING_SDPRINTING_GCODE,namefilegcode);
					bitSet(flag_sdprinting_register,flag_sdprinting_register_datarefresh);
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
						display_ButtonState(BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,1);
						display_ButtonState(BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,0);
					}
					else {
						display_ButtonState(BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,0);
						display_ButtonState(BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,1);
					}
										
					display.WriteObject(GENIE_OBJ_CUSTOM_DIGITS,CUSTOMDIGITS_UTILITIES_FILAMENT_PURGE_LEFTTARGET,int(degHotend(0)));
					display.WriteObject(GENIE_OBJ_CUSTOM_DIGITS,CUSTOMDIGITS_UTILITIES_FILAMENT_PURGE_RIGHTTARGET,int(degHotend(1)));
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

