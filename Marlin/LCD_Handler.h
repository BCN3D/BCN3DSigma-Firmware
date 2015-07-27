/*
* LCD_Handler.h
* A place to hold all interactions between LCD and printer. It is called from Marlin_main.cpp when using genie.DoEvents()
* Created: 12/12/2014 12:48:04
* Author: Jordi Calduch (Dryrain)
*/

#ifdef SIGMA_TOUCH_SCREEN

#ifndef LCD_HANDLER_H_
#define LCD_HANDLER_H_

//Rapduch
#include "genieArduino.h"
#include "Touch_Screen_Definitions.h"
#include "Marlin.h"
#include "Configuration.h"
#include "stepper.h"
#include "temperature.h"
//#include "ultralcd.h"

extern bool cancel_heatup;
void myGenieEventHandler();
bool flag_filament_home= false;
bool flag_pause = false;
bool flag_resume = false;
bool flag_full_calib = false;
int print_setting_tool = 2;
float offset_x_calib = 0;
float offset_y_calib = 0;

//Created by Jordi Calduch for RepRapBCN SIGMA 12/2014
void myGenieEventHandler(void) //Handler for the do.Events() function
{
	genieFrame Event;
	genie.DequeueEvent(&Event);
	//static long waitPeriod = millis();
	int move_mm = 10;
	//If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
	if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
	{
		//*****Winbuttons*****
		#pragma region Winbuttons
		if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)                // If the Reported Message was from a winbutton
		{
			float modified_position;
			if (Event.reportObject.index == BUTTON_MOVE_AXIS_X)                              // If Winbutton1
			{
				Serial.println("Right");
				modified_position=current_position[X_AXIS]+move_mm;
				if (modified_position < X_MIN_POS)modified_position = X_MIN_POS;
				if (modified_position > X_MAX_POS)modified_position = X_MAX_POS;
				plan_buffer_line(modified_position, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 600, active_extruder);
				current_position[X_AXIS]=modified_position;
				//enquecommand_P(PSTR("G1 F1200 X100"));
			}
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_minusX)                              // If Winbutton0
			{
				Serial.println("Left");
				modified_position=current_position[X_AXIS]-move_mm;
				if (modified_position < X_MIN_POS)modified_position = X_MIN_POS;
				if (modified_position > X_MAX_POS)modified_position = X_MAX_POS;
				plan_buffer_line(modified_position, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 600, active_extruder);
				current_position[X_AXIS]=modified_position;
				//enquecommand_P(PSTR("G1 F1200 X150"));
			}
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_Y)                              // If Winbutton2
			{
				Serial.println("Down");
				modified_position=current_position[Y_AXIS]+move_mm;
				if (modified_position < Y_MIN_POS)modified_position = Y_MIN_POS;
				if (modified_position > Y_MAX_POS)modified_position = Y_MAX_POS;
				plan_buffer_line(current_position[X_AXIS], modified_position, current_position[Z_AXIS], current_position[E_AXIS], 600, active_extruder);
				current_position[Y_AXIS]=modified_position;
			}
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_minusY)                              // If Winbutton3
			{
				//if (millis() >= waitPeriod){
				Serial.println("Up");
				modified_position=current_position[Y_AXIS]-move_mm;
				if (modified_position < Y_MIN_POS)modified_position = Y_MIN_POS;
				if (modified_position > Y_MAX_POS)modified_position = Y_MAX_POS;
				plan_buffer_line(current_position[X_AXIS], modified_position, current_position[Z_AXIS], current_position[E_AXIS], 600, active_extruder);
				current_position[Y_AXIS]=modified_position;
				//waitPeriod=millis()+50;
				//}
			}
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_HOME)                              // If Winbutton4
			{
				//if (millis() >= waitPeriod){
				Serial.println("HOME");
				enquecommand_P((PSTR("G28")));
				//waitPeriod=millis()+50;
				//}
			}
			
			
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_Z)                              // If Winbutton7 Button Z
			{
				//if (millis() >= waitPeriod){
				Serial.println("ZUp");
				modified_position=current_position[Z_AXIS]+move_mm;
				if (modified_position < Z_MIN_POS)modified_position = Z_MIN_POS;
				if (modified_position > Z_MAX_POS)modified_position = Z_MAX_POS;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], modified_position, current_position[E_AXIS], 600, active_extruder);
				current_position[Z_AXIS]=modified_position;
				//waitPeriod=millis()+50;
				//}
			}
			
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_minusZ)                              // If Winbutton8
			{
				//if (millis() >= waitPeriod){
				Serial.println("ZDown");
				modified_position=current_position[Z_AXIS]-move_mm;
				if (modified_position < Z_MIN_POS)modified_position = Z_MIN_POS;
				if (modified_position > Z_MAX_POS)modified_position = Z_MAX_POS;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], modified_position, current_position[E_AXIS], 600, active_extruder);
				current_position[Z_AXIS]=modified_position;
				//waitPeriod=millis()+50;
				//}
			}
			
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_E)
			{
				Serial.println("EUp");
				modified_position=current_position[E_AXIS]+move_mm;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], modified_position, 500/60, active_extruder);
				current_position[E_AXIS]=modified_position;
			}
			
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_minusE)
			{
				Serial.println("EDown");
				modified_position=current_position[E_AXIS]-move_mm;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], modified_position, 500/60, active_extruder);
				current_position[E_AXIS]=modified_position;
			}
			
			
		}
		#pragma endregion Winbuttons
		
		//USERBUTTONS------------------------------------------------------
		
		if (Event.reportObject.object == GENIE_OBJ_USERBUTTON) //Userbuttons to select GCODE from SD
		{
			if (card.sdprinting){
				
				//******PRINTING****
				#pragma region Printing_screen
				
				if (Event.reportObject.index == BUTTON_PRINT_SETTINGS )
				{
					//Rapduch
					//Edit for final TouchScreen
					Serial.println("PRINTING SETTINGS");
					char buffer[256];
					
					
					is_on_printing_screen=false;
					genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING_SETTINGS_NEW,0);
					
					
					if(print_setting_tool == 0){ //LEFT_EXTRUDER
						int tHotend=target_temperature[0];
						sprintf(buffer, "%3d",tHotend);
						//Serial.println(buffer);
						genie.WriteStr(STRING_PRINT_VALUE,buffer);
						genie.WriteStr(STRING_PRINT_SELECTED,"LEFT EXTRUDER");
						
						}else if(print_setting_tool == 1){ //RIGHT_EXTRUDER
						int tHotend1=target_temperature[1];
						sprintf(buffer, "%3d",tHotend1);
						//Serial.println(buffer);
						genie.WriteStr(STRING_PRINT_VALUE,buffer);
						genie.WriteStr(STRING_PRINT_SELECTED,"RIGTH EXTRUDER");
					}
					
					else if(print_setting_tool == 2){//BED
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED,1);
						int tBed=target_temperature_bed;
						sprintf(buffer, "%3d",tBed);
						//Serial.println(buffer);
						genie.WriteStr(STRING_PRINT_VALUE,buffer);
						genie.WriteStr(STRING_PRINT_SELECTED,"BED");
					}
					
					else if(print_setting_tool == 3){//SPEED
						sprintf(buffer, "%3d %%",feedmultiply);
						//Serial.println(buffer);
						genie.WriteStr(STRING_PRINT_VALUE,buffer);
						genie.WriteStr(STRING_PRINT_SELECTED,"FEED RATE");
					}
					else{
						
					}
					
					/*sprintf(buffer, "%3d",tHotend);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PRINT_SET_NOZZ1,buffer);
					
					sprintf(buffer, "%3d",tHotend1);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PRINT_SET_NOZZ2,buffer);
					
					sprintf(buffer, "%3d",tBed);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PRINT_SET_BED,buffer);
					
					sprintf(buffer, "%3d %%",feedmultiply);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PRINT_SET_PERCENT,buffer);*/
				}
				#pragma endregion Printing_screen
				
				//***** PRINTING_SETTINGS_new
				#pragma region Printing_settings
				
				//choose tool
				else if (Event.reportObject.index == BUTTON_LEFT_EXTRUDER )
				{
					char buffer[256];
					int value=1;
					sprintf(buffer, "%3d",target_temperature[0]);
					genie.WriteStr(STRING_PRINT_VALUE,buffer);
					genie.WriteStr(STRING_PRINT_SELECTED,"LEFT EXTRUDER");
					print_setting_tool = 0;
					
					
				}
				else if (Event.reportObject.index == BUTTON_RIGHT_EXTRUDER )
				{
					char buffer[256];
					int value=1;
					sprintf(buffer, "%3d",target_temperature[1]);
					genie.WriteStr(STRING_PRINT_VALUE,buffer);
					
					genie.WriteStr(STRING_PRINT_SELECTED,"RIGHT EXTRUDER");
					print_setting_tool = 1;
				}
				else if (Event.reportObject.index == BUTTON_BED )
				{
					char buffer[256];
					int value=1;
					sprintf(buffer, "%3d",target_temperature_bed);
					genie.WriteStr(STRING_PRINT_VALUE,buffer);
					genie.WriteStr(STRING_PRINT_SELECTED,"BED TEMPERATURE");
					print_setting_tool = 2;
				}
				else if (Event.reportObject.index == BUTTON_SPPED )
				{
					char buffer[256];
					int value=1;
					sprintf(buffer, "%3d %%",feedmultiply);
					genie.WriteStr(STRING_PRINT_VALUE,buffer);
					genie.WriteStr(STRING_PRINT_SELECTED,"FEED RATE");
					print_setting_tool = 3;
					
				}
				
				//choose values
				else if(Event.reportObject.index == BUTTON_INCREASE){
					char buffer[256];
					int value=1;
					switch (print_setting_tool)
					{
						case 0: //LEFT EXTRUDER
						
						if (target_temperature[0]<HEATER_0_MAXTEMP)
						{
							target_temperature[0]+=value;
							sprintf(buffer, "%3d",target_temperature[0]);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
						
						case 1: //RIGHT EXTRUDER
						
						if (target_temperature[1]<HEATER_1_MAXTEMP)
						{
							target_temperature[1]+=value;
							sprintf(buffer, "%3d",target_temperature[1]);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
						
						case 2: //BED
						
						//if (target_temperature_bed<BED_MAXTEMP)
						if (target_temperature_bed<BED_MAXTEMP)//MaxTemp
						{
							target_temperature_bed+=value;
							sprintf(buffer, "%3d",target_temperature_bed);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
						
						case 3: //SPEED
						
						if (feedmultiply<200)
						{
							feedmultiply+=value;
							sprintf(buffer, "%3d %%",feedmultiply);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
					}
				}
				
				else if(Event.reportObject.index == BUTTON_INCREASE_X3){
					char buffer[256];
					int value=5;
					switch (print_setting_tool)
					{
						case 0: //LEFT EXTRUDER
						if (target_temperature[0]<HEATER_0_MAXTEMP)
						{
							target_temperature[0]+=value;
							sprintf(buffer, "%3d",target_temperature[0]);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
							
						}
						break;
						
						case 1: //RIGHT EXTRUDER
						
						if (target_temperature[1]<HEATER_1_MAXTEMP)
						{
							target_temperature[1]+=value;
							sprintf(buffer, "%3d",target_temperature[1]);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
						
						case 2: //BED
						
						//if (target_temperature_bed<BED_MAXTEMP)
						if (target_temperature_bed<BED_MAXTEMP)//MaxTemp
						{
							target_temperature_bed+=value;
							sprintf(buffer, "%3d",target_temperature_bed);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
						
						case 3: //SPEED
						
						if (feedmultiply<200)
						{
							feedmultiply+=value;
							sprintf(buffer, "%3d %%",feedmultiply);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
					}
				}
				else if(Event.reportObject.index == BUTTON_DECREASE){
					char buffer[256];
					int value=1;
					switch (print_setting_tool)
					{
						case 0: //LEFT EXTRUDER
						if (target_temperature[0]>HEATER_0_MINTEMP)
						{
							target_temperature[0]-=value;
							sprintf(buffer, "%3d",target_temperature[0]);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
						
						case 1: //RIGHT EXTRUDER
						if (target_temperature[1]>HEATER_1_MINTEMP)
						{
							target_temperature[1]-=value;
							sprintf(buffer, "%3d",target_temperature[1]);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
						
						case 2: //BED
						//if (target_temperature_bed<BED_MAXTEMP)
						if (target_temperature_bed>5)//MaxTemp
						{
							target_temperature_bed-=value;
							sprintf(buffer, "%3d",target_temperature_bed);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
						
						case 3: //SPEED
						if (feedmultiply>50)
						{
							feedmultiply-=value;
							sprintf(buffer, "%3d %%",feedmultiply);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
					}
				}
				else if(Event.reportObject.index == BUTTON_DECREASE_X3){
					char buffer[256];
					int value=5;
					switch (print_setting_tool)
					{
						case 0: //LEFT EXTRUDER
						if (target_temperature[0]>HEATER_0_MINTEMP)
						{
							target_temperature[0]-=value;
							sprintf(buffer, "%3d",target_temperature[0]);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
						
						case 1: //RIGHT EXTRUDER
						if(target_temperature[1]>HEATER_1_MINTEMP)
						{
							target_temperature[1]-=value;
							sprintf(buffer, "%3d",target_temperature[1]);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
						
						case 2: //BED
						//if (target_temperature_bed<BED_MAXTEMP)
						if (target_temperature_bed<120)//MaxTemp
						{
							target_temperature_bed-=value;
							sprintf(buffer, "%3d",target_temperature_bed);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
						
						case 3: //SPEED
						if (feedmultiply>50)
						{
							feedmultiply-=value;
							sprintf(buffer, "%3d %%",feedmultiply);
							genie.WriteStr(STRING_PRINT_VALUE,buffer);
						}
						break;
					}
				}
				
				#pragma endregion Printing_settings
				//*****PRINTING_SEGTTINGS_new
				
				else if (Event.reportObject.index == BUTTON_STOP_YES )
				{
					is_on_printing_screen=false;
					card.sdprinting = false;
					card.closefile();
					
					//plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS]+10,current_position[E_AXIS], 600, active_extruder);
					quickStop();
					
					enquecommand_P(PSTR("G28 X0 Y0")); //Home X and Y
					enquecommand_P(PSTR("T0")); //The default states is Left Extruder active
					
					if(SD_FINISHED_STEPPERRELEASE)
					{
						enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
					}
					autotempShutdown();
					//setTargetHotend0(0);
					//setTargetHotend1(0);
					//setTargetHotend2(0);
					//setTargetBed(0);
					card.sdispaused = false;
					cancel_heatup = true;
					
					//sleep_RELAY();
					//Rapduch
					genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				}
				
				else if (Event.reportObject.index == BUTTON_PAUSE_RESUME )
				{
					int value = genie.GetEventData(&Event);
					if (value == 1) // Need to pause
					{
						////I believe it is a really unsafe way to do it
						////plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+20, current_position[E_AXIS], homing_feedrate[Z_AXIS]/60, RIGHT_EXTRUDER);
						////st_synchronize();
						card.pauseSDPrint();
						Serial.println("PAUSE!");
						flag_pause = true;
					}
					
				}
				
				//*****Printing Settings*****
				#pragma region Printing Settings
				else if (Event.reportObject.index == BUTTON_PRINT_SET_SPEED_UP )
				{
					char buffer[256];
					int value=5;
					if (feedmultiply<200)
					{
						feedmultiply+=value;
						sprintf(buffer, "%3d %%",feedmultiply);
						genie.WriteStr(STRING_PRINT_SET_PERCENT,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_SPEED_DOWN )
				{
					char buffer[256];
					int value=5;
					if (feedmultiply>50)
					{
						feedmultiply-=value;
						sprintf(buffer, "%3d %%",feedmultiply);
						genie.WriteStr(STRING_PRINT_SET_PERCENT,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ1_UP )
				{
					char buffer[256];
					int value=5;
					if (target_temperature[0]<HEATER_0_MAXTEMP)
					{
						target_temperature[0]+=value;
						sprintf(buffer, "%3d",target_temperature[0]);
						genie.WriteStr(STRING_PRINT_SET_NOZZ1,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ1_DOWN )
				{
					char buffer[256];
					int value=5;
					if (target_temperature[0]>HEATER_0_MINTEMP)
					{
						target_temperature[0]-=value;
						sprintf(buffer, "%3d",target_temperature[0]);
						genie.WriteStr(STRING_PRINT_SET_NOZZ1,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ2_UP )
				{
					char buffer[256];
					int value=5;
					if (target_temperature[1]<HEATER_1_MAXTEMP)
					{
						target_temperature[1]+=value;
						sprintf(buffer, "%3d",target_temperature[1]);
						genie.WriteStr(STRING_PRINT_SET_NOZZ2,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ2_DOWN )
				{
					char buffer[256];
					int value=5;
					if (target_temperature[1]>HEATER_1_MINTEMP)
					{
						target_temperature[1]-=value;
						sprintf(buffer, "%3d",target_temperature[1]);
						genie.WriteStr(STRING_PRINT_SET_NOZZ2,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_BED_UP )
				{
					char buffer[256];
					int value=5;
					//if (target_temperature_bed<BED_MAXTEMP)
					if (target_temperature_bed<120)//MaxTemp
					{
						target_temperature_bed+=value;
						sprintf(buffer, "%3d",target_temperature_bed);
						genie.WriteStr(STRING_PRINT_SET_BED,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_BED_DOWN )
				{
					char buffer[256];
					int value=5;
					//if (target_temperature_bed>BED_MINTEMP)
					if (target_temperature_bed>5)//Mintemp
					{
						target_temperature_bed-=value;
						sprintf(buffer, "%3d",target_temperature_bed);
						genie.WriteStr(STRING_PRINT_SET_BED,buffer);
					}
				}
				
				#pragma endregion Printing Settings
				
				
				}else{//All that has to be done out of the printing room
				
				//We need to Resume/Enter Printing Settings/Stop printing
				if (Event.reportObject.index == BUTTON_PAUSE_RESUME )
				{
					//I believe it is a really unsafe way to do it
					//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]-20, current_position[E_AXIS], homing_feedrate[Z_AXIS]/60, RIGHT_EXTRUDER);
					//st_synchronize();
					card.startFileprint();
					Serial.println("RESUME!");
					flag_resume = true;
					if(flag_resume){
						enquecommand_P(((PSTR("G70"))));
						flag_resume = false;
						Serial.println("resume detected");
					}
				}
				
				else if (Event.reportObject.index == BUTTON_CHANGE_EXTRUDER)
				{
					int value = genie.GetEventData(&Event);
					if (value == 1)
					{ //Second extruder
						enquecommand_P(((PSTR("T1"))));
						which_extruder=1;
						}else{
						enquecommand_P(((PSTR("T0"))));
						which_extruder=0;
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SETTINGS )
				{
					//Rapduch
					//Edit for final TouchScreen
					Serial.println("PRINTING SETTINGS");
					char buffer[256];
					int tHotend=target_temperature[0];
					int tHotend1=target_temperature[1];
					int tBed=target_temperature_bed;
					
					is_on_printing_screen=false;
					genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING_SETTINGS,0);
					
					sprintf(buffer, "%3d",tHotend);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PRINT_SET_NOZZ1,buffer);
					
					sprintf(buffer, "%3d",tHotend1);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PRINT_SET_NOZZ2,buffer);
					
					sprintf(buffer, "%3d",tBed);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PRINT_SET_BED,buffer);
					
					sprintf(buffer, "%3d %%",feedmultiply);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PRINT_SET_PERCENT,buffer);
				}
				
				//*****Printing Settings*****
				#pragma region Printing Settings
				else if (Event.reportObject.index == BUTTON_PRINT_SET_SPEED_UP )
				{
					char buffer[256];
					int value=5;
					if (feedmultiply<200)
					{
						feedmultiply+=value;
						sprintf(buffer, "%3d %%",feedmultiply);
						genie.WriteStr(STRING_PRINT_SET_PERCENT,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_SPEED_DOWN )
				{
					char buffer[256];
					int value=5;
					if (feedmultiply>50)
					{
						feedmultiply-=value;
						sprintf(buffer, "%3d %%",feedmultiply);
						genie.WriteStr(STRING_PRINT_SET_PERCENT,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ1_UP )
				{
					char buffer[256];
					int value=5;
					if (target_temperature[0]<HEATER_0_MAXTEMP)
					{
						target_temperature[0]+=value;
						sprintf(buffer, "%3d",target_temperature[0]);
						genie.WriteStr(STRING_PRINT_SET_NOZZ1,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ1_DOWN )
				{
					char buffer[256];
					int value=5;
					if (target_temperature[0]>HEATER_0_MINTEMP)
					{
						target_temperature[0]-=value;
						sprintf(buffer, "%3d",target_temperature[0]);
						genie.WriteStr(STRING_PRINT_SET_NOZZ1,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ2_UP )
				{
					char buffer[256];
					int value=5;
					if (target_temperature[1]<HEATER_1_MAXTEMP)
					{
						target_temperature[1]+=value;
						sprintf(buffer, "%3d",target_temperature[1]);
						genie.WriteStr(STRING_PRINT_SET_NOZZ2,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ2_DOWN )
				{
					char buffer[256];
					int value=5;
					if (target_temperature[1]>HEATER_1_MINTEMP)
					{
						target_temperature[1]-=value;
						sprintf(buffer, "%3d",target_temperature[1]);
						genie.WriteStr(STRING_PRINT_SET_NOZZ2,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_BED_UP )
				{
					char buffer[256];
					int value=5;
					//if (target_temperature_bed<BED_MAXTEMP)
					if (target_temperature_bed<120)//MaxTemp
					{
						target_temperature_bed+=value;
						sprintf(buffer, "%3d",target_temperature_bed);
						genie.WriteStr(STRING_PRINT_SET_BED,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_BED_DOWN )
				{
					char buffer[256];
					int value=5;
					//if (target_temperature_bed>BED_MINTEMP)
					if (target_temperature_bed>5)//Mintemp
					{
						target_temperature_bed-=value;
						sprintf(buffer, "%3d",target_temperature_bed);
						genie.WriteStr(STRING_PRINT_SET_BED,buffer);
					}
				}
				#pragma endregion Printing Settings
				
				
				else if (Event.reportObject.index == BUTTON_STOP_YES )
				{
					is_on_printing_screen=false;
					card.sdprinting = false;
					card.closefile();
					
					//plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS]+10,current_position[E_AXIS], 600, active_extruder);
					quickStop();
					
					enquecommand_P(PSTR("G28 X0 Y0")); //Home X and Y
					enquecommand_P(PSTR("T0")); //The default states is Left Extruder active
					
					if(SD_FINISHED_STEPPERRELEASE)
					{
						enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
					}
					autotempShutdown();
					//setTargetHotend0(0);
					//setTargetHotend1(0);
					//setTargetHotend2(0);
					//setTargetBed(0);
					card.sdispaused = false;
					cancel_heatup = true;
					
					//sleep_RELAY();
					//Rapduch
					genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				}
				
				
				
				//*****SD Gcode Selection*****
				#pragma region SD Gcode Selector
				else if (Event.reportObject.index == BUTTON_SD_SELECTED ||  Event.reportObject.index == STRING_NAME_FILE)
				{
					if(card.cardOK)
					{
						if (!card.filenameIsDir){ //If the filename is a gcode we start printing
							char cmd[30];
							char* c;
							card.getfilename(filepointer);
							sprintf_P(cmd, PSTR("M23 %s"), card.filename);
							for(c = &cmd[4]; *c; c++)
							{
								*c = tolower(*c);
							}
							enquecommand(cmd);
							
							is_on_printing_screen=true;//We are entering printing screen
							enquecommand_P(PSTR("M24")); // It also sends you to PRINTING screen
							
							screen_status="Ready...";//Write the selected SD file to all strings
						}
					}
				}
				
				
				else if (Event.reportObject.index == BUTTON_SD_LEFT || Event.reportObject.index == BUTTON_SD_RIGHT) //TODO: control if SD is out
				{
					if (Event.reportObject.index == BUTTON_SD_LEFT) //LEFT button pressed
					{
						if (filepointer == 0)
						{
							filepointer=card.getnrfilenames()-1; //Last SD file
							}else{
							filepointer--;
						}
					}
					else if (Event.reportObject.index == BUTTON_SD_RIGHT) //RIGHT button pressed
					{
						if (filepointer == card.getnrfilenames()-1)
						{
							filepointer=0; //First SD file
							}else{
							filepointer++;
						}
					}
					
					card.getfilename(filepointer);
					if (card.filenameIsDir)
					{
						//Is a folder
						//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,1);
						}else{
						//Is a file
						//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
					}
					Serial.println(card.longFilename);
					int count = 18;
					char buffer[count];
					memset( buffer, '\0', sizeof(char)*count );
					
					if (String(card.longFilename).length()-6 > count){
						for (int i = 0; i<count-3 ; i++)
						{
							if (card.longFilename[i] == '.') i = count +10;
							else buffer[i]=card.longFilename[i];
						}
						buffer[count]='\0';
						char* buffer2 = strcat(buffer,"...\0");
						genie.WriteStr(STRING_NAME_FILE,buffer2);//Printing form
					}
					else {
						for (int i = 0; i<String(card.longFilename).length() ; i++)	{
							if (card.longFilename[i] == '.') i = count +10;
							else buffer[i]=card.longFilename[i];
						}
						buffer[count]='\0';
						genie.WriteStr(STRING_NAME_FILE,buffer);//Printing form
						//Is a file
						//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
						
					}
					//***************CIRCLE ARRAY***************
					/*char show[count];
					for (int j = 0; j <= String(card.longFilename).length() + 1; j++){
					for(int i = 0; i < count; i++){
					if (i+j == String(card.longFilename).length()) show[i] = ' ';
					else show[i] = buffer[(i+j)%(String(card.longFilename).length() + 1)];
					Serial.print(show[i]);
					delay(1000);
					}
					Serial.println("");
					}*/
					//////////////////////////////////////////////
					//Keep in mind to control the length of the string displayed!
					//genie.WriteStr(2,card.longFilename);
					Serial.print("Image n: ");
					Serial.println(filepointer);
				}
				#pragma endregion SD Gcode Selector

				
				/*//else if (Event.reportObject.index == BUTTON_SPEED_UP )
				//{
				//int value=5;
				//if (feedmultiply<200)
				//{
				//feedmultiply+=value;
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FEEDRATE,feedmultiply);
				//}
				//}
				//
				//else if (Event.reportObject.index == BUTTON_SPEED_DOWN )
				//{
				//int value=5;
				//if (feedmultiply>50)
				//{
				//feedmultiply-=value;
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FEEDRATE,feedmultiply);
				//}
				//}
				//
				//
				//
				//else if (Event.reportObject.index == BUTTON_BED_UP )
				//{
				//int value=5;
				////if (target_temperature_bed<BED_MAXTEMP)
				//if (target_temperature_bed<120)//MaxTemp
				//{
				//target_temperature_bed+=value;
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_BED,target_temperature_bed);
				//}
				//}
				//
				//else if (Event.reportObject.index == BUTTON_BED_DOWN )
				//{
				//int value=5;
				////if (target_temperature_bed>BED_MINTEMP)
				//if (target_temperature_bed>5)//Mintemp
				//{
				//target_temperature_bed-=value;
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_BED,target_temperature_bed);
				//}
				//}
				//
				//else if (Event.reportObject.index == BUTTON_FAN_UP )
				//{
				//int value=5;
				//if (fanSpeed<255)
				//{
				//fanSpeed+=value;
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FAN,fanSpeed);
				//}
				//}
				//
				//else if (Event.reportObject.index == BUTTON_FAN_DOWN )
				//{
				//int value=5;
				//if (fanSpeed>0)
				//{
				//fanSpeed-=value;
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FAN,fanSpeed);
				//}
				//}
				
				//else if (Event.reportObject.index == BUTTON_SETUP_BACK )
				//{
				////Get status string
				//char buffer[15];
				//screen_status.toCharArray(buffer,15,0);
				//
				//if (card.sdprinting || card.sdispaused)
				//{
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING,0);
				////Restore Strings after form update
				//genie.WriteStr(2,card.longFilename);
				////genie.WriteStr(6,buffer);
				//}else
				//{
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_START_PRINT,0);
				////Restore Strings
				//genie.WriteStr(7,card.longFilename);
				//genie.WriteStr(8,buffer);
				//}
				//}
				
				
				//else if (Event.reportObject.index == BUTTON_SETUP_BACK_NOZZLE || Event.reportObject.index == BUTTON_SETUP_BACK_BED )
				//{
				//if (surfing_utilities) // Check if we are backing from utilities or print setup
				//{
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_TEMPERATURE,0);
				//}else
				//{
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_SETUP,0);
				//}
				//}
				
				
				
				
				
				//NOZZLE BUTTONS reusability---------------------------------------------------
				//else if (Event.reportObject.index == BUTTON_NOZZLE1_PRINT || Event.reportObject.index == BUTTON_NOZZLE2_PRINT || Event.reportObject.index == BUTTON_NOZZLE1_TEMP || Event.reportObject.index == BUTTON_NOZZLE2_TEMP)
				//{
				//if (Event.reportObject.index == BUTTON_NOZZLE1_PRINT || Event.reportObject.index == BUTTON_NOZZLE1_TEMP)
				//{
				//which_extruder=0;
				//}else{
				//which_extruder=1;
				//}
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[which_extruder]);
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_NOZZLE,0);
				//}
				
				//else if (Event.reportObject.index == BUTTON_NOZZLE_UP )
				//{
				//int value=5;
				//if (target_temperature[which_extruder]<HEATER_0_MAXTEMP)
				//{
				//target_temperature[which_extruder]+=value;
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[which_extruder]);
				//}
				//}
				//
				//else if (Event.reportObject.index == BUTTON_NOZZLE_DOWN )
				//{
				//int value=5;
				//if (target_temperature[which_extruder]>HEATER_0_MINTEMP)
				//{
				//target_temperature[which_extruder]-=value;
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[which_extruder]);
				//}
				//}
				
				
				
				//NOZZLEBUTTONS-------*/
				else if (Event.reportObject.index == 148  )
				{
					/*int i = (lang%8)+1;					
					lang = i;
					enquecommand_P(PSTR("M500"));
					Serial.println(5,LANGUAGE);*/
				}
				
				//*****INSERT/REMOVE FILAMENT*****
				#pragma region Insert_Remove_Fil
				
				else if (Event.reportObject.index == BUTTON_FILAMENT_BACK  )
				{
					flag_filament_home=false;
					genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES,0);
				}
				
				else if (Event.reportObject.index == BUTTON_INSERT_FIL || Event.reportObject.index == BUTTON_REMOVE_FIL || Event.reportObject.index == BUTTON_PURGE_FIL  )
				{
					if (Event.reportObject.index == BUTTON_INSERT_FIL) filament_mode = 'I'; //Insert Mode
					else if (Event.reportObject.index == BUTTON_REMOVE_FIL) filament_mode = 'R'; //Remove Mode
					
					/*if (!flag_filament_home){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);		
						
					}*/
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_SELECT_EXTRUDER,0);
				}
				

				else if (Event.reportObject.index == BUTTON_FILAMENT_NOZZLE1 || Event.reportObject.index == BUTTON_FILAMENT_NOZZLE2)
				{
					if (Event.reportObject.index == BUTTON_FILAMENT_NOZZLE1) //Left Nozzle
					{
						which_extruder=0;
					}
					else //Right Nozzle
					{
						which_extruder=1;
					}
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
					
					
					//*********Move the bed down and the extruders inside
					if (!home_made) home_axis_from_code();
					
					int feedrate;
					if (!flag_filament_home){
						//MOVING THE EXTRUDERS TO AVOID HITTING THE CASE WHEN PROBING-------------------------
						current_position[X_AXIS]+=25;
						feedrate=homing_feedrate[X_AXIS];
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, LEFT_EXTRUDER);
					
						st_synchronize();
						//current_position[X_AXIS] = x_home_pos(RIGHT_EXTRUDER);
					
						current_position[X_AXIS]=extruder_offset[X_AXIS][1];
						Serial.println(current_position[X_AXIS]);
						plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
						current_position[X_AXIS]-=25;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
						st_synchronize();
						flag_filament_home=true;
					}
					
					current_position[Y_AXIS]=10;
					feedrate=homing_feedrate[Y_AXIS];
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					st_synchronize();
					
					current_position[Z_AXIS]=Z_MAX_POS-5;
					feedrate=homing_feedrate[Z_AXIS];
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate*4/60, active_extruder); //check speed
					
					
					/****************************************************/
					
					//ATTENTION : Order here is important
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INSERT_FIL_PREHEAT,0);
					if (filament_mode=='I'){
						genie.WriteStr(STRING_ADVISE_FILAMENT,"");
						//genie.WriteStr(STRING_ADVISE_FILAMENT,"Insert the filament until you feel it stops, \n then while you keep inserting around \n 10 mm of filament, press the clip");
						genie.WriteObject(GENIE_OBJ_USERIMAGES,10,0);
						setTargetHotend(INSERT_FIL_TEMP,which_extruder);
						
					}
					else if (filament_mode=='R'){		
						Serial.println("REMOVING");		
						genie.WriteStr(STRING_ADVISE_FILAMENT,"");
						genie.WriteObject(GENIE_OBJ_USERIMAGES,10,1);
						setTargetHotend(REMOVE_FIL_TEMP,which_extruder);
				}
				
				is_changing_filament=true; //We are changing filament					
				}


				else if (Event.reportObject.index == BUTTON_INSERT_BACK)
				{
					is_changing_filament=false; //We are no longer waiting for heat
					setTargetHotend(0,which_extruder); //ATTENTION : Order here is important
					genie.WriteObject(GENIE_OBJ_FORM,FORM_SELECT_EXTRUDER,0);
				}
				
				
				else if (Event.reportObject.index == BUTTON_INSERT )
				{// We should have already checked if filament is inserted
					if (filament_mode =='I')
					{ //Inserting...
						Serial.print("Inserting :   ");
						current_position[E_AXIS] += 20;//Extra extrusion at low feedrate
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_SLOW_SPEED/60, which_extruder);
						current_position[E_AXIS] += (BOWDEN_LENGTH-300);
						Serial.println(current_position[E_AXIS]);
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, which_extruder);
						current_position[E_AXIS] += EXTRUDER_LENGTH;//Extra extrusion at low feedrate
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_SLOW_SPEED/60, which_extruder);
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						st_synchronize();
						genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUST_FILAMENT,0);
						
					}else if (filament_mode =='R')
					{ //Removing...
						current_position[E_AXIS] = current_position[E_AXIS]-(BOWDEN_LENGTH + EXTRUDER_LENGTH + 100);//Extra extrusion at low feedrate
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_FAST_SPEED/60, which_extruder);
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						st_synchronize();
						genie.WriteObject(GENIE_OBJ_FORM,FORM_SUCCESS_FILAMENT,0);
					}
					
					//We prefer to maintain temp after changing filament
					//setTargetHotend(0,which_extruder);
					//put_info_text("Filament DONE");
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES,0);
				}
				#pragma endregion Insert_Remove_Fil
				
				
				//*****AdjustFilament******
				#pragma region AdjustFilament
				else if (Event.reportObject.index == BUTTON_ACCEPT_ADJUST)
				{
					genie.WriteObject(GENIE_OBJ_FORM,FORM_SUCCESS_FILAMENT,0);
				}
				
				else if (Event.reportObject.index == BUTTON_ADJUST_ZUp)
				{
					//Adjusting the filament with a retrack Up
					Serial.println("Adjust ZUp");
					float modified_position=current_position[E_AXIS]-5;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], modified_position, 500/60, which_extruder);
					current_position[E_AXIS]=modified_position;
				}
				
				else if (Event.reportObject.index == BUTTON_ADJUST_ZDown)
				{
					//Adjusting the filament with a purge Down
					Serial.println("Adjust ZDown");
					float modified_position=current_position[E_AXIS]+5;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], modified_position, 500/60, which_extruder);
					current_position[E_AXIS]=modified_position;
				}
				#pragma endregion AdjustFilament
				
				
				
				//Extruder Calibrations-------------------------------------------------
				/*else if (Event.reportObject.index == BUTTON_CAL_EXTRUDERS_X)
				{
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G40"));
				
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
				}
				
				
				//else if (Event.reportObject.index == BUTTON_CAL_EXTRUDERS_Y)*/
				else if (Event.reportObject.index == BUTTON_CAL_FULL)
				{
				flag_full_calib = true;
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("T0"));
				enquecommand_P(PSTR("G34"));	//Start BED Calibration Wizard
				
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"BED");
				st_synchronize();
				
				/*enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));		//CALIB Y
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);*/
				}
				
				/*else if (Event.reportObject.index == BUTTON_CAL_EXTRUDERS_Z)
				{
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G43"));
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
				}*/
				
				else if (Event.reportObject.index == BUTTON_CLEAN_DONE){
				if (flag_continue_calib){
				genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Clean the right nozzle \nand press GO, \nthen the Z calibration will start\n");
				if (active_extruder == 0)	{
				changeTool(1);
				current_position[X_AXIS] = 155;
				current_position[Y_AXIS] = 0;
				current_position[Z_AXIS] = 100;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], 0, 50, active_extruder);//move first extruder, bed and Y
				} else {
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"        Z AXIS");
				if (active_extruder == 1) changeTool(0);
				home_axis_from_code();
				st_synchronize();
				enquecommand_P(PSTR("G43"));
				flag_continue_calib = false;
				genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
				}
				
				}
				
				}
				
				else if(Event.reportObject.index == BUTTON_CLEAN_CHANGE){
				/*if (flag_continue_calib){
				if (active_extruder == 0)	changeTool(1);
				else changeTool(0);
				//MOVE EXTRUDERS
				current_position[X_AXIS] = 155;
				current_position[Y_AXIS] = 0;
				current_position[Z_AXIS] = 100;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], 0, 50, active_extruder);//move first extruder, bed and Y
				}*/
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_BACK )
				{
					is_on_printing_screen=true;
					int count = 12;
					char buffer[count];
					if (String(card.longFilename).length()>count){
						for (int i = 0; i<count ; i++)
						{
							/*if (buffer[i] = '.') i = count +10;
							else */buffer[i]=card.longFilename[i];
						}
						buffer[count]='\0';
						char* buffer2 = strcat(buffer,"...\0");
						genie.WriteStr(STRINGS_PRINTING_GCODE,buffer2);//Printing form
					}else{
						/*char* str;
						str = strtok (buffer,".");*/
						for (int i = 0; i<=String(card.longFilename).length(); i++)
						{
							/*if (buffer[i] = '.') i = String(card.longFilename).length() +10;
							else */buffer[i]=card.longFilename[i];
						}
						buffer[count]='\0';
						genie.WriteStr(STRINGS_PRINTING_GCODE,buffer);//Printing form
					}
					genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING,1);
				}
				
				//*****Preheat Settings*****
				#pragma region Preheat Settings
				//Buttons for preheat settings
				
				else if (Event.reportObject.index == BUTTON_COOLDOWN )
				{
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);
				}
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_NOZZ1_UP )
				{
				char buffer[256];
				int value=5;
				if (target_temperature[0]<HEATER_0_MAXTEMP)
				{
				target_temperature[0]+=value;
				sprintf(buffer, "%3d",target_temperature[0]);
				genie.WriteStr(STRING_PREHEAT_SET_NOZZ1,buffer);
				}
				}
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_NOZZ1_DOWN )
				{
				char buffer[256];
				int value=5;
				if (target_temperature[0]>HEATER_0_MINTEMP)
				{
				target_temperature[0]-=value;
				sprintf(buffer, "%3d",target_temperature[0]);
				genie.WriteStr(STRING_PREHEAT_SET_NOZZ1,buffer);
				}
				}
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_NOZZ2_UP )
				{
				char buffer[256];
				int value=5;
				if (target_temperature[1]<HEATER_1_MAXTEMP)
				{
				target_temperature[1]+=value;
				sprintf(buffer, "%3d",target_temperature[1]);
				genie.WriteStr(STRING_PREHEAT_SET_NOZZ2,buffer);
				}
				}
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_NOZZ2_DOWN )
				{
				char buffer[256];
				int value=5;
				if (target_temperature[1]>HEATER_1_MINTEMP)
				{
				target_temperature[1]-=value;
				sprintf(buffer, "%3d",target_temperature[1]);
				genie.WriteStr(STRING_PREHEAT_SET_NOZZ2,buffer);
				}
				}
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_BED_UP )
				{
				char buffer[256];
				int value=5;
				//if (target_temperature_bed<BED_MAXTEMP)
				if (target_temperature_bed<120)//MaxTemp
				{
				target_temperature_bed+=value;
				sprintf(buffer, "%3d",target_temperature_bed);
				genie.WriteStr(STRING_PREHEAT_SET_BED,buffer);
				}
				}
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_BED_DOWN )
				{
				char buffer[256];
				int value=5;
				//if (target_temperature_bed>BED_MINTEMP)
				if (target_temperature_bed>5)//Mintemp
				{
				target_temperature_bed-=value;
				sprintf(buffer, "%3d",target_temperature_bed);
				genie.WriteStr(STRING_PREHEAT_SET_BED,buffer);
				}
				}
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_BACK )
				{
				//Cooldown
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);
				genie.WriteObject(GENIE_OBJ_FORM,FORM_TEMP_MENU,0);
				}
				#pragma endregion Preheat Settings
				
				
				//*****Bed Calibration*****
				#pragma region Bed Calibration
				else if (Event.reportObject.index == BUTTON_Z_CAL_WIZARD)
				{
				flag_full_calib = false;
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("T0"));
				enquecommand_P(PSTR("G34"));	//Start BED Calibration Wizard
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
				}
				
				else if (Event.reportObject.index == BUTTON_REDO_BED_CALIB )
				{
				enquecommand_P((PSTR("G28")));
				enquecommand_P((PSTR("T0")));
				enquecommand_P((PSTR("G34")));
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
				}
				
				else if (Event.reportObject.index == BUTTON_BED_CALIB_SW2)
				{
				char buffer[256];
				if (vuitens2!=0){
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW2,0);
				sprintf(buffer, " %d / 8",vuitens2); //Printing how to calibrate on screen
				//genie.WriteStr(STRING_BED_SCREW2,buffer);
				if (vuitens3==0) genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW3,2);
				else{genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW3,0);}
				if (sentit2>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,vuitens2);} //The direction is inverted in Sigma's bed screws
				else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,vuitens2+8);}
				
				}else if (vuitens3!=0)
				{
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW3,0);
				sprintf(buffer, " %d / 8",vuitens3); //Printing how to calibrate on screen
				//genie.WriteStr(STRING_BED_SCREW3,buffer);
				if (sentit3>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,vuitens3);} //The direction is inverted in Sigma's bed screws
				else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,vuitens3+8);}
				}else{
				enquecommand_P((PSTR("G28")));
				enquecommand_P((PSTR("T0")));
				enquecommand_P((PSTR("G34")));
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
				}
				}
				
				else if (Event.reportObject.index == BUTTON_BED_CALIB_SW3)
				{
				char buffer[256];
				if (vuitens3!=0){
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW3,0);
				sprintf(buffer, " %d / 8",vuitens3); //Printing how to calibrate on screen
				//genie.WriteStr(STRING_BED_SCREW3,buffer);
				if (sentit3>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,vuitens3);} //The direction is inverted in Sigma's bed screws
				else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,vuitens3+8);}
				}else{
				enquecommand_P((PSTR("G28")));
				enquecommand_P((PSTR("T0")));
				enquecommand_P((PSTR("G34")));
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
				}
				}
				#pragma endregion Bed Calibration
				
				
				//*****Success Screens*****
				#pragma region SuccessScreens
				else if (Event.reportObject.index == BUTTON_BED_CALIB_SUCCESS )
				{
				//enquecommand_P((PSTR("G28 X0 Y0")));
				enquecommand_P((PSTR("T0")));
				Serial.println("Calibration Successful, going back to main menu");
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIBRATION,0);
				
				}
				
				else if (Event.reportObject.index == BUTTON_SUCCESS_FILAMENT_OK)
				{
				//enquecommand_P((PSTR("G28 X0 Y0")));
				enquecommand_P((PSTR("T0")));
				Serial.println("Filament Inserted/Removed, returning to Main Menu");
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FILAMENT,0);
				}
				#pragma endregion SuccessScreens
				
				//***** Calibration XYZ *****
				#pragma region CalibrationsXYZ
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT1)
				{
				//char buffer[30];
				float calculus = extruder_offset[X_AXIS][1] + 0.5;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT2)
				{
				float calculus = extruder_offset[X_AXIS][1] + 0.4;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT3)
				{
				float calculus = extruder_offset[X_AXIS][1] + 0.3;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT4)
				{
				float calculus = extruder_offset[X_AXIS][1] + 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT5)
				{
				float calculus = extruder_offset[X_AXIS][1] + 0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT6)
				{
				float calculus = extruder_offset[X_AXIS][1];
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT7)
				{
				float calculus = extruder_offset[X_AXIS][1] - 0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT8)
				{
				float calculus = extruder_offset[X_AXIS][1] - 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT9)
				{
				float calculus = extruder_offset[X_AXIS][1] - 0.3;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT10)
				{
				float calculus = extruder_offset[X_AXIS][1]-0.4;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT1)
				{
				//char buffer[30];
				float calculus = extruder_offset[Y_AXIS][1] - 0.5;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT2)
				{
				//enquecommand_P((PSTR("M218 T1 X-0.4")));
				float calculus = extruder_offset[Y_AXIS][1] - 0.4;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT3)
				{
				float calculus = extruder_offset[Y_AXIS][1] - 0.3;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//enquecommand_P((PSTR("M218 T1 X-0.3")));
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT4)
				{
				float calculus = extruder_offset[Y_AXIS][1] - 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//enquecommand_P((PSTR("M218 T1 X-0.2")));
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT5)
				{
				float calculus = extruder_offset[Y_AXIS][1] -0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.1")));
				//enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT6)
				{
				float calculus = extruder_offset[Y_AXIS][1];
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT7)
				{
				float calculus = extruder_offset[X_AXIS][1] + 0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT8)
				{
				float calculus = extruder_offset[Y_AXIS][1] + 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT9)
				{
				float calculus = extruder_offset[Y_AXIS][1] + 0.3;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//enquecommand_P((PSTR("M218 T1 X-0.2")));
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT10)
				{
				float calculus = extruder_offset[Y_AXIS][1] + 0.4;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_Z_CALIB_Z1_Up)
				{
				float feedrate = homing_feedrate[Z_AXIS];
				current_position[Z_AXIS] += 0.1;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				Serial.print("Z position: ");
				Serial.println(current_position[Z_AXIS]);
				}
				
				else if (Event.reportObject.index == BUTTON_Z_CALIB_Z1_Down)
				{
				float feedrate = homing_feedrate[Z_AXIS];
				if (current_position[Z_AXIS]>-1) current_position[Z_AXIS] -= 0.1; //Max down is Z=-0.5
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				Serial.print("Z position: ");
				Serial.println(current_position[Z_AXIS]);
				}
				
				else if (Event.reportObject.index == BUTTON_Z_CALIB_Z1_OK)
				{
				Serial.println("OK first Extruder!");
				//We have to override z_prove_offset
				zprobe_zoffset-=current_position[Z_AXIS]; //We are putting more offset if needed
				extruder_offset[Z_AXIS][LEFT_EXTRUDER]=0.0;//It is always the reference
				current_position[Z_AXIS]=0;//We are setting this position as Zero
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
				Serial.print("Z1 Probe offset: ");
				Serial.println(zprobe_zoffset);
				enquecommand_P(PSTR("M500"));//Store everything
				enquecommand_P(PSTR("T1"));
				enquecommand_P(PSTR("G43"));
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_Z_EXTRUDER2,0);
				}
				
				else if (Event.reportObject.index == BUTTON_Z_CALIB_Z2_Up)
				{
				float feedrate = homing_feedrate[Z_AXIS];
				current_position[Z_AXIS] += 0.1;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				Serial.print("Z position: ");
				Serial.println(current_position[Z_AXIS]);
				}
				
				else if (Event.reportObject.index == BUTTON_Z_CALIB_Z2_Down)
				{
				float feedrate = homing_feedrate[Z_AXIS];
				if (current_position[Z_AXIS]>-1) current_position[Z_AXIS] -= 0.1;  //Max down is Z=-0.5
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				Serial.print("Z position: ");
				Serial.println(current_position[Z_AXIS]);
				}
				
				else if (Event.reportObject.index == BUTTON_Z_CALIB_Z2_OK)
				{
				Serial.println("OK second Extruder!");
				extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=current_position[Z_AXIS];//Add the difference to the current offset value
				Serial.print("Z2 Offset: ");
				Serial.println(extruder_offset[Z_AXIS][RIGHT_EXTRUDER]);
				enquecommand_P(PSTR("M500"));//Store everything
				enquecommand_P(PSTR("T0"));
				enquecommand_P(PSTR("G28"));
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIBRATION,0);
				
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G40"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"X AXIS, Heating...");
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				#ifdef SIGMA_TOUCH_SCREEN
				#endif
				}
				
				#pragma endregion CalibrationsXYZ
				
				//***** Calibration X new ***
				#pragma region Calibration X
				else if (Event.reportObject.index == BUTTON_X_1){
				//char buffer[30];
				
				genie.WriteStr(STRING_X_CAB_VALUE,"1");
				offset_x_calib = 0.5;
				/*float calculus = extruder_offset[X_AXIS][1] + 0.4;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXEL, heatting");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);*/
				}
				else if (Event.reportObject.index == BUTTON_X_2){
				//char buffer[30];
				genie.WriteStr(STRING_X_CAB_VALUE,"2");
				offset_x_calib = 0.4;
				/*float calculus = extruder_offset[X_AXIS][1] + 0.3;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXEL, heatting");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);*/
				}
				else if (Event.reportObject.index == BUTTON_X_3){
				//char buffer[30];
				genie.WriteStr(STRING_X_CAB_VALUE,"3");
				offset_x_calib = 0.3;
				/*float calculus = extruder_offset[X_AXIS][1] + 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXEL, heatting");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);*/
				}
				else if (Event.reportObject.index == BUTTON_X_4){
				//char buffer[30];
				offset_x_calib = 0.2;
				genie.WriteStr(STRING_X_CAB_VALUE,"4");
				/*float calculus = extruder_offset[X_AXIS][1] + 0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXEL, heatting");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);*/
				}
				else if (Event.reportObject.index == BUTTON_X_5){
				//char buffer[30];
				genie.WriteStr(STRING_X_CAB_VALUE,"5");
				offset_x_calib = 0.1;
				/*float calculus = extruder_offset[X_AXIS][1];
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXEL, heatting");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);*/
				}
				else if (Event.reportObject.index == BUTTON_X_6){
				//char buffer[30];
				genie.WriteStr(STRING_X_CAB_VALUE,"6");
				offset_x_calib = 0;
				/*float calculus = extruder_offset[X_AXIS][1] - 0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXEL, heatting");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);*/
				}
				else if (Event.reportObject.index == BUTTON_X_7){
				//char buffer[30];
				genie.WriteStr(STRING_X_CAB_VALUE,"7");
				offset_x_calib = -0.1;
				/*float calculus = extruder_offset[X_AXIS][1] - 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXEL, heatting");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);*/
				}
				else if (Event.reportObject.index == BUTTON_X_8){
				//char buffer[30];
				genie.WriteStr(STRING_X_CAB_VALUE,"8");
				offset_x_calib = -0.2;
				/*float calculus = extruder_offset[X_AXIS][1] - 0.3;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXEL, heatting");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);*/
				}
				else if (Event.reportObject.index == BUTTON_X_9){
				//char buffer[30];
				genie.WriteStr(STRING_X_CAB_VALUE,"9");
				offset_x_calib = -0.3;
				/*float calculus = extruder_offset[X_AXIS][1] - 0.4;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXEL, heatting");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);*/
				}
				else if (Event.reportObject.index == BUTTON_X_10){
				//char buffer[30];
				genie.WriteStr(STRING_X_CAB_VALUE,"10");
				offset_x_calib = -0.4;
				/*float calculus = extruder_offset[X_AXIS][1] - 0.5;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXEL, heatting");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);*/
				}
				else if (Event.reportObject.index == BUTTON_X_ACCEPT){
				
				float calculus = extruder_offset[X_AXIS][1] + offset_x_calib;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXEL, heatting");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				
				}
				
				/*else if (Event.reportObject.index == BUTTON_X_REDO){
				
				float calculus = extruder_offset[X_AXIS][1] + offset_x_calib;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G40"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"X AXEL, heatting...");
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}*/
				
				#pragma endregion Calibration X
				//***************************
				//***** Calibration Y new ***
				#pragma region Calibration Y
				else if (Event.reportObject.index == BUTTON_Y_1){
				genie.WriteStr(STRING_Y_CAB_VALUE,"1");
				offset_y_calib = -0.5;
				}
				else if (Event.reportObject.index == BUTTON_Y_2){
				genie.WriteStr(STRING_Y_CAB_VALUE,"2");
				offset_y_calib = -0.4;
				}
				else if (Event.reportObject.index == BUTTON_Y_3){
				genie.WriteStr(STRING_Y_CAB_VALUE,"3");
				offset_y_calib = -0.3;
				}
				else if (Event.reportObject.index == BUTTON_Y_4){
				genie.WriteStr(STRING_Y_CAB_VALUE,"4");
				offset_y_calib = -0.2;
				}
				else if (Event.reportObject.index == BUTTON_Y_5){
				genie.WriteStr(STRING_Y_CAB_VALUE,"5");
				offset_y_calib = -0.1;
				}
				else if (Event.reportObject.index == BUTTON_Y_6){
				genie.WriteStr(STRING_Y_CAB_VALUE,"6");
				offset_y_calib = 0;
				}
				else if (Event.reportObject.index == BUTTON_Y_7){
				genie.WriteStr(STRING_Y_CAB_VALUE,"7");
				offset_y_calib = 0.1;
				}
				else if (Event.reportObject.index == BUTTON_Y_8){
				genie.WriteStr(STRING_Y_CAB_VALUE,"8");
				offset_y_calib = 0.2;
				}
				else if (Event.reportObject.index == BUTTON_Y_9){
				genie.WriteStr(STRING_Y_CAB_VALUE,"9");
				offset_y_calib = 0.3;
				}
				else if (Event.reportObject.index == BUTTON_Y_10){
				genie.WriteStr(STRING_Y_CAB_VALUE,"10");
				offset_y_calib = 0.4;
				}
				
				else if (Event.reportObject.index == BUTTON_Y_ACCEPT){
				float calculus = extruder_offset[Y_AXIS][1] +offset_y_calib;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//enquecommand_P((PSTR("M218 T1 X-0.2")));
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				/*else if (Event.reportObject.index == BUTTON_Y_REDO){
				float calculus = extruder_offset[Y_AXIS][1] +offset_y_calib;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				
				//enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G41"));
				st_synchronize();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXEL,"Y AXEL, heatting");
				}*/
				
				#pragma endregion Calibration Y
				//***************************
				
				
				//***** Info Screens *****
				#pragma region Info Screens
				//BACKING FROM INFO SCREENS!------------------------------------------------------------------------
				//Backing from INFO SCREENS
				else if (Event.reportObject.index == BACKBUTTON_INFO_NEEDFIL)
				{
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIBRATION,0);
				}
				
				//Backing from INFO SCREENS
				else if (Event.reportObject.index == BACKBUTTON_INFO_INI_XYCALIB)
				{
				enquecommand_P(PSTR("G28 X0 Y0"));
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIBRATION,0);
				}
				
				//Backing from INFO SCREENS
				else if (Event.reportObject.index == BACKBUTTON_INFO_FIL_INSERTED)
				{
				
				}
				
				//Backing from INFO SCREENS
				else if (Event.reportObject.index == BACKBUTTON_INFO_PLACE_FIL)
				{
				setTargetHotend0(0);
				setTargetHotend1(0);
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FILAMENT,0);
				}
				
				//Backing from INFO SCREENS
				else if (Event.reportObject.index == BACKBUTTON_INFO_TURN_SCREWS)
				{
				enquecommand_P(PSTR("G28 X0 Y0"));
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIBRATION,0);
				}
				
				//Backing from INFO SCREENS
				else if (Event.reportObject.index == BACKBUTTON_INFO_BED_MUST_CAL)
				{
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIBRATION,0);
				}
				
				
				//BUTTON FROM INFO SCREENS!------------------------------------------------------------------------
				//Go! Button from INFO SCREENS
				else if (Event.reportObject.index == BUTTON_INFO_NEEDFIL)
				{
				//enquecommand_P(PSTR("G28"));
				//enquecommand_P(PSTR("G43"));
				genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_BED_MUST_CAL,0);
				}
				
				//Backing from INFO SCREENS
				else if (Event.reportObject.index == BUTTON_INFO_INI_XYCALIB)
				{
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G40"));
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
				}
				
				//Backing from INFO SCREENS
				else if (Event.reportObject.index == BUTTON_INFO_FIL_INSERTED)
				{
				
				}
				
				//Backing from INFO SCREENS
				else if (Event.reportObject.index == BUTTON_INFO_TURN_SCREWS)
				{
				char buffer[256];
				if (vuitens1!= 0){
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW1,0);
				sprintf(buffer, " %d / 8",vuitens1); //Printing how to calibrate on screen
				//genie.WriteStr(STRING_BED_SCREW1,buffer);
				if (vuitens2==0 && vuitens3==0) {genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW2,2);}
				else{genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW2,0);}
				if (sentit1>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW1,vuitens1);} //The direction is inverted in Sigma's bed screws
				else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW1,vuitens1+8);}
				}
				else if (vuitens2!= 0){
				Serial.println("Jump over screw1");
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW2,0);
				sprintf(buffer, " %d / 8",vuitens2); //Printing how to calibrate on screen
				//genie.WriteStr(STRING_BED_SCREW2,buffer);
				if (vuitens3==0) genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW3,2);
				if (sentit2>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,vuitens2);} //The direction is inverted in Sigma's bed screws
				else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,vuitens2+8);}
				}
				else if (vuitens3!= 0){
				Serial.println("Jump over screw1 and screw2");
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW3,0);
				sprintf(buffer, " %d / 8",vuitens3); //Printing how to calibrate on screen
				//genie.WriteStr(STRING_BED_SCREW3,buffer);
				if (sentit3>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,vuitens3);} //The direction is inverted in Sigma's bed screws
				else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,vuitens3+8);}
				}
				}
				
				//Backing from INFO SCREENS
				else if (Event.reportObject.index == BUTTON_INFO_BED_MUST_CAL)
				{
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G43"));
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
				}
				#pragma endregion Info Screens
				
				
				//***** Main Screen ******
				#pragma region main_screen

				
				#pragma endregion main_screen
				
				//*******Sleep screen
				#pragma region sleep_screen
				
				#pragma endregion sleep_screen
				
				}// else
				}//Userbuttons
				//USERBUTTONS------------------------------------------------------

				
			//FORMS--------------------------------------------------------
			if (Event.reportObject.object == GENIE_OBJ_FORM)
			{
				if (Event.reportObject.index == FORM_SDFILES)
				{
					//wake_RELAY();
					Serial.println("Form 2!");
					////Check sdcardFiles
				
					/*char filename[] = "123456789012345678901234567890.gcode";
					int count = 18;
					char buffer[String(filename).length()-6];
					memset( buffer, '\0', String(filename).length()-6 );
				
					if (String(filename).length()-6 > count){ //because we erase 6 characters of ".gcode"
					for (int i = 0; i<String(filename).length() ; i++) //we need 3 characters to write "..."
					{
					if (filename[i] == '.') i = String(filename).length() +10; //go out of the for
					else buffer[i]=filename[i];
					Serial.print(buffer[i]);
					}
					Serial.println("");
					//buffer[count]='\0';
					//char* buffer2 = strcat(buffer,"...\0");
					//genie.WriteStr(1,buffer2);//Printing form
					//***************CIRCLE ARRAY***************
					char show[count];
					for (int j = 0; j <= String(buffer).length() + 1; j++){
					for(int i = 0; i < count; i++){
					if (i+j == String(buffer).length()) show[i] = ' ';
					else show[i] = buffer[(i+j)%(String(buffer).length() + 1)];
					Serial.print(show[i]);
				
					}
					Serial.println(" ");
					delay(500);
					genie.WriteStr(1,show);
				
					}
					//////////////////////////////////////////////
					}
					else {
					for (int i = 0; i<String(filename).length() ; i++)	{
					if (filename[i] == '.') i = String(filename).length() +10; //go out of the for
					else buffer[i]=filename[i];
					}
					buffer[count]='\0';
					genie.WriteStr(1,buffer);//Printing form
					//Is a file
					//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
					}
					*/
					card.initsd();
					uint16_t fileCnt = card.getnrfilenames();
					//Declare filepointer
					card.getWorkDirName();
					//Text index starts at 0
					card.getfilename(filepointer);
					Serial.println(card.longFilename);
					if (card.filenameIsDir)
					{
						//Is a folder
						//genie.WriteStr(1,card.longFilename);
						//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,1);
					}else{
						int count = 18;
						char buffer[count];
						memset( buffer, '\0', sizeof(char)*count );
				
						if (String(card.longFilename).length()-6 > count){
							for (int i = 0; i<count-3 ; i++)
							{
								if (card.longFilename[i] == '.') i = count +10; //go out of the for
								else buffer[i]=card.longFilename[i];
							}
							buffer[count]='\0';
							char* buffer2 = strcat(buffer,"...\0");
							genie.WriteStr(STRING_NAME_FILE,buffer2);//Printing form
						} 
						else {
							for (int i = 0; i<String(card.longFilename).length() -6; i++)	{
								if (card.longFilename[i] == '.') i = String(card.longFilename).length() +10; //go out of the for
								else buffer[i]=card.longFilename[i];
							}
							//buffer[count]='\0';
							genie.WriteStr(STRING_NAME_FILE,buffer);//Printing form
							//Is a file
							//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
						}
						Serial.println(buffer);
						//***************CIRCLE ARRAY***************
						/*	char show[count];
						for (int j = 0; j <= String(card.longFilename).length() + 1; j++){
						for(int i = 0; i < count; i++){
						if (i+j == String(card.longFilename).length()) show[i] = ' ';
						else show[i] = buffer[(i+j)%(String(card.longFilename).length() + 1)];
						Serial.print(show[i]);
						delay(1000);
						}
						Serial.println("");
						}*/
					//////////////////////////////////////////////
					}
				}
				
				else if (Event.reportObject.index == FORM_PAUSE) //NOT USED!!!!!
				{
				//if (genie.ReadObject(GENIE_OBJ_USERBUTTON,BUTTON_PAUSE)){
				//genieFrame.reportObject.data_lsb
				//genie.GetcharSerial()
				//}
				//genie.
				//int value = genie.GetEventData(&Event);
				//if (value == 1) // Need to preheat
				//{
				//Button Pressed ON
				if (card.sdprinting)
				{
				card.pauseSDPrint();
				}
				//}
				//else
				//{
				//card.startFileprint();
				//}
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_PAUSED_PRINT,1);
				genie.WriteStr(10,card.longFilename);//PausedPrint form
				//Many ways to achieve this, we could check ReadObject to know if resume is UP or NOT
				}
				
				else if (Event.reportObject.index == FORM_STOP)
				{
				//plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS]+10,current_position[E_AXIS], 600, active_extruder);
				card.sdprinting = false;
				card.closefile();
				
				quickStop();
				
				if(SD_FINISHED_STEPPERRELEASE)
				{
				enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
				}
				autotempShutdown();
				setTargetHotend(0,active_extruder);
				setTargetBed(0);
				
				//If we are paused, unpause.
				card.sdispaused=false;
				
				//Rapduch
				genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PAUSE_RESUME,0);//Set pause Button to default position
				genie.WriteObject(GENIE_OBJ_FORM,5,0);//Exit to main menu
				}
				
				else if (Event.reportObject.index == FORM_START_PRINT)
				{
				//Not sure if useful to update strings here!!!
				//genie.WriteStr(7,card.longFilename);
				//genie.WriteStr(8,"Ready...");
				}
				
				else if (Event.reportObject.index == FORM_PRINTING)
				{
				//Restart the preheat buttons
				genie.WriteObject(GENIE_OBJ_USERIMAGES,BUTTON_PREHEAT_PLA,0);
				
				int count = 12;
				char buffer[count];
				if (String(card.longFilename).length()>count){
					for (int i = 0; i<count ; i++)
					{
						buffer[i]=card.longFilename[i];
					}
					buffer[count]='\0';
					char* buffer2 = strcat(buffer,"...\0");
					genie.WriteStr(STRINGS_PRINTING_GCODE,buffer2);//Printing form
				}else{
					for (int i = 0; i<=String(card.longFilename).length(); i++)
					{
						/*if (buffer[i] = '.') i = String(card.longFilename).length() +10;
						else */buffer[i]=card.longFilename[i];
					}
					buffer[count]='\0';
					genie.WriteStr(STRINGS_PRINTING_GCODE,buffer);//Printing form//Printing form
				}
				
				//genie.WriteStr(2,card.longFilename);
				//genie.WriteStr(6,"Printing...");
				}
				
				//else if (Event.reportObject.index == FORM_FEEDRATE)
				//{
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FEEDRATE,feedmultiply);
				//}
				
				//else if (Event.reportObject.index == FORM_NOZZLE)
				//{
				////genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[which_extruder]);
				//}
				
				//else if (Event.reportObject.index == FORM_BED)
				//{
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_BED,target_temperature_bed);
				//}
				
				//else if (Event.reportObject.index == FORM_FAN)
				//{
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FAN,fanSpeed);
				//}
				
				
				else if (Event.reportObject.index == FORM_MAIN_SCREEN)
				{
				surfing_utilities=false;
				Serial.println("Surfing 0");
				}
				
				else if (Event.reportObject.index == FORM_UTILITIES)
				{
				surfing_utilities=true;
				Serial.println("Surfing 1");
				}
				
				//else if (Event.reportObject.index == FORM_INSERT_FIL_PREHEAT)
				//{
				////setTargetHotend0(ABS_PREHEAT_HOTEND_TEMP);
				////setTargetHotend1(ABS_PREHEAT_HOTEND_TEMP);
				//}
				
				else if (Event.reportObject.index == FORM_PRINTING_SETTINGS)
				{
				Serial.println("Form Printing Settings");
				}
				
				else if (Event.reportObject.index == FORM_PREHEAT_SETTINGS)
				{
				//Rapduch
				//First send the actual command
				setTargetHotend0(PLA_PREHEAT_HOTEND_TEMP);
				setTargetHotend1(PLA_PREHEAT_HOTEND_TEMP);
				setTargetBed(PLA_PREHEAT_HPB_TEMP);
				
				//Now let's print it on the touchscreen
				char buffer[256];
				int tHotend=target_temperature[0];
				int tHotend1=target_temperature[1];
				int tBed=target_temperature_bed;
				
				//Serial.println("TARGET TEMPS");
				
				sprintf(buffer, "%3d",tHotend);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PREHEAT_SET_NOZZ1,buffer);
				
				sprintf(buffer, "%3d",tHotend1);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PREHEAT_SET_NOZZ2,buffer);
				
				sprintf(buffer, "%3d",tBed);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PREHEAT_SET_BED,buffer);
				}
				
				//else if (Event.reportObject.index == FORM_CALIB_BED_SCREW2)
				//{
				//char buffer[256];
				//sprintf(buffer, " %d / 8",vuitens2); //Printing how to calibrate on screen
				//genie.WriteStr(STRING_BED_SCREW2,buffer);
				//if (sentit2>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,1);} //The direction is inverted in Sigma's bed screws
				//else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,0);}
				//
				//Serial.print("vuitens2:");
				//Serial.print(vuitens2);
				//}
				}
				}
				}



				char* prepareString(char* text, int len){
				Serial.print("Text: ");
				Serial.println(text);
				if (String(text).length()>12){
				char buffer[len+1];
				for (int i = 0; i<len ; i++)
				{
				buffer[i]=card.longFilename[i];
				}
				buffer[len]='\0';
				Serial.print("Buffer temp: ");
				Serial.println(buffer);
				char* buffer2 = strcat(buffer,"...\0");
				Serial.print("Buffer returned: ");
				Serial.println(buffer2);
				return buffer2;
				//buffer2 = strcat(buffer,"...\0");
				}else{
				char* buffer2 = text;
				Serial.print("Buffer returned: ");
				Serial.println(buffer2);
				return buffer2;
				}
				}



				#endif /* INCLUDE */

				#endif