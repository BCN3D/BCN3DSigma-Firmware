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
void insertmetod();
extern bool cancel_heatup;
void myGenieEventHandler();
bool sdscreenblocked = false;
bool print_setting_refresh = false;
bool flag_filament_home= false;
bool filament_accept_ok = false;
bool flag_pause = false;
bool flag_resume = false;
bool flag_full_calib = false;
bool flag_bed_calib_done = false;
int  print_setting_tool = 2;
float offset_x_calib = 0;
float offset_y_calib = 0;
int  purge_extruder_selected = 0;
int  previous_state = FORM_MAIN_SCREEN;
int custom_insert_temp = 210;
int custom_remove_temp = 210;
int custom_print_temp = 210;
int custom_bed_temp = 40;
bool screen_sdcard = false;


int redo_source;


//Created by Jordi Calduch for RepRapBCN SIGMA 12/2014
void myGenieEventHandler(void) //Handler for the do.Events() function
{
	genieFrame Event;
	genie.DequeueEvent(&Event);
	//static long waitPeriod = millis();
	static long waitPeriod_purge = millis(); // This waitperiod avoid some spamming pressing on purging buttons which can block the machine
	int move_mm = 10;
	//If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
	if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
	{
		//*****Winbuttons*****
	/*	#pragma region Winbuttons
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
				home_axis_from_code(true,true,true);
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
		#pragma endregion Winbuttons*/
		
		//USERBUTTONS------------------------------------------------------
		
		if (Event.reportObject.object == GENIE_OBJ_USERBUTTON) //Userbuttons to select GCODE from SD
		{
			if (card.sdprinting || card.sdispaused){
				
				//******PRINTING****
				#pragma region Printing_screen
				
				if (Event.reportObject.index == BUTTON_PRINT_SETTINGS )
				{
					//Rapduch
					//Edit for final TouchScreen
					//Serial.println("PRINTING SETTINGS");
					char buffer[256];
					
					
					is_on_printing_screen=false;
					/*genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTTING_SETTINGS_DEF,0);
					
					
					sprintf(buffer, "%3d %cC",target_temperature[0],0x00B0);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PS_LEFT_TEMP,buffer);
					
					sprintf(buffer, "%3d %cC",target_temperature[1],0x00B0);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PS_RIGHT_TEMP,buffer);
					
					sprintf(buffer, "%3d %cC",target_temperature_bed,0x00B0);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PS_BED_TEMP,buffer);
					
					sprintf(buffer, "%3d %%",feedmultiply);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PS_SPEED,buffer);
					*/
					print_setting_refresh = true;
					
					
				}
				#pragma endregion Printing_screen			
				
				
				else if (Event.reportObject.index == BUTTON_STOP_YES )
				{
					is_on_printing_screen=false;
					card.sdprinting = false;
					card.closefile();
					dobloking =false;
					home_made = false;
					//plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS]+10,current_position[E_AXIS], 600, active_extruder);
					quickStop();
					
					enquecommand_P(PSTR("G28 X0 Y0")); //Home X and Y
					st_synchronize();
					enquecommand_P(PSTR("T0")); //The default states is Left Extruder active
					st_synchronize();
					
					setTargetHotend0(0);
					setTargetHotend1(0);
					setTargetBed(0);
					
					if(SD_FINISHED_STEPPERRELEASE)
					{
						enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
					}
					autotempShutdown();
					setTargetHotend0(0);
					setTargetHotend1(0);
					setTargetBed(0);
					
					card.sdispaused = false;
					cancel_heatup = true;
					
					//sleep_RELAY();
					//Rapduch
					genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
					back_home = true;
				}
				
				else if (Event.reportObject.index == BUTTON_PAUSE_RESUME && card.sdprinting)
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
				//We need to Resume/Enter Printing Settings/Stop printing
				else if (Event.reportObject.index == BUTTON_PAUSE_RESUME && card.sdispaused)
				{
					//I believe it is a really unsafe way to do it
					//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]-20, current_position[E_AXIS], homing_feedrate[Z_AXIS]/60, RIGHT_EXTRUDER);
					//st_synchronize();
					card.startFileprint();
					Serial.println("RESUME!");
					flag_pause = false;
					flag_resume = true;
					if(flag_resume){
						enquecommand_P(((PSTR("G70"))));
						flag_resume = false;
						Serial.println("resume detected");
					}
				}
				
				
				//*****Printing Settings*****
				#pragma region Printing Settings
				else if (Event.reportObject.index == BUTTON_PRINT_SET_SPEED_UP)
				{
					//char buffer[256];
					//int value=5;
					
					
					screen_change_speedup = true;
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_SPEED_DOWN)
				{
					//char buffer[256];
					//int value=5;
					
					screen_change_speeddown = true;
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ1_UP)
				{
					//char buffer[256];
					//int value=5;
					
					screen_change_nozz1up = true;
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ1_DOWN)
				{
					//char buffer[256];
					//int value=5;
					
					screen_change_nozz1down = true;
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ2_UP)
				{
					//char buffer[256];
					//int value=5;
					
					screen_change_nozz2up = true;
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ2_DOWN)
				{
					//char buffer[256];
					//int value=5;
					
					screen_change_nozz2down = true;
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_BED_UP)
				{
					//char buffer[256];
					//int value=5;
					//if (target_temperature_bed<BED_MAXTEMP)
					
					screen_change_bedup = true;
				}
				
				else if (Event.reportObject.index == BUTTON_PRINT_SET_BED_DOWN)
				{
					//char buffer[256];
					//int value=5;
					//if (target_temperature_bed>BED_MINTEMP)
					
					
					screen_change_beddown = true;
				}
				
				#pragma endregion Printing Settings
				
				
				}else{//All that has to be done out of the printing room
				
				
				
				if (Event.reportObject.index == BUTTON_CHANGE_EXTRUDER)
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
				
				/*else if (Event.reportObject.index == BUTTON_PRINT_SETTINGS )
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
					
					sprintf(buffer, "%3d %cC",tHotend,0x00B0);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PRINT_SET_NOZZ1,buffer);
					
					sprintf(buffer, "%3d %cC",tHotend1,0x00B0);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PRINT_SET_NOZZ2,buffer);
					
					sprintf(buffer, "%3d %cC",tBed,0x00B0);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PRINT_SET_BED,buffer);
					
					sprintf(buffer, "%3d %%",feedmultiply);
					//Serial.println(buffer);
					genie.WriteStr(STRING_PRINT_SET_PERCENT,buffer);
				}*/
				
				else if (Event.reportObject.index == BUTTON_STOP_YES )
				{
					is_on_printing_screen=false;
					card.sdprinting = false;
					card.closefile();
					
					//plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS]+10,current_position[E_AXIS], 600, active_extruder);
					quickStop();
					
					home_axis_from_code(true,true,false); //Home X and Y
					enquecommand_P(PSTR("T0")); //The default states is Left Extruder active
					enquecommand_P(PSTR("M107"));
					st_synchronize();
					if(SD_FINISHED_STEPPERRELEASE)
					{
						enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
					}
					autotempShutdown();
					setTargetHotend0(0);
					setTargetHotend1(0);
					//setTargetHotend2(0);
					setTargetBed(0);
					card.sdispaused = false;
					cancel_heatup = true;
					
					
					//sleep_RELAY();
					//Rapduch
					genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				}
				
				
				
				//*****SD Gcode Selection*****
				#pragma region SD Gcode Selector
				else if ((Event.reportObject.index == BUTTON_SD_SELECTED ||  Event.reportObject.index == STRING_NAME_FILE) && !sdscreenblocked)
				{
					if(card.cardOK)
					{						
						dobloking = true;
						setTargetBed(0);
						setTargetHotend0(0);
						setTargetHotend1(0);
						log_prints++;
						Config_StoreSettings();
						//enquecommand_P(PSTR("T0"));
						st_synchronize();
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
				
				
				else if ((Event.reportObject.index == BUTTON_SD_LEFT || Event.reportObject.index == BUTTON_SD_RIGHT) && !sdscreenblocked) //TODO: control if SD is out
				{
					if (card.cardOK){
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
						int line = 23;
						int count = 63;
						char buffer[count+3];
						int x = 0;
						memset( buffer, '\0', sizeof(char)*count );
						
						if (String(card.longFilename).length() > count){
							for (int i = 0; i<count ; i++)
							{
								if (card.longFilename[i] == '.') i = count +10;
								else if(i == 0) buffer[i]=card.longFilename[x];
								else if (i%line == 0){
									buffer[i] = '\n';
									i++;
									buffer[i]=card.longFilename[x];
								}
								else {
									buffer[i]=card.longFilename[x];
								}
								Serial.print(i);
								x++;
							}
							buffer[count]='\0';
							char* buffer2 = strcat(buffer,"...\0");
							genie.WriteStr(STRING_NAME_FILE,buffer2);//Printing form
						}
						else {
							for (int i = 0; i<=String(card.longFilename).length() ; i++)	{
								if (card.longFilename[i] == '.') i = count +10;
								else if(i == 0) buffer[i]=card.longFilename[x];
								else if (i%line == 0){
									buffer[i] = '\n';
									i++;
									buffer[i]=card.longFilename[x];
								}
								else {
									buffer[i]=card.longFilename[x];
								}
								Serial.print(i);
								x++;
							}
							buffer[count]='\0';
							genie.WriteStr(STRING_NAME_FILE,buffer);//Printing form
							//Is a file
							//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
							
						}
						
						//Keep in mind to control the length of the string displayed!
						//genie.WriteStr(2,card.longFilename);
						Serial.print("Image n: ");
						Serial.println(filepointer);
					}
				}
				#pragma endregion SD Gcode Selector

				#pragma region PURGE
				//****************PURGE BUTTONS******
				else if (Event.reportObject.index == BUTTON_PURGE_LEFT ){
					if (purge_extruder_selected == 1){
						purge_extruder_selected = 0;
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
						char buffer[256];
						sprintf(buffer, "%d %cC",target_temperature[0],0x00B0);
						genie.WriteStr(STRING_PURGE_SELECTED,buffer);
						Serial.println(buffer);
						sprintf(buffer, "%3d %cC",int(degHotend(0)),0x00B0);
						genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);
					}	
					else{
						char buffer[256];
						purge_extruder_selected = 0;
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
						sprintf(buffer, "%3d %cC",int(degHotend(0)),0x00B0);
						genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);
					}
				}
				else if (Event.reportObject.index == BUTTON_PURGE_RIGHT ){
					if (purge_extruder_selected == 0){
						purge_extruder_selected = 1;
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,1);
						char buffer[256];
						sprintf(buffer, "%3d %cC",target_temperature[1],0x00B0);
						genie.WriteStr(STRING_PURGE_SELECTED,buffer);
						Serial.println(buffer);
						sprintf(buffer, "%3d %cC",int(degHotend(1)),0x00B0);
						genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);
					}
					else{
						purge_extruder_selected = 0;
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,1);
						char buffer[256];
						sprintf(buffer, "%3d %cC",int(degHotend(1)),0x00B0);
						genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);
					}
				}
				else if(Event.reportObject.index == BUTTON_PURGE_TEMP_UP){
					
					if (target_temperature[purge_extruder_selected] < HEATER_0_MAXTEMP){
						 target_temperature[purge_extruder_selected] += 5;
						setTargetHotend0(target_temperature[0]);
						setTargetHotend1(target_temperature[1]);
						char buffer[256];
						sprintf(buffer, "%3d %cC",int(target_temperature[purge_extruder_selected]),0x00B0);
						genie.WriteStr(STRING_PURGE_SELECTED,buffer);
					}
				}
				else if(Event.reportObject.index == BUTTON_PURGE_TEMP_DOWN){
					if (target_temperature[purge_extruder_selected] > 0){
						target_temperature[purge_extruder_selected] -= 5;
						setTargetHotend0(target_temperature[0]);
						setTargetHotend1(target_temperature[1]);
						char buffer[256];
						sprintf(buffer, "%3d %cC",int(target_temperature[purge_extruder_selected]),0x00B0);
						genie.WriteStr(STRING_PURGE_SELECTED,buffer);
					}
				}
				//***MOVING
				else if(Event.reportObject.index == BUTTON_PURGE_RETRACK){
					if (millis() >= waitPeriod_purge){
						if(degHotend(purge_extruder_selected) >= target_temperature[purge_extruder_selected]-5){
							current_position[E_AXIS]-=3;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60, purge_extruder_selected);//Retrack
						}
						waitPeriod_purge=millis()+1500;
					}
				}
				else if(Event.reportObject.index == BUTTON_PURGE_INSERT){
					if (millis() >= waitPeriod_purge){
						if(degHotend(purge_extruder_selected) >= target_temperature[purge_extruder_selected]-5){
							current_position[E_AXIS]+=3;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60, purge_extruder_selected);//Purge
						}
						waitPeriod_purge=millis()+1500;
					}
				}
				else if(Event.reportObject.index == BUTTON_PURGE_INSERTX3){
					if (millis() >= waitPeriod_purge){
						if(degHotend(purge_extruder_selected) >= target_temperature[purge_extruder_selected]-5){
							current_position[E_AXIS]+=15;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60, purge_extruder_selected);//Purge
						}
						waitPeriod_purge=millis()+7500;
					}
				}
				//***************************************
				//********CHANGE NEW FILAMENT
				/*else if(Event.reportObject.index == BUTTON_PURGE_NEW_FILAMENT){
					
					
					genie.WriteObject(GENIE_OBJ_USERIMAGES,10,0);
					genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
					
					if (purge_extruder_selected == 0) setTargetHotend0(INSERT_FIL_TEMP);
					else setTargetHotend1(INSERT_FIL_TEMP);
					
					if (!home_made) home_axis_from_code(true,true,true);
					home_axis_from_code(true,true,false);
					st_synchronize();
					current_position[Z_AXIS]=Z_MAX_POS-15;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS]*2/60, active_extruder);
					
					while (degHotend(purge_extruder_selected)<=(degTargetHotend(purge_extruder_selected)-5)){ //Waiting to heat the extruder						
						manage_heater();
					}					
					filament_mode = 'C';
					genie.WriteStr(STRING_FILAMENT,"Press GO and keep pushing the filament \n until starts being pulled");
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INSERT_FIL,0);
					genie.WriteStr(STRING_FILAMENT,"Press GO and keep pushing the filament \n until starts being pulled");
					
				}*/
				
				//****************************************
				
				else if(Event.reportObject.index == BUTTON_PURGE){
										
					genie.WriteObject(GENIE_OBJ_FORM,FORM_PURGE,0);
					
					
					Serial.println("Enter in purge mode");
					setTargetHotend0(print_temp_l);
					setTargetHotend1(print_temp_r);
					if(purge_extruder_selected == 0) {
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
						
					}
					else {
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,1);						
					}
					
					char buffer[256];
					sprintf(buffer, "%3d %cC",int(degHotend(0)),0x00B0);
					genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);	Serial.println(buffer);
					sprintf(buffer, "%3d %cC",int(degHotend(1)),0x00B0);
					genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);	Serial.println(buffer);
					sprintf(buffer, "%3d %cC",int(target_temperature[purge_extruder_selected]),0x00B0);
					genie.WriteStr(STRING_PURGE_SELECTED,buffer);	Serial.println(buffer);
					
					
				}
				else if(Event.reportObject.index	== BUTTON_PURGE_BACK){
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FILAMENT,0);
					//setTargetHotend0(0);
					//setTargetHotend1(0);
									
				}
				
				//************************************
				#pragma endregion PURGE		
				
				
				
				
				/*else if (Event.reportObject.index == BUTTON_BED_UP )
				{
				int value=5;
				//if (target_temperature_bed<BED_MAXTEMP)
				if (target_temperature_bed<120)//MaxTemp
				{
				target_temperature_bed+=value;
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_BED,target_temperature_bed);
				}
				}*/
				
				/*else if (Event.reportObject.index == BUTTON_BED_DOWN )
				{
				int value=5;
				//if (target_temperature_bed>BED_MINTEMP)
				if (target_temperature_bed>5)//Mintemp
				{
				target_temperature_bed-=value;
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_BED,target_temperature_bed);
				}
				}*/
				
				/*else if (Event.reportObject.index == BUTTON_FAN_UP )
				{
				int value=5;
				if (fanSpeed<255)
				{
				fanSpeed+=value;
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FAN,fanSpeed);
				}
				}*/
				
				/*else if (Event.reportObject.index == BUTTON_FAN_DOWN )
				{
				int value=5;
				if (fanSpeed>0)
				{
				fanSpeed-=value;
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FAN,fanSpeed);
				}
				}*/
				
				/*else if (Event.reportObject.index == BUTTON_SETUP_BACK )
				{
					//Get status string
					char buffer[15];
					screen_status.toCharArray(buffer,15,0);
				
					if (card.sdprinting || card.sdispaused)
					{
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING,0);
						//Restore Strings after form update
						genie.WriteStr(2,card.longFilename);
						//genie.WriteStr(6,buffer);
					}else
					{
						genie.WriteObject(GENIE_OBJ_FORM,FORM_START_PRINT,0);
						//Restore Strings
						genie.WriteStr(7,card.longFilename);
						genie.WriteStr(8,buffer);
					}
				}*/
				
				
				/*else if (Event.reportObject.index == BUTTON_SETUP_BACK_NOZZLE || Event.reportObject.index == BUTTON_SETUP_BACK_BED )
				{
				if (surfing_utilities) // Check if we are backing from utilities or print setup
				{
				genie.WriteObject(GENIE_OBJ_FORM,FORM_TEMPERATURE,0);
				}else
				{
				genie.WriteObject(GENIE_OBJ_FORM,FORM_SETUP,0);
				}
				}*/
				
				
				
				
				
				//NOZZLE BUTTONS reusability---------------------------------------------------
				/*else if (Event.reportObject.index == BUTTON_NOZZLE1_PRINT || Event.reportObject.index == BUTTON_NOZZLE2_PRINT || Event.reportObject.index == BUTTON_NOZZLE1_TEMP || Event.reportObject.index == BUTTON_NOZZLE2_TEMP)
				{
				if (Event.reportObject.index == BUTTON_NOZZLE1_PRINT || Event.reportObject.index == 
				BUTTON_NOZZLE1_TEMP)
				{
				which_extruder=0;
				}else{
				which_extruder=1;
				}
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[which_extruder]);
				genie.WriteObject(GENIE_OBJ_FORM,FORM_NOZZLE,0);
				}
				
				else if (Event.reportObject.index == BUTTON_NOZZLE_UP )
				{
				int value=5;
				if (target_temperature[which_extruder]<HEATER_0_MAXTEMP)
				{
				target_temperature[which_extruder]+=value;
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[which_extruder]);
				}
				}
				
				else if (Event.reportObject.index == BUTTON_NOZZLE_DOWN )
				{
				int value=5;
				if (target_temperature[which_extruder]>HEATER_0_MINTEMP)
				{
				target_temperature[which_extruder]-=value;
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[which_extruder]);
				}
				}
				
				
				
				NOZZLEBUTTONS-------
				else if (Event.reportObject.index == 148  )
				{
					language = (language%8)+1;
					enquecommand_P(PSTR("M500"));
					updateLanguage();
					Serial.println(idiom[IDIOM_CURRENT_LANGUAGE]);
				}*/
				
				//*****INSERT/REMOVE FILAMENT*****
				#pragma region Insert_Remove_Fil
				
				else if (Event.reportObject.index == BUTTON_FILAMENT_BACK  )
				{
					flag_filament_home=false;
					genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES,0);
				}
				
				else if (Event.reportObject.index == BUTTON_INSERT_FIL || Event.reportObject.index == BUTTON_REMOVE_FIL )
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
					if(which_extruder == 0) setTargetHotend(max(remove_temp_l,old_remove_temp_l),which_extruder);
					else setTargetHotend(max(remove_temp_r,old_remove_temp_r),which_extruder);
					
					if (filament_mode == 'I') {
						if (which_extruder == 0)	genie.WriteObject(GENIE_OBJ_FORM,FORM_LEFT_MATERIAL,0);
						else genie.WriteObject(GENIE_OBJ_FORM,FORM_RIGHT_MATERIAL,0);
					}
					
					else {
						//*********Move the bed down and the extruders inside
						processing = true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						
						home_axis_from_code(true,true,true);
					
						int feedrate;
						if (!flag_filament_home){
							//MOVING THE EXTRUDERS TO AVOID HITTING THE CASE WHEN PROBING-------------------------
							//current_position[X_AXIS]+=25;						
							home_axis_from_code(true,true,false);
							st_synchronize();
							
							flag_filament_home=true;
						}	
						/*	
					
						current_position[Z_AXIS]=Z_MAX_POS-15;
						current_position[Y_AXIS]=10;
						feedrate=max_feedrate[Z_AXIS];
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate, active_extruder); //check speed
						st_synchronize();
						
						
						*/
						
						current_position[Z_AXIS]=Z_MAX_POS-15;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder); //check speed
						
						current_position[Y_AXIS]=10;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Y_AXIS], active_extruder); //check speed
						st_synchronize();
						
						
						processing = false;
						
						genie.WriteObject(GENIE_OBJ_USERIMAGES,10,1);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
						genie.WriteObject(GENIE_OBJ_USERIMAGES,10,1);				
						/****************************************************/
					
						//ATTENTION : Order here is important
						
						
						//Serial.println("REMOVING");		
						//genie.WriteStr(STRING_ADVISE_FILAMENT,"");
						if (filament_mode == 'I') genie.WriteObject(GENIE_OBJ_USERIMAGES,10,0);
						else if (filament_mode == 'R') genie.WriteObject(GENIE_OBJ_USERIMAGES,10,1);
						else genie.WriteObject(GENIE_OBJ_USERIMAGES,10,1);
						delay(3500);
						/*if(which_extruder == 0) setTargetHotend(max(remove_temp_l,old_remove_temp_l),which_extruder);
						else setTargetHotend(max(remove_temp_r,old_remove_temp_r),which_extruder);*/
						is_changing_filament=true; //We are changing filament	
						
					}	
				}
				
				else if (Event.reportObject.index == BUTTON_PLA_R){
					print_temp_r = PLA_PRINT_TEMP;
					insert_temp_r = PLA_INSERT_TEMP;
					remove_temp_r = PLA_REMOVE_TEMP;
					bed_temp_r = PLA_BED_TEMP;
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
					Config_StoreSettings();
					setTargetHotend1(print_temp_r);
					insertmetod();
				}
				
				else if (Event.reportObject.index == BUTTON_ABS_R){
					print_temp_r = ABS_PRINT_TEMP;
					insert_temp_r = ABS_INSERT_TEMP;
					remove_temp_r = ABS_REMOVE_TEMP;
					bed_temp_r = ABS_BED_TEMP;
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
					Config_StoreSettings();
					setTargetHotend1(print_temp_r);
					insertmetod();
				}
				else if (Event.reportObject.index == BUTTON_PVA_R){
					print_temp_r = PVA_PRINT_TEMP;
					insert_temp_r = PVA_INSERT_TEMP;
					remove_temp_r = PVA_REMOVE_TEMP;
					bed_temp_r = PVA_BED_TEMP;
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
					Config_StoreSettings();
					setTargetHotend1(print_temp_r);
					insertmetod();
				}
				else if (Event.reportObject.index == BUTTON_PLA_L){
					print_temp_l = PLA_PRINT_TEMP;
					insert_temp_l = PLA_INSERT_TEMP;
					remove_temp_l = PLA_REMOVE_TEMP;
					bed_temp_l = PLA_BED_TEMP;
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
					Config_StoreSettings();
					setTargetHotend0(print_temp_l);
					insertmetod();
				}
				
				else if (Event.reportObject.index == BUTTON_ABS_L){
					print_temp_l = ABS_PRINT_TEMP;
					insert_temp_l = ABS_INSERT_TEMP;
					remove_temp_l = ABS_REMOVE_TEMP;
					bed_temp_l = ABS_BED_TEMP;
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
					Config_StoreSettings();
					setTargetHotend0(print_temp_l);
					
					
					//processing = true;
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
					insertmetod();
					
					
					
					
					
				}
				else if (Event.reportObject.index == BUTTON_PVA_L){
					print_temp_l = PVA_PRINT_TEMP;
					insert_temp_l = PVA_INSERT_TEMP;
					remove_temp_l = PVA_REMOVE_TEMP;
					bed_temp_l = PVA_BED_TEMP;
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
					Config_StoreSettings();
					setTargetHotend0(print_temp_l);
					insertmetod();
				}
				
				//CUSTOM MATERIAL BUTTONS
				else if((Event.reportObject.index == BUTTON_CUST_L) || (Event.reportObject.index == BUTTON_CUST_R) ){
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CUSTOM_MATERIAL,0);
					char buffer[10];
					char buffer1[10];
					char buffer2[10];
					char buffer3[256];
					sprintf(buffer, "%d %cC",custom_insert_temp,0x00B0);
					genie.WriteStr(STRING_CUSTOM_INSERT,buffer);
					sprintf(buffer1, "%d %cC",custom_remove_temp,0x00B0);
					genie.WriteStr(STRING_CUSTOM_REMOVE,buffer1);
					sprintf(buffer2, "%d %cC",custom_print_temp,0x00B0);
					genie.WriteStr(STRING_CUSTOM_PRINT,buffer2);
					sprintf(buffer3, "%d %cC",custom_bed_temp,0x00B0);
					genie.WriteStr(STRING_CUSTOM_BED,buffer3);
					
				}
				else if (Event.reportObject.index == BUTTON_CUSTOM_BACK){
					if (which_extruder == 0) genie.WriteObject(GENIE_OBJ_FORM,FORM_LEFT_MATERIAL,0);
					else genie.WriteObject(GENIE_OBJ_FORM,FORM_RIGHT_MATERIAL,0);
				}
				
				else if(Event.reportObject.index == BUTTON_CUSTOM_INS_LESS){
					if (custom_insert_temp > 0){
						char buffer[256];
						custom_insert_temp -= 10;
						sprintf(buffer, "%1d %cC",custom_insert_temp,0x00B0);
						genie.WriteStr(STRING_CUSTOM_INSERT,buffer);	
					}					
				}
				else if(Event.reportObject.index == BUTTON_CUSTOM_INS_MORE){
					if (custom_insert_temp < HEATER_0_MAXTEMP-5){
						char buffer[256];
						custom_insert_temp += 10;
						sprintf(buffer, "%1d %cC",custom_insert_temp,0x00B0);
						genie.WriteStr(STRING_CUSTOM_INSERT,buffer);
					}
				}
				else if(Event.reportObject.index == BUTTON_CUSTOM_REM_LESS){
					if (custom_remove_temp > 0){
						char buffer[256];
						custom_remove_temp -= 10;
						sprintf(buffer, "%1d %cC",custom_remove_temp,0x00B0);
						genie.WriteStr(STRING_CUSTOM_REMOVE,buffer);
					}
				}
				else if(Event.reportObject.index == BUTTON_CUSTOM_REM_MORE){
					if (custom_remove_temp < HEATER_0_MAXTEMP-5){
						char buffer[256];
						custom_remove_temp += 10;
						sprintf(buffer, "%1d %cC",custom_remove_temp,0x00B0);
						genie.WriteStr(STRING_CUSTOM_REMOVE,buffer);
					}
				}
				else if(Event.reportObject.index == BUTTON_CUSTOM_PRINT_LESS){
					if (custom_print_temp > 0){
						char buffer[256];
						custom_print_temp -= 10;
						sprintf(buffer, "%1d %cC",custom_print_temp,0x00B0);
						genie.WriteStr(STRING_CUSTOM_PRINT,buffer);
					}
				}
				else if(Event.reportObject.index == BUTTON_CUSTOM_PRINT_MORE){
					if (custom_print_temp < HEATER_0_MAXTEMP-5){
						char buffer[256];
						custom_print_temp += 10;
						sprintf(buffer, "%1d %cC",custom_print_temp,0x00B0);
						genie.WriteStr(STRING_CUSTOM_PRINT,buffer);
					}
				}
				else if(Event.reportObject.index == BUTTON_CUSTOM_BED_LESS){
					if (custom_bed_temp > 0){
						char buffer[256];
						custom_bed_temp -= 10;
						sprintf(buffer, "%1d %cC",custom_bed_temp,0x00B0);
						genie.WriteStr(STRING_CUSTOM_BED,buffer);
					}
				}
				else if(Event.reportObject.index == BUTTON_CUSTOM_BED_MORE){
					if (custom_bed_temp < BED_MAXTEMP -5){
						char buffer[256];
						custom_bed_temp += 10;
						sprintf(buffer, "%1d %cC",custom_bed_temp,0x00B0);
						genie.WriteStr(STRING_CUSTOM_BED,buffer);
					}
				}
				else if(Event.reportObject.index == BUTTON_CUSTOM_ACCEPT){
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
					if(which_extruder == 0){
						if (custom_print_temp <= HEATER_0_MAXTEMP) print_temp_l = custom_print_temp;
						else print_temp_l = HEATER_0_MAXTEMP;
						if (custom_insert_temp <= HEATER_0_MAXTEMP) insert_temp_l = custom_insert_temp;
						else insert_temp_l = HEATER_0_MAXTEMP;
						if (custom_remove_temp <= HEATER_0_MAXTEMP) remove_temp_l = custom_remove_temp;
						else remove_temp_l = HEATER_0_MAXTEMP;
						if (custom_bed_temp <= BED_MAXTEMP) bed_temp_l = custom_bed_temp;
						else bed_temp_l = BED_MAXTEMP;
						setTargetHotend0(insert_temp_l);
					}
					else{
						if (custom_print_temp <= HEATER_1_MAXTEMP)print_temp_r = custom_print_temp;
						else print_temp_r = HEATER_1_MAXTEMP;
						if (custom_insert_temp <= HEATER_1_MAXTEMP)insert_temp_r = custom_insert_temp;
						else print_temp_r = HEATER_1_MAXTEMP;
						if (custom_remove_temp <= HEATER_1_MAXTEMP)remove_temp_r = custom_remove_temp;
						else remove_temp_r = HEATER_1_MAXTEMP;
						if (custom_bed_temp <= BED_MAXTEMP)bed_temp_r = custom_bed_temp;
						else bed_temp_r = BED_MAXTEMP;
						setTargetHotend1(insert_temp_r);
					}
					Config_StoreSettings();
				}
				
				//////////////////////////
				else if (Event.reportObject.index == BUTTON_INFO_FIL_INSERTED)
				{
					
					//*********Move the bed down and the extruders inside
					processing = true;
					genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
					if (!home_made) home_axis_from_code(true,true,true);
					
					int feedrate;
					if (!flag_filament_home){
						//MOVING THE EXTRUDERS TO AVOID HITTING THE CASE WHEN PROBING-------------------------
						//current_position[X_AXIS]+=25;
						
						home_axis_from_code(true,true,false);						
						/*current_position[X_AXIS] = x_home_pos(LEFT_EXTRUDER)+25;
						feedrate=homing_feedrate[X_AXIS];
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, LEFT_EXTRUDER);
						st_synchronize();
						//current_position[X_AXIS] = x_home_pos(RIGHT_EXTRUDER);
					
						current_position[X_AXIS]=extruder_offset[X_AXIS][1];
						Serial.println(current_position[X_AXIS]);
						plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
						//current_position[X_AXIS]-=25;
						current_position[X_AXIS] = x_home_pos(RIGHT_EXTRUDER)-25;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
						st_synchronize();*/
						flag_filament_home=true;
					}
					
					/*current_position[Y_AXIS]=285;
					feedrate=homing_feedrate[Y_AXIS];
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					st_synchronize();*/
					
					current_position[Z_AXIS]=Z_MAX_POS-15;
					feedrate=homing_feedrate[Z_AXIS];
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate*2/60, active_extruder); //check speed
					st_synchronize();					
					/****************************************************/
										
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INSERT_FILAMENT_TOP,0);
					processing = false;
					
				}

				else if(Event.reportObject.index == BUTTON_MOVE_INSERT){					
					
					//ATTENTION : Order here is important
					genie.WriteObject(GENIE_OBJ_USERIMAGES,10,0);
					genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);					
					//genie.WriteStr(STRING_ADVISE_FILAMENT,"");
					//genie.WriteStr(STRING_ADVISE_FILAMENT,"Insert the filament until you feel it stops, \n then while you keep inserting around \n 10 mm of filament, press the clip");
					genie.WriteObject(GENIE_OBJ_USERIMAGES,10,0);
					
					if (which_extruder==0) setTargetHotend(max(insert_temp_l,old_insert_temp_l),which_extruder);
					else setTargetHotend(max(insert_temp_r,old_insert_temp_r),which_extruder);
					delay(3500);
					is_changing_filament=true;
					
					if (which_extruder == 0) changeTool(0);
					else changeTool(1);
					
					current_position[Y_AXIS] = 100;
					current_position[X_AXIS] = 155;					
					plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
					st_synchronize();
				}
				
				
				
				else if (Event.reportObject.index == BUTTON_INSERT )
				{// We should have already checked if filament is inserted
					if (filament_mode =='I')
					{ //Inserting...
						processing = true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						delay(850);
						Serial.print("Inserting :   ");
						current_position[E_AXIS] += 30;//Extra extrusion at low feedrate
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  700/60, which_extruder); //850/60
						current_position[E_AXIS] += ((BOWDEN_LENGTH-EXTRUDER_LENGTH)-15);//BOWDEN_LENGTH-300+340);
						Serial.println(current_position[E_AXIS]);
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, which_extruder);
						current_position[E_AXIS] += EXTRUDER_LENGTH;//Extra extrusion at low feedrate
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_SLOW_SPEED/60, which_extruder);
						
						
						st_synchronize();
						processing = false;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUST_FILAMENT,0);
						
					}else if (filament_mode =='R')
					{ //Removing...
						current_position[E_AXIS] -= (BOWDEN_LENGTH + EXTRUDER_LENGTH + 100);//Extra extrusion at fast feedrate
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_FAST_SPEED/60, which_extruder);
						previous_state = FORM_FILAMENT;
						processing = true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						st_synchronize();
						processing = false;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_SUCCESS_FILAMENT,0);
						if (which_extruder == 0){
							old_insert_temp_l = insert_temp_l;
							old_remove_temp_l = remove_temp_l;
							old_print_temp_l  = print_temp_l;
						}
						else{
							old_insert_temp_r = insert_temp_r;
							old_remove_temp_r = remove_temp_r;
							old_print_temp_r  = print_temp_r;
						}						
					}else if (filament_mode == 'C'){	
						previous_state = FORM_FILAMENT;	
						processing = true;			
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						delay(850);
						Serial.print("Inserting :   ");
						current_position[E_AXIS] += 60;//Extra extrusion at low feedrate
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_SLOW_SPEED/60, which_extruder); //850/60
						current_position[E_AXIS] += (BOWDEN_LENGTH-EXTRUDER_LENGTH);//BOWDEN_LENGTH-300+340);
						Serial.println(current_position[E_AXIS]);
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60, which_extruder);
						current_position[E_AXIS] += EXTRUDER_LENGTH;//Extra extrusion at low feedrate
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_SLOW_SPEED/60, which_extruder);						
						
						st_synchronize();
						Serial.println(current_position[E_AXIS]);
						processing = false;	
						genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUST_FILAMENT,0);
					}
					
					//We prefer to maintain temp after changing filament
					//setTargetHotend(0,which_extruder);
					//put_info_text("Filament DONE");
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES,0);
				}
				#pragma endregion Insert_Remove_Fil
				
				
				//*****AdjustFilament******
				#pragma region AdjustFilament
				else if (Event.reportObject.index == BUTTON_ACCEPT_ADJUST && filament_accept_ok == false)
				{
					
					if (millis() >= waitPeriod_purge){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);	
					filament_accept_ok = true;
					home_made = false;
					processing=true;
					home_axis_from_code(true,true,false);
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_SUCCESS_FILAMENT,0);	
					
					
					}
					
					
					
					
					/*if (quick_guide){
						if (quick_guide_step == 1) genie.WriteObject(GENIE_OBJ_FORM,FORM_QUICK_RIGHT,0);
						else if(quick_guide_step == 2) genie.WriteObject(GENIE_OBJ_FORM,FORM_QUICK_CALIBRATE,0);
					}
					else {*/
						
					//}
					//setTargetHotend0(0);
					//setTargetHotend1(0);
					//changeTool(0);
				}
				
				else if (Event.reportObject.index == BUTTON_ADJUST_ZUp  && filament_accept_ok == false)
				{
					if (millis() >= waitPeriod_purge){
					//Adjusting the filament with a retrack Up
					Serial.println("Adjust ZUp");
					float modified_position=current_position[E_AXIS]-6;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], modified_position, INSERT_SLOW_SPEED/60, which_extruder);
					current_position[E_AXIS]=modified_position;
					
					waitPeriod_purge=millis()+3000;
				}
					
				}
				
				else if (Event.reportObject.index == BUTTON_ADJUST_ZDown  && filament_accept_ok == false)
				{
					//Adjusting the filament with a purge Down
					if (millis() >= waitPeriod_purge){
					Serial.println("Adjust ZDown");
					float modified_position=current_position[E_AXIS]+6;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], modified_position, INSERT_SLOW_SPEED/60, which_extruder);
					current_position[E_AXIS]=modified_position;
					waitPeriod_purge=millis()+3000;
				}
				}
				#pragma endregion AdjustFilament
				
				
				
				//Extruder Calibrations-------------------------------------------------
				else if (Event.reportObject.index == BUTTON_CAL_FULL)
				{
					bed_calibration_times = 0;
					Serial.print("INFO: BED CALIB - ");
					Serial.println(flag_bed_calib_done);
					flag_full_calib = true;
					
					//enquecommand_P(PSTR("T0"));
					if(!flag_bed_calib_done){  //Do g34
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteStr(STRING_AXIS,"BED");
						home_axis_from_code(true,true,true);
						enquecommand_P(PSTR("G34"));	//Start BED Calibration Wizard
						changeTool(0);						
						st_synchronize();
					}
					else{ //Do Z clean
						//genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						//home_axis_from_code(true,true,true);
						active_extruder = LEFT_EXTRUDER;
						genie.WriteStr(STRING_AXIS,"        Z AXIS");
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteStr(STRING_AXIS,"        Z AXIS");
						delay(1500);
						
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
						genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
						genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_EXTRUDERS,0);
						
						//changeToolSigma(LEFT_EXTRUDER);
						genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
						
						
						//Wait until temperature it's okey
						setTargetHotend0(EXTRUDER_LEFT_CLEAN_TEMP);
						setTargetHotend1(EXTRUDER_RIGHT_CLEAN_TEMP);
						setTargetBed(max(bed_temp_l,bed_temp_r));
						
						//MOVE EXTRUDERS
						current_position[Z_AXIS] = 60;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS]*2/60, LEFT_EXTRUDER);//move bed
						st_synchronize();
						current_position[X_AXIS] = 155; current_position[Y_AXIS] = 0;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS]/3, LEFT_EXTRUDER);//move first extruder
						
						dobloking=true;
						
						while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) && degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5)){ //Waiting to heat the extruder
							
							manage_heater();
						}
						
						//home_axis_from_code();			
						
						
						genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,1);
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,1);
						genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Clean the left nozzle \n and press GO to move on to \n the next EXTRUDER");
						flag_continue_calib = true;
					}
				}
				
				else if ((Event.reportObject.index == USERBUTTON_CLEAN_DONE) && (flag_continue_calib)){
					
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Clean the right nozzle \n and press GO, \n then the Z calibration will start");
					if (active_extruder == 0)	{	
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteStr(STRING_AXIS,"        Z AXIS");						
											
						home_axis_from_code(true,false,false);						
						dobloking=false;
						enquecommand_P(PSTR("G43"));
						flag_continue_calib = false;								
						
					}	
					else {
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteStr(STRING_AXIS,"        Z AXIS");
						
						home_axis_from_code(true,false,false);
						dobloking=false;
						enquecommand_P(PSTR("G43"));
						st_synchronize();			
					
						flag_continue_calib = false;						
					}				
						
				}	
				else if (Event.reportObject.index == BUTTON_PRINT_SET_BACK )
				{
					is_on_printing_screen=true;
					int count = 12;
					char buffer[13];
					if (String(card.longFilename).length()>12){
						for (int i = 0; i<12 ; i++)
						{
							buffer[i]=card.longFilename[i];
						}
						buffer[12]='\0';
						char* buffer2 = strcat(buffer,"...\0");
						Serial.print("Card Name: ");
						Serial.println(card.longFilename);
						Serial.print("Buffer1: ");
						Serial.println(buffer);
						Serial.print("buffer out: ");
						Serial.println(buffer2);
						genie.WriteStr(STRINGS_PRINTING_GCODE,buffer2);//Printing form
					}else{
						for (int i = 0; i<=String(card.longFilename).length(); i++)
						{
							if (buffer[i] == '.') i = String(card.longFilename).length() +10;
							else buffer[i]=card.longFilename[i];
						}
						//buffer[count]='\0';
						genie.WriteStr(STRINGS_PRINTING_GCODE,buffer);//Printing form//Printing form
					}
					genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING,1);
				}
				
				//*****Preheat Settings*****
				#pragma region Preheat Settings
				//Buttons for preheat settings
				else if(Event.reportObject.index == BUTTON_COOLDOWN_OK){
					setTargetHotend0(0);
					setTargetHotend1(0);
					setTargetBed(0);
					genie.WriteObject(GENIE_OBJ_FORM,FORM_TEMP_MENU,0);
				}
				
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_NOZZ0_UP )
				{
					char buffer[256];
					int value=5;
					if (preheat_E0_value<HEATER_0_MAXTEMP)
					{						
						preheat_E0_value+=value;
						sprintf(buffer, "%3d%cC",preheat_E0_value,0x00B0);
						genie.WriteStr(STRING_PREHEAT_SET_NOZZ1,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_NOZZ0_DOWN )
				{
				char buffer[256];
					int value=5;
					if (preheat_E0_value>HEATER_0_MINTEMP)
					{
						preheat_E0_value-=value;
						sprintf(buffer, "%3d%cC",preheat_E0_value,0x00B0);
						genie.WriteStr(STRING_PREHEAT_SET_NOZZ1,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_NOZZ1_UP )
				{
					char buffer[256];
					int value=5;
					if (preheat_E1_value<HEATER_1_MAXTEMP)
					{
						preheat_E1_value+=value;
						sprintf(buffer, "%3d%cC",preheat_E1_value,0x00B0);
						genie.WriteStr(STRING_PREHEAT_SET_NOZZ2,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_NOZZ1_DOWN )
				{
					char buffer[256];
					int value=5;
					if (preheat_E1_value<HEATER_1_MAXTEMP)
					{
						preheat_E1_value-=value;
						sprintf(buffer, "%3d%cC",preheat_E1_value,0x00B0);
						genie.WriteStr(STRING_PREHEAT_SET_NOZZ2,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_BED_UP )
				{
					char buffer[256];
					int value=5;
					//if (target_temperature_bed<BED_MAXTEMP)
					if (preheat_B_value<120)//MaxTemp
					{
						preheat_B_value+=value;
						sprintf(buffer, "%3d%cC",preheat_B_value,0x00B0);
						genie.WriteStr(STRING_PREHEAT_SET_BED,buffer);
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_BED_DOWN )
				{
					char buffer[256];
					int value=5;
					//if (target_temperature_bed>BED_MINTEMP)
					if (preheat_B_value>5)//Mintemp
					{
						preheat_B_value-=value;
						sprintf(buffer, "%3d%cC",preheat_B_value,0x00B0);
						genie.WriteStr(STRING_PREHEAT_SET_BED,buffer);
					}
				}
				else if(Event.reportObject.index == BUTTON_PREHEAT_ACCEPT){
					setTargetHotend0(preheat_E0_value);	
					setTargetHotend1(preheat_E1_value);
					setTargetBed(preheat_B_value);					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_TEMP_MENU,0);
				}
				else if (Event.reportObject.index == BUTTON_PREHEAT_SET_BACK )
				{
					//Cooldown				
					genie.WriteObject(GENIE_OBJ_FORM,FORM_TEMP_MENU,0);					
				}
				#pragma endregion Preheat Settings
				
				
				//*****Bed Calibration*****
				#pragma region Bed Calibration
				else if (Event.reportObject.index == BUTTON_Z_CAL_WIZARD)
				{
					processing = true;
					genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
					bed_calibration_times = 0;
					flag_full_calib = false;
					home_axis_from_code(true,true,true);				
					enquecommand_P(PSTR("T0"));
					enquecommand_P(PSTR("G34"));	//Start BED Calibration Wizard
					previous_state = FORM_CALIBRATION;
					
				}
				
				else if (Event.reportObject.index == BUTTON_REDO_BED_CALIB )
				{	
					processing = true;
					genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);		
					home_axis_from_code(true,false,false);	
					enquecommand_P((PSTR("T0")));
					enquecommand_P((PSTR("G34")));
					previous_state = FORM_CALIBRATION;
					flag_bed_calib_done = true;
				}
				
				else if (Event.reportObject.index == BUTTON_BED_CALIB_SW2)
				{
					char buffer[256];
					if (vuitens2!=0){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW2,0);
						sprintf(buffer, " %d / 8",vuitens2); //Printing how to calibrate on screen
						//genie.WriteStr(STRING_BED_SCREW2,buffer);
						if (vuitens3==0) genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW3,0);
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
						processing = true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						home_axis_from_code(true,false,false);	
						enquecommand_P((PSTR("T0")));
						enquecommand_P((PSTR("G34")));
						previous_state = FORM_CALIBRATION;
						
						flag_bed_calib_done = true;
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
						processing = true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						home_axis_from_code(true,false,false);	
						enquecommand_P((PSTR("T0")));
						enquecommand_P((PSTR("G34")));
						previous_state = FORM_CALIBRATION;
						
						flag_bed_calib_done = true;
					}
				}
				#pragma endregion Bed Calibration
				else if (Event.reportObject.index == BUTTON_INFO_TURN_SCREWS)
				{
					char buffer[256];
					if (vuitens1!= 0){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW1,0);
						sprintf(buffer, " %d / 8",vuitens1); //Printing how to calibrate on screen
						//genie.WriteStr(STRING_BED_SCREW1,buffer);
						if (vuitens2==0 && vuitens3==0) {genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW2,0);}
						else{genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW2,0);}
						if (sentit1>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW1,vuitens1);} //The direction is inverted in Sigma's bed screws
						else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW1,vuitens1+8);}
					}
					else if (vuitens2!= 0){
						Serial.println("Jump over screw1");
						genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW2,0);
						sprintf(buffer, " %d / 8",vuitens2); //Printing how to calibrate on screen
						//genie.WriteStr(STRING_BED_SCREW2,buffer);
						if (vuitens3==0) genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW3,0);
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
				
				//*****Success Screens*****
				#pragma region SuccessScreens
				else if (Event.reportObject.index == BUTTON_BED_CALIB_SUCCESS )
				{
					//enquecommand_P((PSTR("G28 X0 Y0")));
					enquecommand_P((PSTR("T0")));
					st_synchronize();
					Serial.println("Calibration Successful, going back to main menu");
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIBRATION,0);
					flag_bed_calib_done = true;
					//quick_guide = false;				
				}
				
				else if (Event.reportObject.index == BUTTON_SUCCESS_FILAMENT_OK)
				{
					//enquecommand_P((PSTR("G28 X0 Y0")));
					enquecommand_P((PSTR("T0")));
					Serial.println("Filament Inserted/Removed, returning to Main Menu");
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FILAMENT,0);
					//setTargetHotend0(0);
					//setTargetHotend1(0);
					filament_accept_ok = false;
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
					Config_StoreSettings(); //Store data
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
					if(flag_full_calib){
					//enquecommand_P(PSTR("G28"));
					st_synchronize();
					enquecommand_P(PSTR("G41"));
				
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"Y AXIS, Heating");
				
					}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT2)
				{
				float calculus = extruder_offset[X_AXIS][1] + 0.4;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				Config_StoreSettings(); //Store data
				if(flag_full_calib){
				
				enquecommand_P(PSTR("G41"));
			
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXIS,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT3)
				{
					float calculus = extruder_offset[X_AXIS][1] + 0.3;
					Serial.print("Calculus:  ");
					Serial.println(calculus);
					extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
					Config_StoreSettings(); //Store changes
					if(flag_full_calib){				
						enquecommand_P(PSTR("G41"));
				
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteStr(STRING_AXIS,"Y AXIS, Heating");
				
					}
					else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT4)
				{
				float calculus = extruder_offset[X_AXIS][1] + 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				Config_StoreSettings(); //Store data
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));				
				enquecommand_P(PSTR("G41"));
				
				
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXIS,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT5)
				{
					float calculus = extruder_offset[X_AXIS][1] + 0.1;
					Serial.print("Calculus:  ");
					Serial.println(calculus);
					extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
					Config_StoreSettings(); //Store changes
					if(flag_full_calib){
						enquecommand_P(PSTR("G41"));				
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteStr(STRING_AXIS,"Y AXIS, Heating");
					}
					else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT6)
				{
					float calculus = extruder_offset[X_AXIS][1];
					Serial.print("Calculus:  ");
					Serial.println(calculus);
					extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
					Config_StoreSettings(); //Store changes
					if(flag_full_calib){
				
						enquecommand_P(PSTR("G41"));
				
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteStr(STRING_AXIS,"Y AXIS, Heating");
				
					}
					else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT7)
				{
				float calculus = extruder_offset[X_AXIS][1] - 0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				Config_StoreSettings(); //Store changes
				if(flag_full_calib){				
					enquecommand_P(PSTR("G41"));
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT8)
				{
				float calculus = extruder_offset[X_AXIS][1] - 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				Config_StoreSettings(); //Store changes
				if(flag_full_calib){
				
				enquecommand_P(PSTR("G41"));
				
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXIS,"Y AXIS, Heating");
				
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
				Config_StoreSettings(); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				if(flag_full_calib){
				//enquecommand_P(PSTR("G28"));
				st_synchronize();
				enquecommand_P(PSTR("G41"));
				
				genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
				genie.WriteStr(STRING_AXIS,"Y AXIS, Heating");
				
				}else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_X_LINE_SELECT10)
				{
					float calculus = extruder_offset[X_AXIS][1]-0.4;
					Serial.print("Calculus:  ");
					Serial.println(calculus);
					extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
					Config_StoreSettings(); //Store changes
					if(flag_full_calib){
						
						enquecommand_P(PSTR("G41"));
				
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteStr(STRING_AXIS,"Y AXIS, Heating");
				
					}
					else genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				}
				
				else if (Event.reportObject.index == BUTTON_REDO_LEFT_CAB)
				{
					redo_source = 1;
					float calculus = extruder_offset[X_AXIS][1] + 0.5;
					Serial.print("Calculus:  ");
					Serial.println(calculus);					
					extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;					
					Config_StoreSettings(); //Store changes
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
				}
				
				else if (Event.reportObject.index == BUTTON_REDO_RIGHT_CAB)
				{
					redo_source = 1;
					float calculus = extruder_offset[X_AXIS][1] -0.4;
					Serial.print("Calculus:  ");
					Serial.println(calculus);
					extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
					Config_StoreSettings(); //Store changes
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
					
					
				}
				else if (Event.reportObject.index == BUTTON_REDO_X_CAB)
				{
					redo_source = 1;
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
										
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT1)
				{
				//char buffer[30];
				float calculus = extruder_offset[Y_AXIS][1] + 0.5;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				Config_StoreSettings(); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT2)
				{
				//enquecommand_P((PSTR("M218 T1 X-0.4")));
				float calculus = extruder_offset[Y_AXIS][1] + 0.4;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				Config_StoreSettings(); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT3)
				{
				float calculus = extruder_offset[Y_AXIS][1] + 0.3;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//enquecommand_P((PSTR("M218 T1 X-0.3")));
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				Config_StoreSettings(); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT4)
				{
				float calculus = extruder_offset[Y_AXIS][1] + 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//enquecommand_P((PSTR("M218 T1 X-0.2")));
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				Config_StoreSettings(); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT5)
				{
				float calculus = extruder_offset[Y_AXIS][1] + 0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.1")));
				Config_StoreSettings(); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT6)
				{
				float calculus = extruder_offset[Y_AXIS][1];
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				Config_StoreSettings(); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT7)
				{
				float calculus = extruder_offset[X_AXIS][1] - 0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				Config_StoreSettings(); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT8)
				{
				float calculus = extruder_offset[Y_AXIS][1] - 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				Config_StoreSettings(); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT9)
				{
				float calculus = extruder_offset[Y_AXIS][1] - 0.3;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//enquecommand_P((PSTR("M218 T1 X-0.2")));
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				Config_StoreSettings(); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);
				}
				
				else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT10)
				{
				float calculus = extruder_offset[Y_AXIS][1] - 0.4;
				Serial.print("Calculus:  ");
				
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				Config_StoreSettings(); //Store changes
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				flag_full_calib = false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);				
				}
				
				else if (Event.reportObject.index == BUTTON_REDO_UP_CAB)
				{
					
					redo_source = 2;
					float calculus = extruder_offset[Y_AXIS][1] + 0.5;
					Serial.print("Calculus:  ");
					Serial.println(calculus);
					extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
					Config_StoreSettings(); //Store changes
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
					
								
				}
				
				else if (Event.reportObject.index == BUTTON_REDO_DOWN_CAB)
				{
					redo_source = 2;
					float calculus = extruder_offset[Y_AXIS][1] -0.4;
					Serial.print("Calculus:  ");
					Serial.println(calculus);
					extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
					Config_StoreSettings(); //Store changes
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
					
					
					
				}
				else if (Event.reportObject.index == BUTTON_REDO_Y_CAB)
				{
					redo_source = 2;
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
					
				}
				
				else if (Event.reportObject.index == BUTTON_Z_CALIB_Z1_Down)
				{
				float feedrate = homing_feedrate[Z_AXIS];
				current_position[Z_AXIS] += 0.05;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				Serial.print("Z position: ");
				Serial.println(current_position[Z_AXIS]);
				}
				
				else if (Event.reportObject.index == BUTTON_Z_CALIB_Z1_Up)
				{
				float feedrate = homing_feedrate[Z_AXIS];
				if (current_position[Z_AXIS]>-1.5) current_position[Z_AXIS] -= 0.05; //Max down is Z=-0.5
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				Serial.print("Z position: ");
				Serial.println(current_position[Z_AXIS]);
				}
				
				else if (Event.reportObject.index == BUTTON_Z_CALIB_Z1_OK)
				{
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_Z_PRINT,0);
					Serial.println("OK first Extruder!");
					//We have to override z_prove_offset
					zprobe_zoffset-=(current_position[Z_AXIS]); //We are putting more offset if needed
					extruder_offset[Z_AXIS][LEFT_EXTRUDER]=0.0;//It is always the reference
					/*current_position[Z_AXIS]=0;//We are setting this position as Zero
					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);*/
					
					setTargetHotend0(print_temp_l);
				
					Serial.print("Z1 Probe offset: ");
					Serial.println(zprobe_zoffset);
					Config_StoreSettings(); //Store changes		
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
					
					current_position[Z_AXIS] += 2;
					plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],homing_feedrate[Z_AXIS]/60,active_extruder);
					st_synchronize();
					
					home_axis_from_code(true,false,false);
											
					while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-10) || degBed()<(target_temperature_bed)-10){ //Waiting to heat the extruder
						manage_heater();
					}				
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_Z_PRINT,0);
					delay(6000);
					home_axis_from_code(true,true,true);						
					left_test_print_code();
				}
				
				else if (Event.reportObject.index == BUTTON_Z_CALIB_Z2_Down )
				{
					float feedrate = homing_feedrate[Z_AXIS];
					current_position[Z_AXIS] += 0.05;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					Serial.print("Z position: ");
					Serial.println(current_position[Z_AXIS]);
				}
				
				else if (Event.reportObject.index == BUTTON_Z_CALIB_Z2_Up)
				{
				float feedrate = homing_feedrate[Z_AXIS];
				if (current_position[Z_AXIS]>-1.5) current_position[Z_AXIS] -= 0.05;  //Max down is Z=-0.5
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				Serial.print("Z position: ");
				Serial.println(current_position[Z_AXIS]);
				}
				
				else if (Event.reportObject.index == BUTTON_Z_CALIB_Z2_OK)
				{
					Serial.println("OK second Extruder!");
					extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=(current_position[Z_AXIS]);//Add the difference to the current offset value
					Serial.print("Z2 Offset: ");
					Serial.println(extruder_offset[Z_AXIS][RIGHT_EXTRUDER]);
					Config_StoreSettings(); //Store changes					
					
					setTargetHotend1(print_temp_r);					
					st_synchronize();						
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
					
					current_position[Z_AXIS] += 2;
					plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],homing_feedrate[Z_AXIS]/60,active_extruder);
					st_synchronize();
					
					home_axis_from_code(true,false,false);	
									
					while (degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-10) || degBed()<(target_temperature_bed)-10){ //Waiting to heat the extruder
						
						manage_heater();
					}
					delay(6000);
					
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_Z_PRINT,0);
					home_axis_from_code(true,true,true);	
					right_test_print_code(); 
					
				}
				else if((Event.reportObject.index == BUTTON_REDO_LEFT)|| (Event.reportObject.index == BUTTON_REDO_RIGHT)){
					
					genie.WriteObject(GENIE_OBJ_FORM, FORM_REDO_Z_TEST,0);
				}
				else if(Event.reportObject.index == BUTTON_Z_LEFT_SELECT1){
										
					genie.WriteStr(STRING_AXIS,"        Z AXIS");
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"        Z AXIS");
					delay(1500);
					
					active_extruder = RIGHT_EXTRUDER;														
					zprobe_zoffset+=0.05;
					Config_StoreSettings(); //Store changes
					
					changeTool(1);				
					
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_EXTRUDERS,0);
					
					//changeToolSigma(LEFT_EXTRUDER);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
					
					
					//Wait until temperature it's okey					
					setTargetHotend1(EXTRUDER_RIGHT_CLEAN_TEMP);
					
					home_axis_from_code(true,true,true);
					//changeTool(LEFT_EXTRUDER);
					
					while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) || degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5) || degBed()<(max(bed_temp_l,bed_temp_r)-15)){ //Waiting to heat the extruder
						
						manage_heater();
					}
					
					//MOVE EXTRUDERS
					current_position[Z_AXIS] = 60;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS]*2/60, RIGHT_EXTRUDER);//move bed
					st_synchronize();
					current_position[X_AXIS] = 155; current_position[Y_AXIS] = 0;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS]/3, RIGHT_EXTRUDER);//move first extruder
										
					dobloking=true;								
										
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,1);
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,1);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Clean the right nozzle \n and press GO to move on to \n the next EXTRUDER");
					flag_continue_calib = true;
				}
				else if(Event.reportObject.index == BUTTON_Z_LEFT_SELECT2){
					genie.WriteStr(STRING_AXIS,"        Z AXIS");
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"        Z AXIS");
					delay(1500);
					
					active_extruder = RIGHT_EXTRUDER;
					
					changeTool(1);
					
					
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_EXTRUDERS,0);
					
					//changeToolSigma(LEFT_EXTRUDER);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
					
					
					//Wait until temperature it's okey
					setTargetHotend1(EXTRUDER_RIGHT_CLEAN_TEMP);
					
					home_axis_from_code(true,true,true);
					//changeTool(LEFT_EXTRUDER);
					
					while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) || degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5) || degBed()<(max(bed_temp_l,bed_temp_r)-15)){ //Waiting to heat the extruder
						
						manage_heater();
					}
					
					//MOVE EXTRUDERS
					current_position[Z_AXIS] = 60;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS]*2/60, RIGHT_EXTRUDER);//move bed
					st_synchronize();
					current_position[X_AXIS] = 155; current_position[Y_AXIS] = 0;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS]/3, RIGHT_EXTRUDER);//move first extruder
								
					dobloking = true;
										
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,1);
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,1);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Clean the right nozzle \n and press GO to move on to \n the next EXTRUDER");
					flag_continue_calib = true;
				
				}
				else if(Event.reportObject.index == BUTTON_Z_LEFT_SELECT3){
					genie.WriteStr(STRING_AXIS,"        Z AXIS");
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"        Z AXIS");
					delay(1500);
					
					active_extruder = RIGHT_EXTRUDER;														
					zprobe_zoffset-=0.05;
					Config_StoreSettings(); //Store changes
					
					changeTool(1);
					
					/*enquecommand_P(PSTR("G43"));
					st_synchronize();*/	
					
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_EXTRUDERS,0);
					
					//changeToolSigma(LEFT_EXTRUDER);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
					
					
					//Wait until temperature it's okey
					setTargetHotend1(EXTRUDER_RIGHT_CLEAN_TEMP);
					
					home_axis_from_code(true,true,true);
					//changeTool(LEFT_EXTRUDER);
					
					while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) || degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5) || degBed()<(max(bed_temp_l,bed_temp_r)-15)){ //Waiting to heat the extruder
						
						manage_heater();
					}
					
					//MOVE EXTRUDERS
					current_position[Z_AXIS] = 60;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS]*2/60, RIGHT_EXTRUDER);//move bed
					st_synchronize();
					current_position[X_AXIS] = 155; current_position[Y_AXIS] = 0;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS]/3, RIGHT_EXTRUDER);//move first extruder
					
					dobloking = true;		
								
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,1);
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,1);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Clean the right nozzle \n and press GO to move on to \n the next EXTRUDER");
					flag_continue_calib = true;			
				}
				else if(Event.reportObject.index == BUTTON_Z_LEFT_SELECT4){
					genie.WriteStr(STRING_AXIS,"        Z AXIS");
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"        Z AXIS");
					delay(1500);
					
					active_extruder = RIGHT_EXTRUDER;														
					zprobe_zoffset-=0.1;
					Config_StoreSettings(); //Store changes
					
					changeTool(1);
					
					/*enquecommand_P(PSTR("G43"));
					st_synchronize();*/	
					
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_EXTRUDERS,0);
					
					//changeToolSigma(LEFT_EXTRUDER);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
					
					
					//Wait until temperature it's okey
					setTargetHotend1(EXTRUDER_RIGHT_CLEAN_TEMP);
					
					home_axis_from_code(true,true,true);
					//changeTool(LEFT_EXTRUDER);
					
					while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) || degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5) || degBed()<(max(bed_temp_l,bed_temp_r)-15)){ //Waiting to heat the extruder
						
						manage_heater();
					}
					
					//MOVE EXTRUDERS
					current_position[Z_AXIS] = 60;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS]*2/60, RIGHT_EXTRUDER);//move bed
					st_synchronize();
					current_position[X_AXIS] = 155; current_position[Y_AXIS] = 0;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS]/3, RIGHT_EXTRUDER);//move first extruder
					
					dobloking = true;	
									
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,1);
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,1);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Clean the right nozzle \n and press GO to move on to \n the next EXTRUDER");
					flag_continue_calib = true;
				}
				else if(Event.reportObject.index == BUTTON_Z_LEFT_SELECT5){
					genie.WriteStr(STRING_AXIS,"        Z AXIS");
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"        Z AXIS");
					delay(1500);
					
					active_extruder = RIGHT_EXTRUDER;														
					zprobe_zoffset-=0.15;
					Config_StoreSettings(); //Store changes
					
					changeTool(1);
					
					/*enquecommand_P(PSTR("G43"));
					st_synchronize();*/	
					
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_EXTRUDERS,0);
					
					//changeToolSigma(LEFT_EXTRUDER);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
					
					
					//Wait until temperature it's okey
					setTargetHotend1(EXTRUDER_RIGHT_CLEAN_TEMP);
					
					home_axis_from_code(true,true,true);
					//changeTool(LEFT_EXTRUDER);
					
					while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) || degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5) || degBed()<(max(bed_temp_l,bed_temp_r)-15)){ //Waiting to heat the extruder
						
						manage_heater();
					}
					
					//MOVE EXTRUDERS
					current_position[Z_AXIS] = 60;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS]*2/60, RIGHT_EXTRUDER);//move bed
					st_synchronize();
					current_position[X_AXIS] = 155; current_position[Y_AXIS] = 0;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS]/3, RIGHT_EXTRUDER);//move first extruder
					
					dobloking = true;	
									
					genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,1);
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,1);
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Clean the right nozzle \n and press GO to move on to \n the next EXTRUDER");
					flag_continue_calib = true;
				}
				
				else if(Event.reportObject.index == BUTTON_RECALIBRATE_Z){					
					redo_source = 3;
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);			
					
				}
				
				else if(Event.reportObject.index == BUTTON_CLEAN_BED){
					if(redo_source == 0){		 //redo z test print				
						if (active_extruder==0){
							processing = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							home_axis_from_code(true,true,true);
							left_test_print_code();
							processing =false;
						}
						else{
							processing = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							home_axis_from_code(true,true,true);
							right_test_print_code();
							processing =false;
						}
					}
					else if(redo_source == 1){ //redo x test print						
						current_position[Z_AXIS] = 0.2;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						enquecommand_P((PSTR("G40")));
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteStr(STRING_AXIS,"X AXIS, Heating...");
					}
					else if(redo_source == 2){ //redo y test print
						current_position[Z_AXIS] = 0.3;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						enquecommand_P((PSTR("G41")));
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteStr(STRING_AXIS,"Y AXIS, Heating...");
					}
					else if(redo_source == 3){	//recalibrate
						if (active_extruder == 0){
							processing = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							current_position[E_AXIS]-=4;
							plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],INSERT_FAST_SPEED/60,LEFT_EXTRUDER);
							st_synchronize();
							setTargetHotend0(170);
							home_axis_from_code(true,true,true);
							enquecommand_P(PSTR("G43"));
							processing = false;
						}
						else{
							processing = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							current_position[E_AXIS]-=4;
							plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],INSERT_FAST_SPEED/60,RIGHT_EXTRUDER);
							st_synchronize();
							setTargetHotend1(170);
							home_axis_from_code(true,true,true);
							enquecommand_P(PSTR("G43"));
							processing = false;
						}
					}
				}
				else if(Event.reportObject.index == BUTTON_REDO_Z_1){
					redo_source = 0;
					if (active_extruder == 0){						
						zprobe_zoffset+=0.1;	
						Config_StoreSettings(); //Store changes
						enquecommand_P(PSTR("T1"));
						enquecommand_P(PSTR("T0"));
						st_synchronize();	
						
					}
					else{						
						extruder_offset[Z_AXIS][RIGHT_EXTRUDER]+=0.1;
						Config_StoreSettings(); //Store changes
						enquecommand_P(PSTR("T0"));
						enquecommand_P(PSTR("T1"));
						st_synchronize();
						
					}
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
				}
				else if(Event.reportObject.index == BUTTON_REDO_Z_5){
					redo_source = 0;
					if (active_extruder == 0){
						
						zprobe_zoffset-=0.1;
						Config_StoreSettings(); //Store changes					
						enquecommand_P(PSTR("T1"));
						enquecommand_P(PSTR("T0"));
						st_synchronize();	
					}
					else{						
						extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=0.1;
						Config_StoreSettings(); //Store changes
						enquecommand_P(PSTR("T0"));
						enquecommand_P(PSTR("T1"));
						st_synchronize();											
					}
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
				}
				else if(Event.reportObject.index == BUTTON_REDO_Z){
					redo_source = 0;
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);	
				}
				
				else if(Event.reportObject.index == BUTTON_Z_RIGHT_SELECT1){
					extruder_offset[Z_AXIS][RIGHT_EXTRUDER]+=0.05;
					Config_StoreSettings(); //Store changes
					enquecommand_P(PSTR("T0"));
					enquecommand_P(PSTR("G40"));
					st_synchronize();
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"X AXIS, Heating...");
					
				}
				else if(Event.reportObject.index == BUTTON_Z_RIGHT_SELECT2){					
								
					enquecommand_P(PSTR("T0"));
					enquecommand_P(PSTR("G40"));
					st_synchronize();
										
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"X AXIS, Heating...");
					
				}
				else if(Event.reportObject.index == BUTTON_Z_RIGHT_SELECT3){
					extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=0.05;
					Config_StoreSettings(); //Store changes
					enquecommand_P(PSTR("T0"));
					enquecommand_P(PSTR("G40"));
					st_synchronize();
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"X AXIS, Heating...");
					
				}
				else if(Event.reportObject.index == BUTTON_Z_RIGHT_SELECT4){
					extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=0.1;
					Config_StoreSettings(); //Store changes
					enquecommand_P(PSTR("T0"));
					enquecommand_P(PSTR("G40"));
					st_synchronize();
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"X AXIS, Heating...");
					
				}
				else if(Event.reportObject.index == BUTTON_Z_RIGHT_SELECT5){
					extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=0.15;
					Config_StoreSettings(); //Store changes
					enquecommand_P(PSTR("T0"));
					enquecommand_P(PSTR("G40"));
					st_synchronize();
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"X AXIS, Heating...");
					
				}
				#pragma endregion CalibrationsXYZ
				
				#pragma region QUICK START
				//*****START QUICK GUIDE
				
				/*else if (Event.reportObject.index == BUTTON_QUICK_INSERT_LEFT || Event.reportObject.index == BUTTON_QUICK_INSERT_LEFT2)
				{
					quick_guide_step = 1;
					which_extruder=0;
					filament_mode = 'I';
					//setTargetHotend(REMOVE_FIL_TEMP,which_extruder);
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
					surfing_utilities = true;
					
				}
				
				else if (Event.reportObject.index == BUTTON_QUICK_INSERT_RIGHT || Event.reportObject.index == BUTTON_QUICK_INSERT_RIGHT2 )
				{
					quick_guide_step = 2;
					which_extruder=1;
					//setTargetHotend(REMOVE_FIL_TEMP,which_extruder);
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
				}
				
				else if (Event.reportObject.index == BUTTON_QUICK_FULLCALIB )
				{
					genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_QUICK_MESAGE_CALIB,1);
					genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_QUICK_START_FULLCALIB,1);
				}
				
				else if (Event.reportObject.index == BUTTON_QUICK_START_FULLCALIB)
				{
					bed_calibration_times = 0;
					Serial.print("INFO: BED CALIB - ");
					Serial.println(flag_bed_calib_done);
					flag_full_calib = true;
					
					//enquecommand_P(PSTR("T0"));
					if(!flag_bed_calib_done){  //Do g34
						home_axis_from_code(true,true,true);
						enquecommand_P(PSTR("G34"));	//Start BED Calibration Wizard
						changeTool(0);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_QUICK_MESAGE_CALIB,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_QUICK_START_FULLCALIB,0);
						genie.WriteStr(STRING_AXEL,"BED");
						st_synchronize();
					}
					else{ //Do Z clean
						//genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						//enquecommand_P(PSTR("G28"));
						active_extruder = LEFT_EXTRUDER;
						genie.WriteStr(STRING_AXEL,"        Z AXIS");
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteStr(STRING_AXEL,"        Z AXIS");
						delay(1500);
						
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
						genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
						genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_EXTRUDERS,0);
						
						//changeToolSigma(LEFT_EXTRUDER);
						genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
						
						
						//Wait until temperature it's okey
						setTargetHotend0(EXTRUDER_LEFT_CLEAN_TEMP);
						setTargetHotend1(EXTRUDER_RIGHT_CLEAN_TEMP);
						
						//MOVE EXTRUDERS
						current_position[Z_AXIS] = 60;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS]*2/60, LEFT_EXTRUDER);//move bed
						st_synchronize();
						current_position[X_AXIS] = 155; current_position[Y_AXIS] = 0;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS]/3, LEFT_EXTRUDER);//move first extruder
						
						while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) && degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5)){ //Waiting to heat the extruder
							
							manage_heater();
						}
						
						//home_axis_from_code();
						
						
						genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,1);
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,1);
						genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Clean the left nozzle \n and press GO to move on to \n the next EXTRUDER");
						flag_continue_calib = true;
					}
				}
				
				*/
				//*****END QUICK GUIDE
				#pragma endregion QUICK START
				
				
				//***** Info Screens *****
				#pragma region Info Screens					
				
				//Backing from INFO SCREENS								
				else if (Event.reportObject.index == BACKBUTTON_CALIBRATION)
				{					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES,0);
					flag_bed_calib_done = false;
				}
				
				else if (Event.reportObject.index == BACKBUTTON_INSERT)
				{
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FILAMENT,0);
				}
				//SKIP BED CALIBRATION
				else if (Event.reportObject.index == BUTTON_SKIP_BED)
				{
					if (flag_full_calib){					
						bed_calibration_times = 0;
						//genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						//enquecommand_P(PSTR("G28"));
						active_extruder = LEFT_EXTRUDER;
						genie.WriteStr(STRING_AXIS,"        Z AXIS");
						genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
						genie.WriteStr(STRING_AXIS,"        Z AXIS");
						delay(1500);
						
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
						genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
						genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,0);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_EXTRUDERS,0);
						
						//changeToolSigma(LEFT_EXTRUDER);
						genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Wait until the image \n turns red, the \n EXTRUDER are heating up");
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,0);
						
						
						//Wait until temperature it's okey
						setTargetHotend0(EXTRUDER_LEFT_CLEAN_TEMP);
						setTargetHotend1(EXTRUDER_RIGHT_CLEAN_TEMP);
						
						home_axis_from_code(true,true,true);
						//changeTool(LEFT_EXTRUDER);
						
						//MOVE EXTRUDERS
						current_position[Z_AXIS] = 60;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS]*2/60, LEFT_EXTRUDER);//move bed
						st_synchronize();
						current_position[X_AXIS] = 155; current_position[Y_AXIS] = 0;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS]/3, LEFT_EXTRUDER);//move first extruder
						
						dobloking=true;
						
						while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) && degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5)){ //Waiting to heat the extruder
							
							manage_heater();
						}
						
						genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,1);
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,1);
						genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Clean the left nozzle \n and press GO to move on to \n the next EXTRUDER");
						flag_continue_calib = true;
					}
					else{
						genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
					}
				}				
				
				
				#pragma endregion Info Screens
				
				
				
				}// else
			}
			//Userbuttons
				//USERBUTTONS------------------------------------------------------

				
			//FORMS--------------------------------------------------------
			if (Event.reportObject.object == GENIE_OBJ_FORM)
			{
				if (Event.reportObject.index == FORM_SDFILES)
				{
					//wake_RELAY();
					sdscreenblocked = true;
					Serial.println("Form 2!");
					////Check sdcardFiles
					filepointer = 0;
					card.initsd();
					if(card.cardOK){
						
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
							
							int line = 23;
							int count = 63;
							char buffer[count+3];
							int x = 0;
							memset( buffer, '\0', sizeof(char)*count );
							
							if (String(card.longFilename).length() > count){
								for (int i = 0; i<count ; i++)
								{
									if (card.longFilename[i] == '.') i = count +10; //go out of the for
									else if(i == 0) buffer[i]=card.longFilename[x];
									else if (i%line == 0){
										buffer[i] = '\n';
										i++;
										buffer[i]=card.longFilename[x];
									}
									else {
										buffer[i]=card.longFilename[x];
									}
									x++;
									Serial.print(i);
								}
								buffer[count]='\0';
								char* buffer2 = strcat(buffer,"...\0");
								genie.WriteStr(STRING_NAME_FILE,buffer2);//Printing form
							}
							else {
								for (int i = 0; i<String(card.longFilename).length(); i++)	{
									if (card.longFilename[i] == '.') i = String(card.longFilename).length() +10; //go out of the for
									else if(i == 0) buffer[i]=card.longFilename[x];
									else if (i%line == 0){
										buffer[i] = '\n';
										i++;
										buffer[i]=card.longFilename[x];
									}
									else {
										buffer[i]=card.longFilename[x];
									}
									x++;
									Serial.print(i);
								}
								//buffer[count]='\0';
								genie.WriteStr(STRING_NAME_FILE,buffer);//Printing form
								//Is a file
								//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
							}
							Serial.println(buffer);
						}
					}
					
					else{
						
						genie.WriteStr(STRING_NAME_FILE,"\n                Insert SD Card");//Printing form
						screen_sdcard = true;
						
					}
					sdscreenblocked=false;
					
				}
			
				else if (Event.reportObject.index == FORM_PRINTING)
				{
					//Restart the preheat buttons
					/*genie.WriteObject(GENIE_OBJ_USERIMAGES,BUTTON_PREHEAT_PLA,0);
					
					#ifdef SIGMA_TOUCH_SCREEN
					genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING,0);
					char buffer[13];
					if (String(card.longFilename).length()>12){
						for (int i = 0; i<12 ; i++)
						{
							buffer[i]=card.longFilename[i];
						}
						buffer[12]='\0';
						char* buffer2 = strcat(buffer,"...\0");
						Serial.print("Card Name: ");
						Serial.println(card.longFilename);
						Serial.print("Buffer1: ");
						Serial.println(buffer);
						Serial.print("buffer out: ");
						Serial.println(buffer2);
						genie.WriteStr(STRINGS_PRINTING_GCODE,buffer2);//Printing form
					}else{
						for (int i = 0; i<=String(card.longFilename).length(); i++)
						{
							if (buffer[i] == '.') i = String(card.longFilename).length() +10;
							else buffer[i]=card.longFilename[i];
						}
						//buffer[count]='\0';
						genie.WriteStr(STRINGS_PRINTING_GCODE,buffer);//Printing form//Printing form
					}
					
					//Serial.println((char*)prepareString(card.longFilename,12));
					//genie.WriteStr(6,"Ready");
					#endif*/
					/*enquecommand_P(PSTR("M24"));
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
							else *//*buffer[i]=card.longFilename[i];
						}
						buffer[count]='\0';
						genie.WriteStr(STRINGS_PRINTING_GCODE,buffer);//Printing form//Printing form
					}*/
				
				//genie.WriteStr(2,card.longFilename);
				//genie.WriteStr(6,"Printing...");
				is_on_printing_screen = true;
				
				}
				else if (Event.reportObject.index == FORM_MAIN_SCREEN)
				{
					screen_sdcard = false;
					surfing_utilities=false;
					Serial.println("Surfing 0");
				}
				
				else if (Event.reportObject.index == FORM_UTILITIES)
				{
				surfing_utilities=true;
				Serial.println("Surfing 1");
				}
				
				else if (Event.reportObject.index == FORM_PURGE)
				{
					Serial.println("Enter in purge mode");
					if(purge_extruder_selected == 0) {
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
					}
					else {
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,1);			
					}		
					
					char buffer[256];
					sprintf(buffer, "%3d %cC",int(degHotend(0)),0x00B0);
					genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);	Serial.println(buffer);
					sprintf(buffer, "%3d %cC",int(degHotend(1)),0x00B0);
					genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);	Serial.println(buffer);	
				}				
				else if (Event.reportObject.index == FORM_PREHEAT_SETTINGS)
				{
				//Rapduch
				//First send the actual command
				/*setTargetHotend0(print_temp_l);
				setTargetHotend1(print_temp_r);
				setTargetBed(max(bed_temp_l,bed_temp_r));
				
				//Now let's print it on the touchscreen
				char buffer[256];
				int tHotend=target_temperature[0];
				int tHotend1=target_temperature[1];
				int tBed=target_temperature_bed;
				
				//Serial.println("TARGET TEMPS");
				
				sprintf(buffer, "%3d%cC",tHotend,0x00B0);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PREHEAT_SET_NOZZ1,buffer);
				
				sprintf(buffer, "%3d%cC",tHotend1,0x00B0);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PREHEAT_SET_NOZZ2,buffer);
				
				sprintf(buffer, "%3d%cC",tBed,0x00B0);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PREHEAT_SET_BED,buffer);*/
				/*setTargetHotend0(print_temp_l);
				setTargetHotend1(print_temp_r);
				setTargetBed(max(bed_temp_l,bed_temp_r));*/
				
				//Now let's print it on the touchscreen
				char buffer[256];
				int tHotend=preheat_E0_value;
				int tHotend1=preheat_E1_value;
				int tBed=preheat_B_value;
				
				//Serial.println("TARGET TEMPS");
				
				sprintf(buffer, "%3d%cC",tHotend,0x00B0);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PREHEAT_SET_NOZZ1,buffer);
				
				sprintf(buffer, "%3d%cC",tHotend1,0x00B0);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PREHEAT_SET_NOZZ2,buffer);
				
				sprintf(buffer, "%3d%cC",tBed,0x00B0);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PREHEAT_SET_BED,buffer);
				
				}
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
void insertmetod(){
	processing = true;
					genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
					if (!home_made) home_axis_from_code(true,true,true);
					
					int feedrate;
					if (!flag_filament_home){
						//MOVING THE EXTRUDERS TO AVOID HITTING THE CASE WHEN PROBING-------------------------
						//current_position[X_AXIS]+=25;
						
						home_axis_from_code(true,true,false);						
						/*current_position[X_AXIS] = x_home_pos(LEFT_EXTRUDER)+25;
						feedrate=homing_feedrate[X_AXIS];
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, LEFT_EXTRUDER);
						st_synchronize();
						//current_position[X_AXIS] = x_home_pos(RIGHT_EXTRUDER);
					
						current_position[X_AXIS]=extruder_offset[X_AXIS][1];
						Serial.println(current_position[X_AXIS]);
						plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
						//current_position[X_AXIS]-=25;
						current_position[X_AXIS] = x_home_pos(RIGHT_EXTRUDER)-25;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
						st_synchronize();*/
						flag_filament_home=true;
					}
					
					/*current_position[Y_AXIS]=285;
					feedrate=homing_feedrate[Y_AXIS];
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					st_synchronize();*/
					
					current_position[Z_AXIS]=Z_MAX_POS-15;
					feedrate=homing_feedrate[Z_AXIS];
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate*2/60, active_extruder); //check speed
					st_synchronize();					
					/****************************************************/
										
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INSERT_FILAMENT_TOP,0);
					processing = false;
}


	#endif /* INCLUDE */

#endif