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
#include "SD_ListFiles.h"

//#include "ultralcd.h"
inline void setfilenames(int jint);
inline void insertmetod();
inline void ListFilesParsingProcedure(int vecto, int jint);
inline void ListFilesUpfunc();
inline void ListFilesDownfunc();
inline void ListFileListINITSD();
inline void ListFileListENTERBACKFORLDERSD();
inline void ListFileSelect_find();
inline void ListFileSelect0();
inline void ListFileSelect1();
inline void ListFileSelect2();
inline void ListFileSelect3();
inline void ListFileSelect4();
inline void ListFileSelect5();
inline void setfoldernames(int jint);
inline void folder_navigation_register(bool upchdir);
void myGenieEventHandler();
bool FLAG_NylonCleanMetode = false;
bool FLAG_PrintSettingRefresh = false;
bool FLAG_PrintSettingBack = false;
bool FLAG_PIDautotune = false;
bool FLAG_FilamentHome = false;
bool FLAG_FilamentAcceptOk = false;
bool FLAG_PausePause = false;
bool FLAG_GifHotent0 = false;
bool FLAG_GifHotent1 = false;
bool FLAG_GifBed = false;
bool FLAG_PrintPrintStop = false;
bool FLAG_PrintPrintSave = false;
bool FLAG_PrintPrintPause = false;
bool FLAG_PrintPrintResume = false;
bool FLAG_PauseResume = false;
bool FLAG_CalibFull = false;
bool FLAG_CalibBedDone = false;
bool FLAG_FilesUpDown = true;
bool FLAG_ListFilesSelect0 = false;
bool FLAG_ListFilesSelect1 = false;
bool FLAG_ListFilesSelect2 = false;
bool FLAG_ListFilesSelect3 = false;
bool FLAG_ListFilesSelect4 = false;
bool FLAG_ListFilesSelect5 = false;
bool FLAG_ListFilesDown = false;
bool FLAG_ListFilesUp = false;
bool FLAG_ListFilesInit = false;
bool FLAG_ListFileEnterBackFolder = false;
bool FLAG_ZAdjust50Up = false;
bool FLAG_ZAdjust10Up = false;
bool FLAG_ZAdjust50Down = false;
bool FLAG_ZAdjust10Down = false;
bool FLAG_DataRefresh =  false;
bool FLAG_PurgeSelect0 = false;//purge
bool FLAG_PurgeSelect1 = false;//Retract
bool FLAG_LoadSelect0 = false;//purge
bool FLAG_UnloadSelect1 = false;//Retract
bool FLAG_SavePrintCommand = false;
bool busy_button = false;
int Temp_ChangeFilament_Saved = 0;
int Tref1 = 0;
int Tfinal1 = 0;
int  print_setting_tool = 2;
float offset_calib_manu[4] = {0.0,0.0,0.0,0.0};
unsigned int calib_value_selected;
float offset_x_calib = 0;
float offset_y_calib = 0;
int  previous_state = FORM_MAIN_SCREEN;
int custom_insert_temp = 210;
int custom_remove_temp = 210;
int custom_print_temp = 210;
int custom_bed_temp = 40;
unsigned int buttonsdselected[6] = {BUTTON_SD_SELECTED0, BUTTON_SD_SELECTED1, BUTTON_SD_SELECTED2, BUTTON_SD_SELECTED3, BUTTON_SD_SELECTED4, BUTTON_SD_SELECTED5};
unsigned int stringfilename[8] = {STRING_NAME_FILE0, STRING_NAME_FILE1, STRING_NAME_FILE2, STRING_NAME_FILE3, STRING_NAME_FILE4, STRING_NAME_FILE5,STRING_NAME_FILE6,STRING_RECOVERY_PRINT_ASK};
unsigned int stringfiledur[8] = {STRING_NAME_FILE_DUR0, STRING_NAME_FILE_DUR1, STRING_NAME_FILE_DUR2, STRING_NAME_FILE_DUR3, STRING_NAME_FILE_DUR4,STRING_NAME_FILE_DUR5, STRING_NAME_FILE_DUR6, STRING_RECOVERY_PRINT_ASK_DUR};


int redo_source;


//Created by Jordi Calduch for RepRapBCN SIGMA 12/2014
void myGenieEventHandler(void) //Handler for the do.Events() function
{
	genieFrame Event;
	genie.DequeueEvent(&Event);
	//static long waitPeriod = millis();
	static long waitPeriod_s = millis();
	static long waitPeriod_button_press = millis(); // This waitperiod avoid some spamming pressing on purging buttons which can block the machine
	int move_mm = 10;
	//If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
	if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
	{
		
		if (Event.reportObject.object == GENIE_OBJ_USERBUTTON) //Userbuttons to select GCODE from SD
		{
			if (card.sdprinting || card.sdispaused){
				
				//******PRINTING****
				#pragma region Printing_screen
				
				if (Event.reportObject.index == BUTTON_PRINT_SETTINGS )
				{
					if (millis() >= waitPeriod_button_press){
						
						FLAG_PrintSettingRefresh = true;
						
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					
				}
				else if (Event.reportObject.index == BUTTON_PRINT_SET_BACK)
				{
					if (millis() >= waitPeriod_button_press){
						
						FLAG_PrintSettingBack = true;
						
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				
				
				else if (Event.reportObject.index == BUTTON_PRINT_SETTINGS_PAUSE )
				{
					if (millis() >= waitPeriod_button_press){
						
						FLAG_PrintSettingRefresh = true;
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PRINTING_BACK_STATE_PAUSE, 1);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				
				
				else if (Event.reportObject.index == BUTTON_PRINTING_BACK_STATE_PAUSE )
				{
					if (millis() >= waitPeriod_button_press){
						
						if(screen_printing_pause_form ==screen_printing_pause_form2){
							screen_printing_pause_form = screen_printing_pause_form1;
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_STOP_SCREEN_PAUSE, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PAUSE_RESUME_PAUSE, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PRINT_SETTINGS_PAUSE, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PRINTING_BACK_STATE_PAUSE, 0);
							surfing_utilities = false;
						}
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				
				
				
				else if (Event.reportObject.index == BUTTON_STOP_YES )
				{
					if (millis() >= waitPeriod_button_press){
						
						is_on_printing_screen=false;
						
						card.closefile();
						FLAG_PrintPrintStop = true;
						cancel_heatup = true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						processing = true;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				else if (Event.reportObject.index == BUTTON_STOP_NO )
				{
					if (millis() >= waitPeriod_button_press){
						
						if(screen_printing_pause_form == screen_printing_pause_form0){
							
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING,0);
							is_on_printing_screen = true;
							genie.WriteStr(STRINGS_PRINTING_GCODE,namefilegcode);
							FLAG_DataRefresh = true;
							surfing_utilities = false;
							}else{
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING_PAUSE,0);
							is_on_printing_screen = true;
							genie.WriteStr(STRINGS_PRINTING_GCODE_PAUSE,namefilegcode);
							FLAG_DataRefresh = true;
							surfing_utilities = false;
						}
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				else if (Event.reportObject.index == BUTTON_STOP_SAVE && !waiting_temps && !card.sdispaused)
				{
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_SAVEPRINT_SURE_ASK,0);
						
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				else if (Event.reportObject.index == BUTTON_SAVEPRINT_SURE_ASK_NOT && !waiting_temps)
				{
					if (millis() >= waitPeriod_button_press){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_STOP_PRINT,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				else if (Event.reportObject.index == BUTTON_SAVEPRINT_SURE_ASK_OK && !waiting_temps)
				{
					if (millis() >= waitPeriod_button_press){
						
						
						if(screen_printing_pause_form == screen_printing_pause_form0){
							
							
							is_on_printing_screen=false;
							
							card.sdprinting = false;
							card.sdispaused = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							processing = true;
							FLAG_SavePrintCommand = true;
							
						}
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				else if (Event.reportObject.index == BUTTON_STOP_SCREEN && (screen_printing_pause_form == screen_printing_pause_form0))
				{
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_STOP_PRINT,0);
						
						is_on_printing_screen=false;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				else if (Event.reportObject.index == BUTTON_STOP_SCREEN_PAUSE && (screen_printing_pause_form == screen_printing_pause_form1))
				{
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_STOP_PRINT,0);
						
						is_on_printing_screen=false;
						
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				else if (Event.reportObject.index == BUTTON_STOP_SCREEN_PAUSE && (screen_printing_pause_form == screen_printing_pause_form2))
				{
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_REMOVE_FIL,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_REMOVE_MENU_FILAMENT,1);
						filament_mode = 'R';
						surfing_utilities = true;
						is_on_printing_screen=false;
						
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				
				else if (Event.reportObject.index == BUTTON_PAUSE_RESUME && card.sdprinting && screen_printing_pause_form == screen_printing_pause_form0)
				{
					if (millis() >= waitPeriod_button_press){
						
						FLAG_PrintPrintPause = true;
						
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					
				}
				//We need to Resume/Enter Printing Settings/Stop printing
				else if (Event.reportObject.index == BUTTON_PAUSE_RESUME_PAUSE && card.sdispaused && screen_printing_pause_form == screen_printing_pause_form1)
				{
					if (millis() >= waitPeriod_button_press){
						
						FLAG_PrintPrintResume = true;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				else if (Event.reportObject.index == BUTTON_PAUSE_RESUME_PAUSE && card.sdispaused && screen_printing_pause_form == screen_printing_pause_form2)
				{
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_INSERT,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_Retract,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_TEMP_UP,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_TEMP_DOWN,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_MENU,1);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PURGE,0);
						surfing_utilities = true;
						purge_extruder_selected = -1;
						SERIAL_PROTOCOLPGM("Enter in purge mode \n");
						/*setTargetHotend0(print_temp_l);
						setTargetHotend1(print_temp_r);*/
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
						/*if(purge_extruder_selected == 0) {
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
						
						}
						else {
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,1);
						}*/
						
						char buffer[256];
						sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
						genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);	Serial.println(buffer);
						sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
						genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);	Serial.println(buffer);
						sprintf_P(buffer, PSTR("%3d %cC"),0,0x00B0);
						genie.WriteStr(STRING_PURGE_SELECTED,buffer);	Serial.println(buffer);
						
						is_on_printing_screen = false;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				#pragma endregion Printing_screen
				
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
				
				
				
				
				
				#pragma region Insert_Remove_Fil
				/*else if (Event.reportObject.index == BUTTON_UTILITIES_PRINT_PURGE)
				{
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
				sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
				genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);	Serial.println(buffer);
				sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
				genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);	Serial.println(buffer);
				sprintf_P(buffer, PSTR("%3d %cC"),int(target_temperature[purge_extruder_selected]),0x00B0);
				genie.WriteStr(STRING_PURGE_SELECTED,buffer);	Serial.println(buffer);
				
				}*/
				
				else if ((Event.reportObject.index == BUTTON_REMOVE_BACK_FILAMENT ))
				{
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING_PAUSE,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_REMOVE_MENU_FILAMENT,0);
						is_on_printing_screen = true;
						surfing_utilities = false;
						genie.WriteStr(STRINGS_PRINTING_GCODE_PAUSE,namefilegcode);
						FLAG_DataRefresh = true;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				
				
				
				else if (Event.reportObject.index == BUTTON_INSERT_FIL || Event.reportObject.index == BUTTON_REMOVE_FIL )
				{
					if (millis() >= waitPeriod_button_press){
						
						if (Event.reportObject.index == BUTTON_INSERT_FIL) filament_mode = 'I'; //Insert Mode
						else if (Event.reportObject.index == BUTTON_REMOVE_FIL) filament_mode = 'R'; //Remove Mode
						
						/*if (!FLAG_FilamentHome){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						
						}*/
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 0);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_SELECT_EXTRUDER,0);
						which_extruder = -1;
						//genie.WriteObject(GENIE_OBJ_IMAGE, IMAG_MATERIALS, -1);
						
						/*genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PLA, -1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_ABS, -1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PVA, -1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUST, -1);*/
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				
				
				//else if (Event.reportObject.index == BUTTON_FILAMENT_NOZZLE1 || Event.reportObject.index == BUTTON_FILAMENT_NOZZLE2)
				//{
				//if (Event.reportObject.index == BUTTON_FILAMENT_NOZZLE1) //Left Nozzle
				//{
				//
				//which_extruder=0;
				//
				//}
				//else //Right Nozzle
				//{
				//
				//which_extruder=1;
				//}
				//if (filament_mode == 'I') {
				//if (which_extruder == 0){
				//genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 1);
				//genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 0);
				//genie.WriteObject(GENIE_OBJ_IMAGE, IMAG_MATERIALS, 0);
				///*genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PLA, 0);
				//genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_ABS, 0);
				//genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PVA, 0);
				//genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUST, 0);*/
				//}
				//else{
				//genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 1);
				//genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 0);
				//genie.WriteObject(GENIE_OBJ_IMAGE, IMAG_MATERIALS, 0);
				///*genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PLA, 0);
				//genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_ABS, 0);
				//genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PVA, 0);
				//genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUST, 0);*/
				//}
				//}
				//
				//
				//
				//else {
				////*********Move the bed down and the extruders inside
				//if(which_extruder == 0) setTargetHotend(max(print_temp_r,old_print_temp_l),which_extruder);
				//else setTargetHotend(max(print_temp_r,old_print_temp_r),which_extruder);
				//processing = true;
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
				//
				//
				//current_position[Z_AXIS]=Z_MAX_POS-15;
				//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder); //check speed
				//
				//current_position[Y_AXIS]=10;
				//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Y_AXIS], active_extruder); //check speed
				//st_synchronize();
				//
				//
				//processing = false;
				//
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
				//
				//
				//processing_adjusting = true;
				//
				//is_changing_filament=true;
				//
				//}
				//}
				
				else if (Event.reportObject.index == BUTTON_REMOVE_SELECT_LEFT || Event.reportObject.index == BUTTON_REMOVE_SELECT_RIGHT)
				{
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						if (Event.reportObject.index == BUTTON_REMOVE_SELECT_LEFT) //Left Nozzle
						{
							
							which_extruder=0;
							
						}
						else //Right Nozzle
						{
							
							which_extruder=1;
						}
						Temp_ChangeFilament_Saved = target_temperature[which_extruder];
						if(which_extruder == 0) setTargetHotend(max(remove_temp_l,old_remove_temp_l),which_extruder);
						else setTargetHotend(max(remove_temp_r,old_remove_temp_r),which_extruder);
						processing = true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_REMOVE_MENU_FILAMENT,0);
						/*if (home_made_Z){
						home_axis_from_code(true,true,false);
						}
						else{
						home_axis_from_code(true,true,true);
						}*/
						
						current_position[Z_AXIS]=Z_MAX_POS-15;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder); //check speed
						
						current_position[Y_AXIS]=10;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Y_AXIS], active_extruder); //check speed
						st_synchronize();
						if(processing_error)return;
						
						processing = false;
						genie.WriteStr(STRING_CHANGE_FILAMENT_TEMPS,"0%");
						genie.WriteObject(GENIE_OBJ_FORM,FORM_CHANGE_FILAMENT_TEMPS,0);
						Tref1 = (int)degHotend(which_extruder);
						Tfinal1 = (int)degTargetHotend(which_extruder);
						
						
						processing_change_filament_temps = true;
						
						is_changing_filament=true; //We are changing filament
						
					}
				}

				else if (Event.reportObject.index == BUTTON_PLA){
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						if (which_extruder == 1) // Need to pause
						{
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							print_temp_r = PLA_PRINT_TEMP;
							insert_temp_r = PLA_INSERT_TEMP;
							remove_temp_r = PLA_REMOVE_TEMP;
							bed_temp_r = PLA_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend1(print_temp_r);
							insertmetod();
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SELECT_EXTRUDER_MENU,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FILAMENT_BACK,0);
							
						}
						else if(which_extruder == 0){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							print_temp_l = PLA_PRINT_TEMP;
							insert_temp_l = PLA_INSERT_TEMP;
							remove_temp_l = PLA_REMOVE_TEMP;
							bed_temp_l = PLA_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend0(print_temp_l);
							insertmetod();
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SELECT_EXTRUDER_MENU,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FILAMENT_BACK,0);
						}
					}
				}
				
				else if (Event.reportObject.index == BUTTON_ABS){
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						if (which_extruder == 1) // Need to pause
						{
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							
							
							print_temp_r = ABS_PRINT_TEMP;
							insert_temp_r = ABS_INSERT_TEMP;
							remove_temp_r = ABS_REMOVE_TEMP;
							bed_temp_r = ABS_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend1(print_temp_r);
							insertmetod();
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SELECT_EXTRUDER_MENU,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FILAMENT_BACK,0);
						}
						else if(which_extruder == 0){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							
							print_temp_l = ABS_PRINT_TEMP;
							insert_temp_l = ABS_INSERT_TEMP;
							remove_temp_l = ABS_REMOVE_TEMP;
							bed_temp_l = ABS_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend0(print_temp_l);
							insertmetod();
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SELECT_EXTRUDER_MENU,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FILAMENT_BACK,0);
						}
					}
					
				}
				else if (Event.reportObject.index == BUTTON_PVA){
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						if (which_extruder == 1) // Need to pause
						{
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							
							
							print_temp_r = PVA_PRINT_TEMP;
							insert_temp_r = PVA_INSERT_TEMP;
							remove_temp_r = PVA_REMOVE_TEMP;
							bed_temp_r = PVA_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend1(print_temp_r);
							insertmetod();
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SELECT_EXTRUDER_MENU,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FILAMENT_BACK,0);
						}
						else if(which_extruder == 0){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							
							print_temp_l = PVA_PRINT_TEMP;
							insert_temp_l = PVA_INSERT_TEMP;
							remove_temp_l = PVA_REMOVE_TEMP;
							bed_temp_l = PVA_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend0(print_temp_l);
							insertmetod();
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SELECT_EXTRUDER_MENU,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FILAMENT_BACK,0);
						}
					}
				}
				
				//CUSTOM MATERIAL BUTTONS
				else if(Event.reportObject.index == BUTTON_CUST){
					if (millis() >= waitPeriod_button_press){
						
						if (which_extruder == 1 || which_extruder == 0) // Need to pause
						{
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
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
				}
				else if (Event.reportObject.index == BUTTON_CUSTOM_BACK){
					if (millis() >= waitPeriod_button_press){
						
						
						/*genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 0);*/
						genie.WriteObject(GENIE_OBJ_FORM,FORM_SELECT_EXTRUDER,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						//which_extruder = -1;
					}
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
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
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
						insertmetod();
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SELECT_EXTRUDER_MENU,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FILAMENT_BACK,0);
					}
				}
				//////////////////////////
				

				else if(Event.reportObject.index == BUTTON_MOVE_INSERT){
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						//ATTENTION : Order here is important
						genie.WriteStr(STRING_CHANGE_FILAMENT_TEMPS,"0%");
						genie.WriteObject(GENIE_OBJ_FORM,FORM_CHANGE_FILAMENT_TEMPS,0);
						//genie.WriteStr(STRING_ADVISE_FILAMENT,"");
						//genie.WriteStr(STRING_ADVISE_FILAMENT,"Insert the filament until you feel it stops, \n then while you keep inserting around \n 10 mm of filament, press the clip");
						processing_change_filament_temps = true;
						
						if (which_extruder==0) setTargetHotend(max(insert_temp_l,old_insert_temp_l),which_extruder);
						else setTargetHotend(max(insert_temp_r,old_insert_temp_r),which_extruder);
						//delay(3500);
						is_changing_filament=true;
						
						/*if (which_extruder == 0) changeTool(0);
						else changeTool(1);
						
						current_position[Y_AXIS] = 100;
						current_position[X_AXIS] = 155;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
						st_synchronize();*/
						
						
						Tref1 = (int)degHotend(which_extruder);
						Tfinal1 = (int)degTargetHotend(which_extruder);
					}
				}
				
				
				else if (Event.reportObject.index == BUTTON_INSERT ){
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						if (filament_mode =='I')
						{ //Inserting...
							processing = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							//delay(850);
							SERIAL_PROTOCOLPGM("Inserting :   \n");
							current_position[E_AXIS] += 30;//Extra extrusion at low feedrate
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  700/60, which_extruder); //850/60
							st_synchronize();
							if(processing_error)return;
							current_position[E_AXIS] += ((BOWDEN_LENGTH-EXTRUDER_LENGTH)-15);//BOWDEN_LENGTH-300+340);
							Serial.println(current_position[E_AXIS]);
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, which_extruder);
							st_synchronize();
							if(processing_error)return;
							current_position[E_AXIS] += EXTRUDER_LENGTH;//Extra extrusion at low feedrate
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_SLOW_SPEED/60, which_extruder);
							st_synchronize();
							if(processing_error)return;
							processing = false;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUST_FILAMENT,0);
						}
					}
				}
				
				
				else if (Event.reportObject.index == BUTTON_REMOVE )
				{// We should have already checked if filament is inserted
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						
						if (filament_mode =='R')
						{ //Removing...
							processing = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							current_position[E_AXIS] -= (BOWDEN_LENGTH + EXTRUDER_LENGTH + 100);//Extra extrusion at fast feedrate
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_FAST_SPEED/60, which_extruder);
							st_synchronize();
							if(processing_error)return;
							previous_state = FORM_FILAMENT;
							
							processing = false;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_SUCCESS_FILAMENT,0);
							processing_success = true;
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
						}
					}
				}
				#pragma endregion Insert_Remove_Fil
				
				#pragma region AdjustFilament
				else if (Event.reportObject.index == BUTTON_ACCEPT_ADJUST && FLAG_FilamentAcceptOk == false)
				{
					
					if (millis() >= waitPeriod_button_press){
						/*genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						FLAG_FilamentAcceptOk = true;
						home_made = false;
						processing=true;
						home_axis_from_code(true,true,false);*/
						setTargetHotend((float)Temp_ChangeFilament_Saved, which_extruder);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_SUCCESS_FILAMENT,0);
						processing_success = true;
						
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
					
					else if (Event.reportObject.index == BUTTON_ADJUST_Load  && FLAG_FilamentAcceptOk == false)
					{
						if (millis() >= waitPeriod_button_press){
							//Adjusting the filament with a Retract Up
							
							float modified_position=current_position[E_AXIS]+6;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], modified_position, INSERT_SLOW_SPEED/60, which_extruder);
							current_position[E_AXIS]=modified_position;
							
							waitPeriod_button_press=millis()+3000;
						}
						
					}
					
					else if (Event.reportObject.index == BUTTON_ADJUST_Unload  && FLAG_FilamentAcceptOk == false)
					{
						//Adjusting the filament with a purge Down
						if (millis() >= waitPeriod_button_press){
							
							float modified_position=current_position[E_AXIS]-6;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], modified_position, INSERT_SLOW_SPEED/60, which_extruder);
							current_position[E_AXIS]=modified_position;
							waitPeriod_button_press=millis()+3000;
						}
					}
					#pragma endregion AdjustFilament
					#pragma region PURGEpause
					
					
					
					//****************PURGE BUTTONS******
					else if (Event.reportObject.index == BUTTON_PURGE_LEFT  && !blocks_queued()){
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_INSERT,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_Retract,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_TEMP_UP,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_TEMP_DOWN,1);
						if (purge_extruder_selected == 1){
							purge_extruder_selected = 0;
							if(target_temperature[0] == 0){
								setTargetHotend0(print_temp_l);
							}
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
							char buffer[256];
							sprintf(buffer, "%d %cC",target_temperature[0],0x00B0);
							genie.WriteStr(STRING_PURGE_SELECTED,buffer);
							Serial.println(buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
							genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);
						}
						else{
							char buffer[256];
							purge_extruder_selected = 0;
							if(target_temperature[0] == 0){
								setTargetHotend0(print_temp_l);
							}
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
							sprintf_P(buffer, PSTR("%3d %cC"),target_temperature[0],0x00B0);
							genie.WriteStr(STRING_PURGE_SELECTED,buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
							genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);
						}
					}
					else if (Event.reportObject.index == BUTTON_PURGE_RIGHT  && !blocks_queued()){
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_INSERT,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_Retract,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_TEMP_UP,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_TEMP_DOWN,1);
						if (purge_extruder_selected == 0){
							purge_extruder_selected = 1;
							if(target_temperature[1] == 0){
								setTargetHotend1(print_temp_r);
							}
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,1);
							char buffer[256];
							sprintf_P(buffer, PSTR("%3d %cC"),target_temperature[1],0x00B0);
							genie.WriteStr(STRING_PURGE_SELECTED,buffer);
							Serial.println(buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
							genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);
						}
						else{
							purge_extruder_selected = 1;
							if(target_temperature[1] == 0){
								setTargetHotend1(print_temp_r);
							}
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,1);
							char buffer[256];
							sprintf_P(buffer, PSTR("%3d %cC"),target_temperature[1],0x00B0);
							genie.WriteStr(STRING_PURGE_SELECTED,buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
							genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);
						}
					}
					else if(Event.reportObject.index == BUTTON_PURGE_TEMP_UP && purge_extruder_selected != -1){
						
						if (target_temperature[purge_extruder_selected] < HEATER_0_MAXTEMP){
							target_temperature[purge_extruder_selected] += 5;
							setTargetHotend0(target_temperature[0]);
							setTargetHotend1(target_temperature[1]);
							char buffer[256];
							sprintf_P(buffer, PSTR("%3d %cC"),int(target_temperature[purge_extruder_selected]),0x00B0);
							genie.WriteStr(STRING_PURGE_SELECTED,buffer);
						}
					}
					else if(Event.reportObject.index == BUTTON_PURGE_TEMP_DOWN && purge_extruder_selected != -1){
						if (target_temperature[purge_extruder_selected] > 0){
							target_temperature[purge_extruder_selected] -= 5;
							setTargetHotend0(target_temperature[0]);
							setTargetHotend1(target_temperature[1]);
							char buffer[256];
							sprintf_P(buffer, PSTR("%3d %cC"),int(target_temperature[purge_extruder_selected]),0x00B0);
							genie.WriteStr(STRING_PURGE_SELECTED,buffer);
						}
					}
					//***MOVING
					else if(Event.reportObject.index == BUTTON_PURGE_Retract && purge_extruder_selected != -1 && !blocks_queued()){
						
						if(degHotend(purge_extruder_selected) >= target_temperature[purge_extruder_selected]-PURGE_TEMP_HYSTERESIS){
							processing_purge_load = true;
							current_position[E_AXIS]-=5;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60, purge_extruder_selected);//Retract
							st_synchronize();
							if(processing_error)return;
							processing_purge_load = false;
							genie.WriteObject(GENIE_OBJ_VIDEO,GIF_PURGE_LOAD,0);
						}
					}
					else if(Event.reportObject.index == BUTTON_PURGE_INSERT && purge_extruder_selected != -1 && !blocks_queued()){
						
						if(degHotend(purge_extruder_selected) >= target_temperature[purge_extruder_selected]-PURGE_TEMP_HYSTERESIS){
							processing_purge_load = true;
							current_position[E_AXIS]+=15;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60, purge_extruder_selected);//Purge
							st_synchronize();
							if(processing_error)return;
							processing_purge_load = false;
							genie.WriteObject(GENIE_OBJ_VIDEO,GIF_PURGE_LOAD,0);
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
					
					
					else if(Event.reportObject.index	== BUTTON_PURGE_BACK  && !blocks_queued()){
						//quickStop();
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING_PAUSE,0);
							is_on_printing_screen = true;
							surfing_utilities = false;
							genie.WriteStr(STRINGS_PRINTING_GCODE_PAUSE,namefilegcode);
							FLAG_DataRefresh = true;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						/*
						current_position[E_AXIS] = saved_position[E_AXIS]-2;
						plan_set_e_position(current_position[E_AXIS]);*/
						//setTargetHotend0(0);
						//setTargetHotend1(0);
						
					}
					
					//************************************
					
					
					
					
					#pragma endregion PURGEpause
					
					#pragma endregion Printing_UTILITIES
					
					#pragma region SuccessScreensPrint
					else if (Event.reportObject.index == BUTTON_SUCCESS_FILAMENT_OK)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							
							//enquecommand_P((PSTR("G28 X0 Y0")));
							SERIAL_PROTOCOLPGM("SUCCESS \n");
							processing_success = false;
							FLAG_FilamentAcceptOk = false;
							if (filament_mode == 'R')
							{
								
								filament_mode = 'I';
								if(which_extruder ==  1){
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 0);
								}
								else{
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 0);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 1);
								}
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SELECT_EXTRUDER_MENU,1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FILAMENT_BACK,1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PLA, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_ABS, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PVA, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUST, 1);
								
								genie.WriteObject(GENIE_OBJ_FORM,FORM_SELECT_EXTRUDER,0);
								//which_extruder = -1;
							}
							else if (filament_mode == 'I')
							{
								
								processing = true;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								current_position[Y_AXIS] = saved_position[Y_AXIS];
								plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Y_AXIS]/60, active_extruder);//Purge
								st_synchronize();
								if(processing_error)return;
								
								//home_axis_from_code(true, true, false);
								
								current_position[Z_AXIS] = saved_position[Z_AXIS] + 10;
								plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],homing_feedrate[Z_AXIS] ,saved_active_extruder);
								st_synchronize();
								if(processing_error)return;
								processing = false;
								
								
								
								
								
								genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING_PAUSE,0);
								is_on_printing_screen = true;
								surfing_utilities = false;
								genie.WriteStr(STRINGS_PRINTING_GCODE_PAUSE,namefilegcode);
								FLAG_DataRefresh = true;
							}
							//doblocking =true;
						}
						
						
					}
					#pragma endregion SuccessScreensPrin
					else if (Event.reportObject.index == BUTTON_ERROR_OK)
					{
						if (millis() >= waitPeriod_button_press){
							
							if(FLAG_thermal_runaway_screen){
								processing_error = false;
								FLAG_thermal_runaway_screen = false;
								FLAG_PrintSettingBack = true;
								if(screen_printing_pause_form == screen_printing_pause_form0){
									genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING,0);
									}else{
									genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING_PAUSE,0);
								}
								
								}else{
								processing_error = false;
								screen_sdcard = false;
								surfing_utilities=false;
								SERIAL_PROTOCOLPGM("Surfing 0 \n");
								surfing_temps = false;
								HeaterCooldownInactivity(true);
								genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN_SCREEN, 0);
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					}else{//All that has to be done out of the printing room
					
					
					
					
					
					
					//*****SD Gcode Selection*****
					#pragma region SD Gcode Selector
					
					if ((Event.reportObject.index == BUTTON_SD_SELECTED0) && FLAG_FilesUpDown)
					{
						FLAG_ListFilesSelect0 = true;
						SERIAL_PROTOCOLLNPGM("Select 0");
					}
					else if ((Event.reportObject.index == BUTTON_SD_SELECTED1) && FLAG_FilesUpDown)
					{
						FLAG_ListFilesSelect1 = true;
						SERIAL_PROTOCOLLNPGM("Select 1");
					}
					else if ((Event.reportObject.index == BUTTON_SD_SELECTED2) && FLAG_FilesUpDown)
					{
						FLAG_ListFilesSelect2 = true;
						SERIAL_PROTOCOLLNPGM("Select 2");
					}
					else if ((Event.reportObject.index == BUTTON_SD_SELECTED3) && FLAG_FilesUpDown)
					{
						FLAG_ListFilesSelect3 = true;
						SERIAL_PROTOCOLLNPGM("Select 3");
					}
					else if ((Event.reportObject.index == BUTTON_SD_SELECTED4) && FLAG_FilesUpDown)
					{
						FLAG_ListFilesSelect4 = true;
						SERIAL_PROTOCOLLNPGM("Select 4");
					}
					else if ((Event.reportObject.index == BUTTON_SD_SELECTED5) && FLAG_FilesUpDown)
					{
						FLAG_ListFilesSelect5 = true;
					}
					else if (Event.reportObject.index == BUTTON_FOLDER_BACK)
					{
						if (millis() >= waitPeriod_button_press){
							
							int updir = card.updir();
							
							if (updir==0){
								FLAG_ListFileEnterBackFolder = true;
								folder_navigation_register(false);
							}
							else if(updir==1){
								FLAG_ListFileEnterBackFolder = true;
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FOLDER_BACK,0);
								genie.WriteObject(GENIE_OBJ_USERIMAGES, USERIMAGE_FOLDER_FILE,0);
								folder_navigation_register(false);
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_INSERT_SD_CARD)
					{
						if (millis() >= waitPeriod_button_press){
							
							screen_sdcard = false;
							surfing_utilities=false;
							SERIAL_PROTOCOLPGM("Surfing 0 \n");
							surfing_temps = false;
							HeaterCooldownInactivity(true);
							genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN_SCREEN, 0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_SDCONFIRMATION_YES)
					{
						if (millis() >= waitPeriod_button_press){
							
							
							if(card.cardOK)
							{
								//doblocking = true;
								
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
									
									
									enquecommand_P(PSTR("M24")); // It also sends you to PRINTING screen
									
									screen_status="Ready...";//Write the selected SD file to all strings
								}
								
							}
						}
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					
					else if (Event.reportObject.index == BUTTON_SD_RIGHT || Event.reportObject.index == BUTTON_SD_LEFT )
					{
						if (millis() >= waitPeriod_button_press){
							
							
							if (Event.reportObject.index == BUTTON_SD_RIGHT) //RIGHT button pressed
							{
								FLAG_ListFilesUp = true;
							}
							else if (Event.reportObject.index == BUTTON_SD_LEFT) //LEFT button pressed
							{
								FLAG_ListFilesDown = true;
							}
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					#pragma endregion SD Gcode Selector
					
					
					#pragma region PREHEAT
					else if (Event.reportObject.index == BUTTON_MAINTENANCE ){
						
						if (millis() >= waitPeriod_button_press){
							genie.WriteObject(GENIE_OBJ_FORM, FORM_MAINTENANCE, 0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
						
					}
					else if (Event.reportObject.index == BUTTON_GO_TEMPS ){
						if (millis() >= waitPeriod_button_press){
							genie.WriteObject(GENIE_OBJ_FORM, FORM_TEMP_MENU, 0);
							HeaterCooldownInactivity(false);
							int tHotend=target_temperature[0];
							int tHotend1=target_temperature[1];
							int tBed=target_temperature_bed;
							
							
							FLAG_GifHotent0=false;
							FLAG_GifHotent1 = false;
							FLAG_GifBed = false;
							surfing_temps = true;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					else if (Event.reportObject.index == BUTTON_PREHEAT_BACK ){
						if (millis() >= waitPeriod_button_press){
							screen_sdcard = false;
							surfing_utilities=false;
							SERIAL_PROTOCOLPGM("Surfing 0 \n");
							surfing_temps = false;
							HeaterCooldownInactivity(true);
							genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN_SCREEN, 0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_PREHEAT_LEXTR ){
						int tHotend=target_temperature[0];
						if(tHotend != 0){
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PREHEAT_LEXTR,0); //<GIFF
							setTargetHotend0(0);
						}
						else{
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PREHEAT_LEXTR,1); //<GIFF
							setTargetHotend0(print_temp_l);
						}
						FLAG_GifHotent0 = false;
					}
					else if (Event.reportObject.index == BUTTON_PREHEAT_REXTR ){
						int tHotend1=target_temperature[1];
						if(tHotend1 != 0)
						{
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PREHEAT_REXTR,0); //<GIFF
							setTargetHotend1(0);
						}
						else{
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PREHEAT_REXTR,1); //<GIFF
							setTargetHotend1(print_temp_r);
						}
						FLAG_GifHotent1 = false;
					}
					else if (Event.reportObject.index == BUTTON_PREHEAT_BED ){
						int tBed=target_temperature_bed;
						if(tBed != 0){
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PREHEAT_BED,0); //<GIFF
							setTargetBed(0);
						}
						else {
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PREHEAT_BED,1); //<GIFF
							setTargetBed( max(bed_temp_l,bed_temp_r));
							
						}
						FLAG_GifBed = false;
					}
					
					#pragma endregion PREHEAT
					
					#pragma region RecoveyPrint
					
					else if (Event.reportObject.index == BUTTON_RECOVERY_PRINT_ASK_ACCEPT){
						if (millis() >= waitPeriod_button_press){
							enquecommand_P(PSTR("M34"));
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_RECOVERY_PRINT_ASK_CANCEL){
						if (millis() >= waitPeriod_button_press){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_RECOVERY_TOBELOST,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_RECOVERY_TOBELOST_ACCEPT){
						if (millis() >= waitPeriod_button_press){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
							screen_sdcard = false;
							surfing_utilities=false;
							SERIAL_PROTOCOLPGM("Surfing 0 \n");
							surfing_temps = false;
							saved_print_flag = 888;
							Config_StoreSettings();
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_RECOVERY_TOBELOST_BACK){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_RECOVERY_PRINT_ASK,0);
							card.initsd();
							if (card.cardOK){
								
								workDir_vector_lenght=saved_workDir_vector_lenght;
								for(int i=0; i<saved_workDir_vector_lenght;i++){
									card.getWorkDirName();
									card.getfilename(saved_workDir_vector[i]);
									workDir_vector[i]=saved_workDir_vector[i];
									if (!card.filenameIsDir){
										SERIAL_PROTOCOLLNPGM("Te pille");
										}else{
										if (card.chdir(card.filename)!=-1){
										}
									}
								}
								setfilenames(7);
							}
							
						}
					}
					
					#pragma endregion RecoveyPrint
					
					
					
					#pragma region Maintenance
					
					else if (Event.reportObject.index == Z_ADJUST){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
							if(saved_print_flag==1888){
								saved_print_flag = 888;
								Config_StoreSettings();
							}
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							processing = true;
							if(home_made_Z){
								home_axis_from_code(true,true,false);
								st_synchronize();
								if(processing_error)return;
							}
							else{
								home_axis_from_code(true,true,true);
								st_synchronize();
								if(processing_error)return;
							}
							HeaterCooldownInactivity(true);
							processing = false;
							enquecommand_P((PSTR("T0")));
							st_synchronize();
							if(processing_error)return;
							genie.WriteObject(GENIE_OBJ_FORM, FORM_ZSET, 0);
							
						}
					}
					
					else if (Event.reportObject.index == BUTTON_MAINTENANCE_BACKUTILITIES ){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES, 0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_AUTOTUNE_HOTENDS ){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
							genie.WriteObject(GENIE_OBJ_FORM, FORM_ADJUSTING_TEMPERATURES, 0);
							FLAG_PIDautotune = true;
							processing_adjusting = true;
							char buffer[25];
							int percentage = 0;
							sprintf(buffer, "%d%%", percentage);
							genie.WriteStr(STRING_ADJUSTING_TEMPERATURES,buffer);
							PID_autotune_Save(print_temp_l, 0, AUTOTUNE_ITERATIONS, 25.0);
							Config_StoreSettings();
							SERIAL_PROTOCOL(MSG_OK);
							SERIAL_PROTOCOL(" p:");
							SERIAL_PROTOCOL(Kp[0]);
							SERIAL_PROTOCOL(" i:");
							SERIAL_PROTOCOL(unscalePID_i(Ki[0]));
							SERIAL_PROTOCOL(" d:");
							SERIAL_PROTOCOL(unscalePID_d(Kd[0]));
							PID_autotune_Save(print_temp_r, 1, AUTOTUNE_ITERATIONS, 25.0);
							Config_StoreSettings();
							SERIAL_PROTOCOL(MSG_OK);
							SERIAL_PROTOCOL(" p:");
							SERIAL_PROTOCOL(Kp[1]);
							SERIAL_PROTOCOL(" i:");
							SERIAL_PROTOCOL(unscalePID_i(Ki[1]));
							SERIAL_PROTOCOL(" d:");
							SERIAL_PROTOCOL(unscalePID_d(Kd[1]));
							processing_adjusting = false;
							genie.WriteObject(GENIE_OBJ_FORM, FORM_CAL_WIZARD_DONE_GOOD, 0);
							processing_bed_success =  true;
							
						}
					}
					else if (Event.reportObject.index == BUTTON_MAINTENANCE_BACKMENU ){
						if (millis() >= waitPeriod_button_press){
							screen_sdcard = false;
							surfing_utilities=false;
							SERIAL_PROTOCOLPGM("Surfing 0 \n");
							surfing_temps = false;
							genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN_SCREEN, 0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == CLEAN_NYLON_METODE ){
						if (millis() >= waitPeriod_button_press){
							
							filament_mode = 'R';
							FLAG_NylonCleanMetode = true;
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_LEFT, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_RIGHT, 0);
							genie.WriteObject(GENIE_OBJ_USERIMAGES, IMAG_NYLON_SELECT_TEXT, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_SKIP, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_GO, 0);
							genie.WriteObject(GENIE_OBJ_FORM,FORM_NYLON_SELECT,0);
							
							which_extruder = 255;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						}
					}
					#pragma region Nylon
					else if (Event.reportObject.index == BUTTON_NYLON_SELECT_BACKMENU ){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM, FORM_MAINTENANCE, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_LEFT, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_RIGHT, 0);
							genie.WriteObject(GENIE_OBJ_USERIMAGES, IMAG_NYLON_SELECT_TEXT, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_SKIP, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_GO, 0);
							FLAG_NylonCleanMetode = false;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_NYLON_SELECT_BACKMAINMENU ){
						if (millis() >= waitPeriod_button_press){
							
							screen_sdcard = false;
							surfing_utilities=false;
							SERIAL_PROTOCOLPGM("Surfing 0 \n");
							surfing_temps = false;
							genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN_SCREEN, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_LEFT, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_RIGHT, 0);
							genie.WriteObject(GENIE_OBJ_USERIMAGES, IMAG_NYLON_SELECT_TEXT, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_SKIP, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_GO, 0);
							FLAG_NylonCleanMetode = false;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_NYLON_SELECT_LEFT ){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_LEFT, 1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_RIGHT, 0);
							genie.WriteObject(GENIE_OBJ_USERIMAGES, IMAG_NYLON_SELECT_TEXT, 1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_SKIP, 1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_GO, 1);
							which_extruder = 0;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_NYLON_SELECT_RIGHT ){
						if (millis() >= waitPeriod_button_press){
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_LEFT, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_RIGHT, 1);
							genie.WriteObject(GENIE_OBJ_USERIMAGES, IMAG_NYLON_SELECT_TEXT, 1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_SKIP, 1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_NYLON_SELECT_GO, 1);
							which_extruder = 1;
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_NYLON_SELECT_GO ){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							if(saved_print_flag==1888){
								saved_print_flag = 888;
								Config_StoreSettings();
							}
							if(which_extruder != 255){
								
								if(which_extruder == 0) setTargetHotend(max(remove_temp_l,old_remove_temp_l),which_extruder);
								else setTargetHotend(max(remove_temp_r,old_remove_temp_r),which_extruder);
								
								doblocking = true;
								processing = true;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								
								if (home_made_Z){
									home_axis_from_code(true,true,false);
								}
								else{
									home_axis_from_code(true,true,true);
								}
								
								current_position[Z_AXIS]=Z_MAX_POS-15;
								plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder); //check speed
								
								current_position[Y_AXIS]=10;
								plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Y_AXIS], active_extruder); //check speed
								st_synchronize();
								if(processing_error)return;
								
								processing = false;
								genie.WriteStr(STRING_CHANGE_FILAMENT_TEMPS,"0%");
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CHANGE_FILAMENT_TEMPS,0);
								Tref1 = (int)degHotend(which_extruder);
								Tfinal1 = (int)degTargetHotend(which_extruder);
								
								processing_change_filament_temps = true;
								
								is_changing_filament=true; //We are changing filament
								
								
							}
							
						}
					}
					else if (Event.reportObject.index == BUTTON_NYLON_SELECT_SKIP ){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							if(saved_print_flag==1888){
								saved_print_flag = 888;
								Config_StoreSettings();
							}
							if(which_extruder != 255){
								doblocking = true;
								setTargetHotend(NYLON_TEMP_HEATUP_THRESHOLD,which_extruder);
								processing = true;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								
								if (home_made_Z){
									home_axis_from_code(true,true,false);
								}
								else{
									home_axis_from_code(true,true,true);
								}
								st_synchronize();
								if(processing_error)return;
								current_position[Z_AXIS]=Z_MAX_POS-15;
								plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder); //check speed
								st_synchronize();
								if(processing_error)return;
								current_position[Y_AXIS]=10;
								plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Y_AXIS], active_extruder); //check speed
								st_synchronize();
								if(processing_error)return;
								
								
								
								SERIAL_PROTOCOLPGM("Filament Removed, GOING TO CLEAN THE NOZZLE \n");
								
								if (which_extruder == 0) changeTool(0);
								else changeTool(1);
								
								
								current_position[X_AXIS] = 155;
								
								plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
								st_synchronize();
								if(processing_error)return;
								processing = false;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_NYLON_STEP0,0);
								
							}
							
						}
					}
					
					
					#pragma endregion Nylon
					#pragma region Zset
					else if(Event.reportObject.index == BUTTON_Z_TOP ){
						//if(current_position[Z_AXIS]!=Z_MIN_POS){
						if (millis() >= waitPeriod_button_press){
							FLAG_ZAdjust50Up = true;
							FLAG_ZAdjust50Down = false;
							FLAG_ZAdjust10Down = false;
							FLAG_ZAdjust10Up = false;
							waitPeriod_button_press = millis()+250;
						}
						//}
						
					}
					else if(Event.reportObject.index == BUTTON_Z_BOT ){
						//if(current_position[Z_AXIS]!=Z_MAX_POS-15){
						if (millis() >= waitPeriod_button_press){
							FLAG_ZAdjust50Up = false;
							FLAG_ZAdjust50Down = true;
							FLAG_ZAdjust10Down = false;
							FLAG_ZAdjust10Up = false;
							waitPeriod_button_press = millis()+250;
						}
						//}
						
					}
					else if(Event.reportObject.index == BUTTON_Z_DOWN){
						//if(current_position[Z_AXIS]!=Z_MIN_POS){
						if (millis() >= waitPeriod_button_press){
							FLAG_ZAdjust50Up = false;
							FLAG_ZAdjust50Down = false;
							FLAG_ZAdjust10Down = true;
							FLAG_ZAdjust10Up = false;
							waitPeriod_button_press = millis()+250;
						}
						//}
						
					}
					else if(Event.reportObject.index == BUTTON_Z_UP ){
						//if(current_position[Z_AXIS]!=Z_MAX_POS-15){
						if (millis() >= waitPeriod_button_press){
							FLAG_ZAdjust50Up = false;
							FLAG_ZAdjust50Down = false;
							FLAG_ZAdjust10Down = false;
							FLAG_ZAdjust10Up = true;
							waitPeriod_button_press = millis()+250;
						}
						//}
						
					}
					else if(Event.reportObject.index == BUTTON_Z_BACK ){
						if (millis() >= waitPeriod_button_press){
							genie.WriteObject(GENIE_OBJ_FORM, FORM_MAINTENANCE, 0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_Z_ACCEPT ){
						if (millis() >= waitPeriod_button_press){
							genie.WriteObject(GENIE_OBJ_FORM, FORM_MAINTENANCE, 0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					#pragma endregion Zset
					
					#pragma endregion Maintenance

					#pragma region PURGE
					//****************PURGE BUTTONS******
					else if (Event.reportObject.index == BUTTON_PURGE_LEFT ){
						if (millis() >= waitPeriod_button_press){
							
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_INSERT,1);
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_Retract,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_TEMP_UP,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_TEMP_DOWN,1);
							if (purge_extruder_selected == 1){
								purge_extruder_selected = 0;
								if(target_temperature[0] == 0){
									setTargetHotend0(print_temp_l);
								}
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
								char buffer[256];
								sprintf(buffer, "%d %cC",target_temperature[0],0x00B0);
								genie.WriteStr(STRING_PURGE_SELECTED,buffer);
								Serial.println(buffer);
								sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
								genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);
							}
							else{
								char buffer[256];
								purge_extruder_selected = 0;
								if(target_temperature[0] == 0){
									setTargetHotend0(print_temp_l);
								}
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
								sprintf(buffer, "%d %cC",target_temperature[0],0x00B0);
								genie.WriteStr(STRING_PURGE_SELECTED,buffer);
								sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
								genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_PURGE_RIGHT ){
						if (millis() >= waitPeriod_button_press){
							
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_INSERT,1);
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_Retract,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_TEMP_UP,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_TEMP_DOWN,1);
							if (purge_extruder_selected == 0){
								purge_extruder_selected = 1;
								
								if(target_temperature[1] == 0){
									setTargetHotend1(print_temp_r);
								}
								
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,1);
								char buffer[256];
								sprintf_P(buffer, PSTR("%3d %cC"),target_temperature[1],0x00B0);
								genie.WriteStr(STRING_PURGE_SELECTED,buffer);
								Serial.println(buffer);
								sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
								genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);
							}
							else{
								purge_extruder_selected = 1;
								if(target_temperature[1] == 0){
									setTargetHotend1(print_temp_r);
								}
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,1);
								char buffer[256];
								sprintf_P(buffer, PSTR("%3d %cC"),target_temperature[1],0x00B0);
								genie.WriteStr(STRING_PURGE_SELECTED,buffer);
								sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
								genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_PURGE_TEMP_UP && purge_extruder_selected != -1){
						
						if (target_temperature[purge_extruder_selected] < HEATER_0_MAXTEMP){
							target_temperature[purge_extruder_selected] += 5;
							setTargetHotend0(target_temperature[0]);
							setTargetHotend1(target_temperature[1]);
							char buffer[256];
							sprintf_P(buffer, PSTR("%3d %cC"),int(target_temperature[purge_extruder_selected]),0x00B0);
							genie.WriteStr(STRING_PURGE_SELECTED,buffer);
						}
					}
					else if(Event.reportObject.index == BUTTON_PURGE_TEMP_DOWN && purge_extruder_selected != -1){
						if (target_temperature[purge_extruder_selected] > 0){
							target_temperature[purge_extruder_selected] -= 5;
							setTargetHotend0(target_temperature[0]);
							setTargetHotend1(target_temperature[1]);
							char buffer[256];
							sprintf_P(buffer, PSTR("%3d %cC"),int(target_temperature[purge_extruder_selected]),0x00B0);
							genie.WriteStr(STRING_PURGE_SELECTED,buffer);
						}
					}
					//***MOVING
					else if(Event.reportObject.index == BUTTON_PURGE_Retract && purge_extruder_selected != -1){
						if(!blocks_queued()){
							FLAG_PurgeSelect1 = 1;
							}else{
							quickStop();
						}
						
						/*if (millis() >= waitPeriod_button_press){
						if(degHotend(purge_extruder_selected) >= target_temperature[purge_extruder_selected]-PURGE_TEMP_HYSTERESIS){
						current_position[E_AXIS]-=5;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60, purge_extruder_selected);//Retract
						}
						waitPeriod_button_press=millis()+2500;
						}*/
					}
					else if(Event.reportObject.index == BUTTON_PURGE_INSERT && purge_extruder_selected != -1){
						if(!blocks_queued()){
							FLAG_PurgeSelect0 = 1;
							}else{
							quickStop();
						}
						
						
						/*if (millis() >= waitPeriod_button_press){
						if(degHotend(purge_extruder_selected) >= target_temperature[purge_extruder_selected]-PURGE_TEMP_HYSTERESIS){
						current_position[E_AXIS]+=PURGE_DISTANCE_INSERTED;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60, purge_extruder_selected);//Purge
						
						}
						waitPeriod_button_press=millis()+PURGE_DISTANCE_INSERTED*300;
						}*/
					}
					//else if(Event.reportObject.index == BUTTON_PURGE_INSERTX3 && purge_extruder_selected != -1){
					//if (millis() >= waitPeriod_button_press){
					//if(degHotend(purge_extruder_selected) >= target_temperature[purge_extruder_selected]-PURGE_TEMP_HYSTERESIS){
					//current_position[E_AXIS]+=15;
					//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60, purge_extruder_selected);//Purge
					//}
					//waitPeriod_button_press=millis()+7500;
					//}
					//}
					
					else if(Event.reportObject.index == BUTTON_PURGE){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_INSERT,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_Retract,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_TEMP_UP,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_TEMP_DOWN,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_MENU,0);
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PURGE,0);
							
							purge_extruder_selected = -1;
							SERIAL_PROTOCOLPGM("Enter in purge mode \n");
							/*setTargetHotend0(print_temp_l);
							setTargetHotend1(print_temp_r);*/
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
							/*if(purge_extruder_selected == 0) {
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
							
							}
							else {
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,1);
							}*/
							
							char buffer[256];
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
							genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);	Serial.println(buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
							genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);	Serial.println(buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),0,0x00B0);
							genie.WriteStr(STRING_PURGE_SELECTED,buffer);	Serial.println(buffer);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					else if(Event.reportObject.index	== BUTTON_PURGE_BACK){
						if (millis() >= waitPeriod_button_press){
							
							quickStop();
							HeaterCooldownInactivity(true);
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FILAMENT,0);
							
							//setTargetHotend0(0);
							//setTargetHotend1(0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index	== BUTTON_PURGE_MENU){
						if (millis() >= waitPeriod_button_press){
							
							quickStop();
							screen_sdcard = false;
							surfing_utilities=false;
							surfing_temps = false;
							SERIAL_PROTOCOLPGM("Surfing 0 \n");
							HeaterCooldownInactivity(true);
							genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
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
					
					else if (Event.reportObject.index == BUTTON_FILAMENT_OPTIONS_BACK  )
					{
						if (millis() >= waitPeriod_button_press){
							
							FLAG_FilamentHome=false;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_REMOVE_FIL){
						if (millis() >= waitPeriod_button_press){
							filament_mode = 'R';//Remove Mode
							which_extruder = -1;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_REMOVE_FIL,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_INSERT_FIL)
					{
						if (millis() >= waitPeriod_button_press){
							
							filament_mode = 'I'; //Insert Mode
							
							/*if (!FLAG_FilamentHome){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							
							}*/
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 0);
							
							//genie.WriteObject(GENIE_OBJ_IMAGE, IMAG_MATERIALS, -1);
							
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PLA, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_ABS, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PVA, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUST, 0);
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_SELECT_EXTRUDER,0);
							which_extruder = -1;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_REMOVE_SELECT_LEFT || Event.reportObject.index == BUTTON_REMOVE_SELECT_RIGHT)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							if (Event.reportObject.index == BUTTON_REMOVE_SELECT_LEFT) //Left Nozzle
							{
								
								which_extruder=0;
								
							}
							else //Right Nozzle
							{
								
								which_extruder=1;
							}
							
							if(which_extruder == 0) setTargetHotend(max(remove_temp_l,old_remove_temp_l),which_extruder);
							else setTargetHotend(max(remove_temp_r,old_remove_temp_r),which_extruder);
							processing = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							
							if (home_made_Z){
								home_axis_from_code(true,true,false);
							}
							else{
								home_axis_from_code(true,true,true);
							}
							
							current_position[Z_AXIS]=Z_MAX_POS-15;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder); //check speed
							
							current_position[Y_AXIS]=10;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Y_AXIS], active_extruder); //check speed
							st_synchronize();
							if(processing_error)return;
							
							processing = false;
							genie.WriteStr(STRING_CHANGE_FILAMENT_TEMPS,"0%");
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CHANGE_FILAMENT_TEMPS,0);
							Tref1 = (int)degHotend(which_extruder);
							Tfinal1 = (int)degTargetHotend(which_extruder);
							
							processing_change_filament_temps = true;
							
							is_changing_filament=true; //We are changing filament
							
						}
						
					}
					else if (Event.reportObject.index == BUTTON_FILAMENT_BACK && !Step_First_Start_Wizard){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FILAMENT,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_REMOVE_BACK_FILAMENT){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FILAMENT,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_REMOVE_MENU_FILAMENT){
						if (millis() >= waitPeriod_button_press){
							screen_sdcard = false;
							surfing_utilities=false;
							surfing_temps = false;
							SERIAL_PROTOCOLPGM("Surfing 0 \n");
							genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}

					}
					else if (Event.reportObject.index == BUTTON_SELECT_EXTRUDER_MENU && !Step_First_Start_Wizard){
						if (millis() >= waitPeriod_button_press){
							
							screen_sdcard = false;
							surfing_utilities=false;
							surfing_temps = false;
							SERIAL_PROTOCOLPGM("Surfing 0 \n");
							genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}

					else if ((Event.reportObject.index == BUTTON_FILAMENT_NOZZLE1 || Event.reportObject.index == BUTTON_FILAMENT_NOZZLE2) && !Step_First_Start_Wizard)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							if (Event.reportObject.index == BUTTON_FILAMENT_NOZZLE1) //Left Nozzle
							{
								
								which_extruder=0;
								
							}
							else //Right Nozzle
							{
								
								which_extruder=1;
							}
							if (filament_mode == 'I') {
								if (which_extruder == 0){
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 0);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PLA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_ABS, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PVA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUST, 1);
								}
								else{
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 0);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PLA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_ABS, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PVA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUST, 1);
								}
							}
							
							
							
							else {
								//*********Move the bed down and the extruders inside
								if(which_extruder == 0) setTargetHotend(max(remove_temp_l,old_remove_temp_l),which_extruder);
								else setTargetHotend(max(remove_temp_r,old_remove_temp_r),which_extruder);
								processing = true;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								
								if (home_made_Z){
									home_axis_from_code(true,true,false);
								}
								else{
									home_axis_from_code(true,true,true);
								}
								/*int feedrate;
								if (!FLAG_FilamentHome){
								//MOVING THE EXTRUDERS TO AVOID HITTING THE CASE WHEN PROBING-------------------------
								//current_position[X_AXIS]+=25;
								home_axis_from_code(true,true,false);
								st_synchronize();
								
								FLAG_FilamentHome=true;
								}*/
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
								if(processing_error)return;
								
								processing = false;
								
								//genie.WriteObject(GENIE_OBJ_USERIMAGES,10,1);
								genie.WriteStr(STRING_CHANGE_FILAMENT_TEMPS,"0%");
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CHANGE_FILAMENT_TEMPS,0);
								Tref1 = (int)degHotend(which_extruder);
								Tfinal1 = (int)degTargetHotend(which_extruder);
								/****************************************************/
								
								//ATTENTION : Order here is important
								
								
								//Serial.println("REMOVING");
								//genie.WriteStr(STRING_ADVISE_FILAMENT,"");
								/*if (filament_mode == 'I') genie.WriteObject(GENIE_OBJ_USERIMAGES,10,0);
								else if (filament_mode == 'R') genie.WriteObject(GENIE_OBJ_USERIMAGES,10,1);
								else genie.WriteObject(GENIE_OBJ_USERIMAGES,10,1);*/
								processing_change_filament_temps = true;
								//delay(3500);
								/*if(which_extruder == 0) setTargetHotend(max(remove_temp_l,old_remove_temp_l),which_extruder);
								else setTargetHotend(max(remove_temp_r,old_remove_temp_r),which_extruder);*/
								is_changing_filament=true; //We are changing filament
								
							}
							
						}
					}
					else if (Event.reportObject.index == BUTTON_PLA){
						if (millis() >= waitPeriod_button_press){
							
							saved_print_flag = 888;
							if (which_extruder == 1) // Need to pause
							{
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								
								
								print_temp_r = PLA_PRINT_TEMP;
								insert_temp_r = PLA_INSERT_TEMP;
								remove_temp_r = PLA_REMOVE_TEMP;
								bed_temp_r = PLA_BED_TEMP;
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
								Config_StoreSettings();
								setTargetHotend1(print_temp_r);
								insertmetod();
							}
							else if(which_extruder == 0){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								
								print_temp_l = PLA_PRINT_TEMP;
								insert_temp_l = PLA_INSERT_TEMP;
								remove_temp_l = PLA_REMOVE_TEMP;
								bed_temp_l = PLA_BED_TEMP;
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
								Config_StoreSettings();
								setTargetHotend0(print_temp_l);
								insertmetod();
							}
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_ABS){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							saved_print_flag = 888;
							if (which_extruder == 1) // Need to pause
							{
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								
								
								print_temp_r = ABS_PRINT_TEMP;
								insert_temp_r = ABS_INSERT_TEMP;
								remove_temp_r = ABS_REMOVE_TEMP;
								bed_temp_r = ABS_BED_TEMP;
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
								Config_StoreSettings();
								setTargetHotend1(print_temp_r);
								insertmetod();
							}
							else if(which_extruder == 0){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								
								print_temp_l = ABS_PRINT_TEMP;
								insert_temp_l = ABS_INSERT_TEMP;
								remove_temp_l = ABS_REMOVE_TEMP;
								bed_temp_l = ABS_BED_TEMP;
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
								Config_StoreSettings();
								setTargetHotend0(print_temp_l);
								insertmetod();
							}
							
							
						}
					}
					else if (Event.reportObject.index == BUTTON_PVA){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							saved_print_flag = 888;
							if (which_extruder == 1) // Need to pause
							{
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								
								
								print_temp_r = PVA_PRINT_TEMP;
								insert_temp_r = PVA_INSERT_TEMP;
								remove_temp_r = PVA_REMOVE_TEMP;
								bed_temp_r = PVA_BED_TEMP;
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
								Config_StoreSettings();
								setTargetHotend1(print_temp_r);
								insertmetod();
							}
							else if(which_extruder == 0){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								
								print_temp_l = PVA_PRINT_TEMP;
								insert_temp_l = PVA_INSERT_TEMP;
								remove_temp_l = PVA_REMOVE_TEMP;
								bed_temp_l = PVA_BED_TEMP;
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
								Config_StoreSettings();
								setTargetHotend0(print_temp_l);
								insertmetod();
							}
							
						}
					}
					
					//CUSTOM MATERIAL BUTTONS
					else if(Event.reportObject.index == BUTTON_CUST){
						if (millis() >= waitPeriod_button_press){
							
							if (which_extruder == 1 || which_extruder == 0) // Need to pause
							{
								if(Step_First_Start_Wizard){
									//genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUSTOM_MENU, 1);
								}
								
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
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_CUSTOM_BACK){
						if (millis() >= waitPeriod_button_press){
							
							if(Step_First_Start_Wizard){
								if(which_extruder == 0){
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 0);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 1);
									
									genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SELECT_EXTRUDER_MENU,1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FILAMENT_BACK,1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PLA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_ABS, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PVA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUST, 1);
									}else if (which_extruder == 1){
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 0);
									
									genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SELECT_EXTRUDER_MENU,1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FILAMENT_BACK,1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PLA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_ABS, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PVA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUST, 1);
								}
								
								}else{
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PLA, 0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_ABS, 0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PVA, 0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUST, 0);
								
								
								which_extruder = -1;
							}
							genie.WriteObject(GENIE_OBJ_FORM,FORM_SELECT_EXTRUDER,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					/*else if (Event.reportObject.index == BUTTON_CUSTOM_MENU){
					screen_sdcard = false;
					surfing_utilities=false;
					surfing_temps = false;
					SERIAL_PROTOCOLPGM("Surfing 0 \n");
					genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);

					}*/
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
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							saved_print_flag = 888;
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
							insertmetod();
						}
					}
					
					

					else if(Event.reportObject.index == BUTTON_MOVE_INSERT){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							//ATTENTION : Order here is important
							genie.WriteStr(STRING_CHANGE_FILAMENT_TEMPS,"0%");
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CHANGE_FILAMENT_TEMPS,0);
							Tref1 = (int)degHotend(which_extruder);
							Tfinal1 = (int)degTargetHotend(which_extruder);
							//genie.WriteStr(STRING_ADVISE_FILAMENT,"");
							//genie.WriteStr(STRING_ADVISE_FILAMENT,"Insert the filament until you feel it stops, \n then while you keep inserting around \n 10 mm of filament, press the clip");
							processing_change_filament_temps = true;
							
							if (which_extruder==0) setTargetHotend(max(insert_temp_l,old_insert_temp_l),which_extruder);
							else setTargetHotend(max(insert_temp_r,old_insert_temp_r),which_extruder);
							//delay(3500);
							is_changing_filament=true;
							
							if (which_extruder == 0) changeTool(0);
							else changeTool(1);
							
							current_position[Y_AXIS] = 100;
							plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
							st_synchronize();
							if(processing_error)return;
						}
					}
					else if (Event.reportObject.index == BUTTON_INSERT ){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							if (filament_mode =='I')
							{ //Inserting...
								processing = true;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								current_position[X_AXIS] = 155;
								plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
								delay(850);
								st_synchronize();
								SERIAL_PROTOCOLPGM("Inserting :   \n");
								doblocking = false;
								current_position[E_AXIS] += 30;//Extra extrusion at low feedrate
								plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  700/60, which_extruder); //850/60
								current_position[E_AXIS] += ((BOWDEN_LENGTH-EXTRUDER_LENGTH)-15);//BOWDEN_LENGTH-300+340);
								st_synchronize();
								Serial.println(current_position[E_AXIS]);
								plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, which_extruder);
								st_synchronize();
								current_position[E_AXIS] += EXTRUDER_LENGTH;//Extra extrusion at low feedrate
								plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_SLOW_SPEED/60, which_extruder);
								st_synchronize();
								
								if(processing_error)return;
								processing = false;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUST_FILAMENT,0);
							}
						}
					}
					
					
					else if (Event.reportObject.index == BUTTON_REMOVE )
					{// We should have already checked if filament is inserted
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							if (filament_mode =='R')
							{ //Removing...
								processing = true;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								current_position[E_AXIS] -= (BOWDEN_LENGTH + EXTRUDER_LENGTH + 100);//Extra extrusion at fast feedrate
								plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_FAST_SPEED/60, which_extruder);
								previous_state = FORM_FILAMENT;
								
								st_synchronize();
								if(processing_error)return;
								processing = false;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_SUCCESS_FILAMENT,0);
								processing_success = true;
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
							}
						}
					}
					else if (Event.reportObject.index == BUTTON_NYLON_STEP0)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_NYLON_TEMPS,0);
							processing_nylon_temps = true;
							int Tref = (int)degHotend(which_extruder);
							int Tfinal = (int)(degTargetHotend(which_extruder)-NYLON_TEMP_HYSTERESIS);
							int percentage = 0;
							while (degHotend(which_extruder)<(degTargetHotend(which_extruder)-NYLON_TEMP_HYSTERESIS)){ //Waiting to heat the extruder
								if (millis() >= waitPeriod_s){
									char buffer[25];
									memset(buffer, '\0', sizeof(buffer) );
									
									int Tinstant;
									if(Tref > (int)degHotend(which_extruder)){
										Tinstant = Tref;
										}else if((int)degHotend(which_extruder) > Tfinal){
										Tinstant = Tfinal;
										}else{
										Tinstant = (int)degHotend(which_extruder);
									}
									
									percentage = Tfinal-Tref;
									percentage = 100*(Tinstant-Tref)/percentage;
									sprintf(buffer, "%d%%", percentage);
									genie.WriteStr(STRING_NYLON_TEMPS,buffer);
									waitPeriod_s=2000+millis();
								}
								manage_heater();
								touchscreen_update();
								if(processing_error)return;
							}
							processing_nylon_temps = false;
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_NYLON_STEP2,0);
							
						}
					}
					else if (Event.reportObject.index == BUTTON_NYLON_STEP2)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							setTargetHotend(0.0,which_extruder);
							if(which_extruder == 0)digitalWrite(FAN_PIN, 1);
							else digitalWrite(FAN2_PIN, 1);
							genie.WriteObject(GENIE_OBJ_FORM,FORM_NYLON_STEP3,0);
							//processing_nylon_temps = true;
							processing_nylon_step3 = true;
							int Tref = (int)degHotend(which_extruder);
							int Tfinal = 160;
							int percentage = 0;
							while (degHotend(which_extruder)>160.0){ //Waiting to heat the extruder
								//previous_millis_cmd = millis();
								
								if (millis() >= waitPeriod_s){
									char buffer[25];
									memset(buffer, '\0', sizeof(buffer) );
									int Tinstant;
									if(Tref < (int)degHotend(which_extruder)){
										Tinstant = Tref;
										}else if((int)degHotend(which_extruder) < Tfinal){
										Tinstant = Tfinal;
										}else{
										Tinstant = (int)degHotend(which_extruder);
									}
									
									percentage = ((Tref-Tfinal)-(Tinstant-Tfinal))*100;
									percentage = percentage/(Tref-Tfinal);
									sprintf(buffer, "%d%%", percentage);
									genie.WriteStr(STRING_NYLON_STEP3,buffer);
									waitPeriod_s=2000+millis();
								}
								manage_heater();
								touchscreen_update();
								if(processing_error)return;
							}
							processing_nylon_step3 = false;
							fanSpeed=255;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_NYLON_STEP4,0);
							processing_nylon_step4 = true;
						}
					}
					else if (Event.reportObject.index == BUTTON_NYLON_STEP4)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							if(which_extruder == 0)digitalWrite(FAN_PIN, 1);
							else digitalWrite(FAN2_PIN, 1);
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_NYLON_TEMPS,0);
							processing_nylon_temps = true;
							int Tref = (int)degHotend(which_extruder);
							int Tfinal = NYLON_TEMP_COOLDOWN_THRESHOLD;
							int percentage = 0;
							while (degHotend(which_extruder)>Tfinal){ //Waiting to heat the extruder
								if (millis() >= waitPeriod_s){
									char buffer[25];
									memset(buffer, '\0', sizeof(buffer) );
									int Tinstant;
									if(Tref < (int)degHotend(which_extruder)){
										Tinstant = Tref;
										}else if((int)degHotend(which_extruder) < Tfinal){
										Tinstant = Tfinal;
										}else{
										Tinstant = (int)degHotend(which_extruder);
									}
									
									percentage = ((Tref-Tfinal)-(Tinstant-Tfinal))*90; //<<<<<<<<<<<<<  0% TO 90%
									percentage = percentage/(Tref-Tfinal);
									sprintf(buffer, "%d%%", percentage);
									genie.WriteStr(STRING_NYLON_TEMPS,buffer);
									waitPeriod_s=2000+millis();
								}
								//previous_millis_cmd = millis();
								manage_heater();
								touchscreen_update();
								if(processing_error)return;
								
							}
							SERIAL_PROTOCOLPGM("60 degrees \n");
							fanSpeed=0;
							if(which_extruder == 0)digitalWrite(FAN_PIN, 0);
							else digitalWrite(FAN2_PIN, 0);
							setTargetHotend(105.0,which_extruder);
							Tref = (int)degHotend(which_extruder);
							Tfinal = 105-NYLON_TEMP_HYSTERESIS;
							percentage = 0;
							while (degHotend(which_extruder)<(degTargetHotend(which_extruder)-NYLON_TEMP_HYSTERESIS)){ //Waiting to heat the extruder
								
								if (millis() >= waitPeriod_s){
									char buffer[25];
									memset(buffer, '\0', sizeof(buffer) );
									
									int Tinstant;
									if(Tref > (int)degHotend(which_extruder)){
										Tinstant = Tref;
										}else if((int)degHotend(which_extruder) > Tfinal){
										Tinstant = Tfinal;
										}else{
										Tinstant = (int)degHotend(which_extruder);
									}
									percentage = Tfinal-Tref;
									percentage = 90+ 10*(Tinstant-Tref)/percentage;//<<<<<<<<<<<<<  90% TO 100%
									sprintf(buffer, "%d%%", percentage);
									genie.WriteStr(STRING_NYLON_TEMPS,buffer);
									waitPeriod_s=2000+millis();
								}
								manage_heater();
								touchscreen_update();
								if(processing_error)return;
								
							}
							processing_nylon_temps = false;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_NYLON_STEP5,0);
						}
					}
					else if (Event.reportObject.index == BUTTON_NYLON_STEP5)
					{
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_NYLON_STEP6,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_NYLON_REPEAT)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							setTargetHotend(NYLON_TEMP_HEATUP_THRESHOLD,which_extruder);
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_NYLON_TEMPS,0);
							processing_nylon_temps = true;
							int Tref = (int)degHotend(which_extruder);
							int Tfinal = (int)(degTargetHotend(which_extruder)-NYLON_TEMP_HYSTERESIS);
							int percentage = 0;
							while (degHotend(which_extruder)<(degTargetHotend(which_extruder)-NYLON_TEMP_HYSTERESIS)){ //Waiting to heat the extruder
								if (millis() >= waitPeriod_s){
									char buffer[25];
									memset(buffer, '\0', sizeof(buffer) );
									
									percentage = Tfinal-Tref;
									percentage = 100*((int)degHotend(which_extruder)-Tref)/percentage;
									sprintf(buffer, "%d%%", percentage);
									genie.WriteStr(STRING_NYLON_TEMPS,buffer);
									waitPeriod_s=2000+millis();
								}
								manage_heater();
								touchscreen_update();
								if(processing_error)return;
							}
							processing_nylon_temps = false;
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_NYLON_STEP2,0);
						}
					}
					else if (Event.reportObject.index == BUTTON_NYLON_SUCCESS)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							doblocking = false;
							setTargetHotend0(0);
							setTargetHotend1(0);
							HeaterCooldownInactivity(true);
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							processing = true;
							home_axis_from_code(true, true, false);
							processing =  false;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_MAINTENANCE,0);
							FLAG_NylonCleanMetode = false;
						}
					}
					
					#pragma endregion Insert_Remove_Fil
					
					
					//*****AdjustFilament******
					#pragma region AdjustFilament
					else if (Event.reportObject.index == BUTTON_ACCEPT_ADJUST && FLAG_FilamentAcceptOk == false)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							if(blocks_queued()) quickStop();
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							FLAG_FilamentAcceptOk = true;
							home_made = false;
							processing=true;
							home_axis_from_code(true,true,false);
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_SUCCESS_FILAMENT,0);
						}
						
					}
					
					else if (Event.reportObject.index == BUTTON_ADJUST_Load  && FLAG_FilamentAcceptOk == false)
					{
						if(!blocks_queued()){
							FLAG_LoadSelect0 = 1;
							}else{
							quickStop();
						}
						
					}
					
					else if (Event.reportObject.index == BUTTON_ADJUST_Unload  && FLAG_FilamentAcceptOk == false)
					{
						if(!blocks_queued()){
							FLAG_UnloadSelect1 = 1;
							}else{
							quickStop();
						}
					}
					#pragma endregion AdjustFilament
					
					
					
					//Extruder Calibrations-------------------------------------------------
					else if (Event.reportObject.index == BUTTON_CAL_FULL)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
							bed_calibration_times = 0;
							if(saved_print_flag==1888){
								saved_print_flag = 888;
								Config_StoreSettings();
							}
							
							SERIAL_PROTOCOLPGM("INFO: BED CALIB - ");
							Serial.println(FLAG_CalibBedDone);
							FLAG_CalibFull = true;
							
							//enquecommand_P(PSTR("T0"));
							if(!FLAG_CalibBedDone){  //Do g34
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								processing = true;
								doblocking= true;
								home_axis_from_code(true,true,true);
								st_synchronize();
								if(processing_error)return;
								enquecommand_P(PSTR("G34"));	//Start BED Calibration Wizard
								changeTool(0);
								
								
							}
							else{
								
								active_extruder = LEFT_EXTRUDER;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								processing = true;
								setTargetHotend0(print_temp_l);
								setTargetHotend1(print_temp_r);
								setTargetBed(max(bed_temp_l,bed_temp_r));
								
								
								home_axis_from_code(true,true,false);
								st_synchronize();
								if(processing_error)return;
								enquecommand_P(PSTR("T0"));
								processing = false;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_ZL,0);
								if(Step_First_Start_Wizard){
									genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_ZL_SKIP,1);
								}
								
								
								
								
								
								
								
								
							}
						}
						
					}
					else if (Event.reportObject.index == BUTTON_MANUAL_FINE_CALIB)
					{
						if (millis() >= waitPeriod_button_press){
							
							offset_calib_manu[0]=0.0;
							offset_calib_manu[1]=0.0;
							offset_calib_manu[2]=0.0;
							offset_calib_manu[3]=0.0;
							calib_value_selected = 0;
							char buffer[25];
							memset(buffer, '\0', sizeof(buffer) );
							sprintf(buffer, "0.000");
							
							genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_MANUAL_FINE_CALIB,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_ZR,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_ZL,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_X,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_Y,0);
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_RIGHT,0);
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_LEFT,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_UP,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_DOWN,1);
							genie.WriteObject(GENIE_OBJ_FORM, FORM_MANUAL_FINE_CALIB,0);
							genie.WriteStr(STRING_MANUAL_FINE_CALIB,manual_fine_calib_offset[0],3);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						}
					}
					
					/*
					else if ((Event.reportObject.index == USERBUTTON_CLEAN_DONE) && (flag_continue_calib)){
					
					genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Clean the right nozzle \n and press GO, \n then the Z calibration will start");
					if (active_extruder == 0)	{
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"        Z AXIS");
					
					home_axis_from_code(true,false,false);
					doblocking=false;
					enquecommand_P(PSTR("G43"));
					flag_continue_calib = false;
					
					}
					else {
					genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL,0);
					genie.WriteStr(STRING_AXIS,"        Z AXIS");
					
					home_axis_from_code(true,false,false);
					doblocking=false;
					enquecommand_P(PSTR("G43"));
					st_synchronize();
					
					flag_continue_calib = false;
					}
					
					}*/
					
					
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
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
							if(saved_print_flag==1888){
								saved_print_flag = 888;
								Config_StoreSettings();
							}
							processing = true;
							doblocking = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							bed_calibration_times = 0;
							FLAG_CalibFull = false;
							home_axis_from_code(true,true,true);
							enquecommand_P(PSTR("G34"));	//Start BED Calibration Wizard
							changeTool(0);
							previous_state = FORM_CALIBRATION;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_REDO_BED_CALIB )
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
							processing = true;
							doblocking = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							home_axis_from_code(true,true,false);
							changeTool(0);
							enquecommand_P((PSTR("G34")));
							previous_state = FORM_CALIBRATION;
							FLAG_CalibBedDone = true;
						}
					}
					
					
					else if (Event.reportObject.index == BUTTON_BED_CALIB_SW3)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
							char buffer[256];
							if (vuitens3!=0){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW3,0);
								sprintf(buffer, " %d / 8",vuitens3); //Printing how to calibrate on screen
								//genie.WriteStr(STRING_BED_SCREW3,buffer);
								if (sentit3>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,vuitens3);} //The direction is inverted in Sigma's bed screws
								else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,vuitens3+8);}
								}else{
								processing = true;
								doblocking = true;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								home_axis_from_code(true,true,false);
								changeTool(0);
								enquecommand_P((PSTR("G34")));
								previous_state = FORM_CALIBRATION;
								
								FLAG_CalibBedDone = true;
							}
						}
					}
					#pragma endregion Bed Calibration
					else if (Event.reportObject.index == BUTTON_INFO_TURN_SCREWS || Event.reportObject.index == BUTTON_INFO_TURN_SCREWS_FIRST)
					{
						if (millis() >= waitPeriod_button_press){
							
							processing_bed = false;
							processing_bed_first = false;
							char buffer[256];
							/*if (vuitens1!= 0){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW1,0);
							sprintf(buffer, " %d / 8",vuitens1); //Printing how to calibrate on screen
							//genie.WriteStr(STRING_BED_SCREW1,buffer);
							if (vuitens2==0 && vuitens3==0) {genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW2,0);}
							else{genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW2,0);}
							if (sentit1>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW1,vuitens1);} //The direction is inverted in Sigma's bed screws
							else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW1,vuitens1+8);}
							}*/
							if (vuitens2!= 0){
								SERIAL_PROTOCOLPGM("Jump over screw1 \n");
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW2,0);
								sprintf(buffer, " %d / 8",vuitens2); //Printing how to calibrate on screen
								//genie.WriteStr(STRING_BED_SCREW2,buffer);
								if (vuitens3==0) genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW3,0);
								if (sentit2>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,vuitens2);} //The direction is inverted in Sigma's bed screws
								else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,vuitens2+8);}
							}
							else if (vuitens3!= 0){
								SERIAL_PROTOCOLPGM("Jump over screw1 and screw2 \n");
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW3,0);
								sprintf(buffer, " %d / 8",vuitens3); //Printing how to calibrate on screen
								//genie.WriteStr(STRING_BED_SCREW3,buffer);
								if (sentit3>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,vuitens3);} //The direction is inverted in Sigma's bed screws
								else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,vuitens3+8);}
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					//*****Success Screens*****
					#pragma region SuccessScreens
					else if (Event.reportObject.index == BUTTON_BED_CALIB_SUCCESS )
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							processing_bed_success = false;
							if(FLAG_PIDautotune){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_MAINTENANCE,0);
								FLAG_PIDautotune = false;
							}
							else{
								
								//enquecommand_P((PSTR("G28 X0 Y0")));
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								processing = true;
								setTargetHotend0(0);
								setTargetHotend1(0);
								setTargetBed(0);
								home_axis_from_code(true, true, false);
								enquecommand_P((PSTR("T0")));
								st_synchronize();
								if(processing_error)return;
								SERIAL_PROTOCOLPGM("Calibration Successful, going back to main menu \n");
								processing = false;
								
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIBRATION,0);
								
								FLAG_CalibBedDone = true;
								
								doblocking=false;
							}
							
						}
					}
					
					else if (Event.reportObject.index == BUTTON_SUCCESS_FILAMENT_OK)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							processing_success = false;
							if(Step_First_Start_Wizard){
								if(which_extruder == 0){
									enquecommand_P((PSTR("T0")));
									SERIAL_PROTOCOLPGM("Filament Inserted/Removed, going to the next extruder \n");
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 0);
									genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SELECT_EXTRUDER_MENU,1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FILAMENT_BACK,1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PLA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_ABS, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PVA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUST, 1);
									genie.WriteObject(GENIE_OBJ_FORM,FORM_SELECT_EXTRUDER,0);
									
									which_extruder = 1;
								}
								else if (which_extruder == 1){
									enquecommand_P((PSTR("T0")));
									SERIAL_PROTOCOLPGM("Filament Inserted/Removed, going to Calib \n");
									
									genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_STEP_2,0);
									which_extruder = 0;
								}
							}
							else if(!FLAG_NylonCleanMetode){
								enquecommand_P((PSTR("T0")));
								SERIAL_PROTOCOLPGM("Filament Inserted/Removed, returning to Main Menu \n");
								genie.WriteObject(GENIE_OBJ_FORM,FORM_FILAMENT,0);
								HeaterCooldownInactivity(true);
							}
							else{
								setTargetHotend0(0);
								setTargetHotend1(0);
								home_axis_from_code(true, true, false);
								SERIAL_PROTOCOLPGM("Filament Removed, GOING TO CLEAN THE NOZZLE \n");
								setTargetHotend(NYLON_TEMP_HEATUP_THRESHOLD,which_extruder);
								if (which_extruder == 0) changeTool(0);
								else changeTool(1);
								
								current_position[Y_AXIS] = 10;
								current_position[X_AXIS] = 155;
								doblocking = true;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								processing = true;
								plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
								st_synchronize();
								if(processing_error)return;
								processing = false;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_NYLON_STEP0,0);
							}
							FLAG_FilamentAcceptOk = false;
						}
					}
					#pragma endregion SuccessScreens
					
					//***** Calibration XYZ *****
					#pragma region CalibrationsXYZ
					else if (Event.reportObject.index == BUTTON_X_LINE_SELECT1)
					{
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[0]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_Y,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_Y_SKIP,1);
							}
							//char buffer[30];
							float calculus = extruder_offset[X_AXIS][1] + 0.5;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store data
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_X_LINE_SELECT2)
					{
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[0]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_Y,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_Y_SKIP,1);
							}
							float calculus = extruder_offset[X_AXIS][1] + 0.4;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store data
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_X_LINE_SELECT3)
					{
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[0]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_Y,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_Y_SKIP,1);
							}
							float calculus = extruder_offset[X_AXIS][1] + 0.3;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_X_LINE_SELECT4)
					{
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[0]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_Y,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_Y_SKIP,1);
							}
							float calculus = extruder_offset[X_AXIS][1] + 0.2;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store data
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_X_LINE_SELECT5)
					{
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[0]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_Y,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_Y_SKIP,1);
							}
							float calculus = extruder_offset[X_AXIS][1] + 0.1;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_X_LINE_SELECT6)
					{
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[0]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_Y,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_Y_SKIP,1);
							}
							float calculus = extruder_offset[X_AXIS][1];
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_X_LINE_SELECT7)
					{
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[0]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_Y,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_Y_SKIP,1);
							}
							float calculus = extruder_offset[X_AXIS][1] - 0.1;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_X_LINE_SELECT8)
					{
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[0]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_Y,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_Y_SKIP,1);
							}
							float calculus = extruder_offset[X_AXIS][1] - 0.2;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_X_LINE_SELECT9)
					{
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[0]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_Y,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_Y_SKIP,1);
							}
							float calculus = extruder_offset[X_AXIS][1] - 0.3;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							//sprintf(buffer, "M218 T1 X%f",calculus); //
							//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
							//enquecommand(buffer);
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
							//enquecommand_P((PSTR("M218 T1 X-0.5")));
							Config_StoreSettings(); //Store changes
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_X_LINE_SELECT10)
					{
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[0]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_Y,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_Y_SKIP,1);
							}
							float calculus = extruder_offset[X_AXIS][1]-0.4;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_REDO_LEFT_CAB)
					{
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
							redo_source = 1;
							float calculus = extruder_offset[X_AXIS][1] + 0.5;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_REDO_RIGHT_CAB)
					{
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
							
							redo_source = 1;
							float calculus = extruder_offset[X_AXIS][1] -0.4;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					else if (Event.reportObject.index == BUTTON_REDO_X_CAB)
					{
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
							redo_source = 1;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT1)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							setTargetHotend0(0);
							setTargetHotend1(0);
							setTargetBed(0);
							manual_fine_calib_offset[1]=0.0;
							if (!Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
								processing_bed_success=true;
								
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] + 0.5;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
								
								
								}else{
								genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_SUCCESS,0);
								processing_success_first_run = true;
								FLAG_First_Start_Wizard = 888;
								Step_First_Start_Wizard = false;
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] + 0.5;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
							}
							
						}
					}
					
					else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT2)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							setTargetHotend0(0);
							setTargetHotend1(0);
							setTargetBed(0);
							manual_fine_calib_offset[1]=0.0;
							if (!Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
								processing_bed_success=true;
								
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] + 0.4;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
								
								
								}else{
								genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_SUCCESS,0);
								processing_success_first_run = true;
								FLAG_First_Start_Wizard = 888;
								Step_First_Start_Wizard = false;
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] + 0.4;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
							}
							
						}
					}
					
					else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT3)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							setTargetHotend0(0);
							setTargetHotend1(0);
							setTargetBed(0);
							manual_fine_calib_offset[1]=0.0;
							if (!Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
								processing_bed_success=true;
								
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] + 0.3;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
								
								
								}else{
								genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_SUCCESS,0);
								processing_success_first_run = true;
								FLAG_First_Start_Wizard = 888;
								Step_First_Start_Wizard = false;
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] + 0.3;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
							}
						}
					}
					
					else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT4)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							setTargetHotend0(0);
							setTargetHotend1(0);
							setTargetBed(0);
							manual_fine_calib_offset[1]=0.0;
							if (!Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
								processing_bed_success=true;
								
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] + 0.2;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
								
								
								}else{
								genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_SUCCESS,0);
								processing_success_first_run = true;
								FLAG_First_Start_Wizard = 888;
								Step_First_Start_Wizard = false;
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] + 0.2;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
							}
						}
					}
					
					else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT5)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							setTargetHotend0(0);
							setTargetHotend1(0);
							setTargetBed(0);
							manual_fine_calib_offset[1]=0.0;
							if (!Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
								processing_bed_success=true;
								
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] + 0.1;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
								
								
								}else{
								genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_SUCCESS,0);
								processing_success_first_run = true;
								FLAG_First_Start_Wizard = 888;
								Step_First_Start_Wizard = false;
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] + 0.1;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
							}
						}
					}
					
					else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT6)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							setTargetHotend0(0);
							setTargetHotend1(0);
							setTargetBed(0);
							manual_fine_calib_offset[1]=0.0;
							if (!Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
								processing_bed_success=true;
								
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1];
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
								
								
								}else{
								genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_SUCCESS,0);
								processing_success_first_run = true;
								FLAG_First_Start_Wizard = 888;
								Step_First_Start_Wizard = false;
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1];
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
							}
						}
					}
					
					else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT7)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							setTargetHotend0(0);
							setTargetHotend1(0);
							setTargetBed(0);
							manual_fine_calib_offset[1]=0.0;
							if (!Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
								processing_bed_success=true;
								
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] - 0.1;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
								
								
								}else{
								genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_SUCCESS,0);
								processing_success_first_run = true;
								FLAG_First_Start_Wizard = 888;
								Step_First_Start_Wizard = false;
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] - 0.1;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
							}
						}
					}
					
					else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT8)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							setTargetHotend0(0);
							setTargetHotend1(0);
							setTargetBed(0);
							manual_fine_calib_offset[1]=0.0;
							if (!Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
								processing_bed_success=true;
								
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] - 0.2;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
								
								
								}else{
								genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_SUCCESS,0);
								processing_success_first_run = true;
								FLAG_First_Start_Wizard = 888;
								Step_First_Start_Wizard = false;
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] - 0.2;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
							}
						}
					}
					else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT9)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							setTargetHotend0(0);
							setTargetHotend1(0);
							setTargetBed(0);
							manual_fine_calib_offset[1]=0.0;
							if (!Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
								processing_bed_success=true;
								
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] - 0.3;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
								
								
								}else{
								genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_SUCCESS,0);
								processing_success_first_run = true;
								FLAG_First_Start_Wizard = 888;
								Step_First_Start_Wizard = false;
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] - 0.3;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
							}
						}
					}
					else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT10)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							setTargetHotend0(0);
							setTargetHotend1(0);
							setTargetBed(0);
							manual_fine_calib_offset[1]=0.0;
							if (!Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
								processing_bed_success=true;
								
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] - 0.4;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
								
								
								}else{
								genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_SUCCESS,0);
								processing_success_first_run = true;
								FLAG_First_Start_Wizard = 888;
								Step_First_Start_Wizard = false;
								//char buffer[30];
								float calculus = extruder_offset[Y_AXIS][1] - 0.4;
								SERIAL_PROTOCOLPGM("Calculus:  ");
								Serial.println(calculus);
								//sprintf(buffer, "M218 T1 X%f",calculus); //
								//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
								//enquecommand(buffer);
								extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
								//enquecommand_P((PSTR("M218 T1 X-0.5")));
								Config_StoreSettings(); //Store changes
								//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
								FLAG_CalibFull = false;
								
							}
						}
					}
					else if (Event.reportObject.index == BUTTON_REDO_UP_CAB)
					{
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
							redo_source = 2;
							float calculus = extruder_offset[Y_AXIS][1] + 0.5;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					
					else if (Event.reportObject.index == BUTTON_REDO_DOWN_CAB)
					{
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
							redo_source = 2;
							float calculus = extruder_offset[Y_AXIS][1] -0.4;
							SERIAL_PROTOCOLPGM("Calculus:  ");
							Serial.println(calculus);
							extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					else if (Event.reportObject.index == BUTTON_REDO_Y_CAB)
					{
						if (millis() >= waitPeriod_button_press){
							redo_source = 2;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_Z_CALIB_Z1_Down)
					{
						if (millis() >= waitPeriod_button_press){
							
							float feedrate = homing_feedrate[Z_AXIS];
							current_position[Z_AXIS] += 0.05;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
							SERIAL_PROTOCOLPGM("Z position: ");
							Serial.println(current_position[Z_AXIS]);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_Z_CALIB_Z1_Up)
					{
						if (millis() >= waitPeriod_button_press){
							float feedrate = homing_feedrate[Z_AXIS];
							if (current_position[Z_AXIS]>-1.5) current_position[Z_AXIS] -= 0.05; //Max down is Z=-0.5
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
							SERIAL_PROTOCOLPGM("Z position: ");
							Serial.println(current_position[Z_AXIS]);
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_Z_CALIB_Z1_OK)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							processing_calib_ZL = false;
							touchscreen_update();
							genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
							SERIAL_PROTOCOLPGM("OK first Extruder! \n");
							//We have to override z_prove_offset
							zprobe_zoffset-=(current_position[Z_AXIS]); //We are putting more offset if needed
							extruder_offset[Z_AXIS][LEFT_EXTRUDER]=0.0;//It is always the reference
							/*current_position[Z_AXIS]=0;//We are setting this position as Zero
							plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);*/
							
							setTargetHotend0(print_temp_l);
							
							SERIAL_PROTOCOLPGM("Z1 Probe offset: ");
							Serial.println(zprobe_zoffset);
							Config_StoreSettings(); //Store changes
							
							
							processing_adjusting = true;
							current_position[Z_AXIS] += 2;
							plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],homing_feedrate[Z_AXIS]/60,active_extruder);
							st_synchronize();
							if(processing_error)return;
							home_axis_from_code(true,false,false);
							
							while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-10) || degBed()<(target_temperature_bed)-10){ //Waiting to heat the extruder
								
								
								manage_heater();
								touchscreen_update();
								if(processing_error)return;
							}
							processing_adjusting = false;
							//delay(6000);
							touchscreen_update();
							genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_Z_PRINT,0);
							processing_test = true;
							home_axis_from_code(false,true,true);
							if(processing_error)return;
							left_test_print_code();
							enquecommand_P(PSTR("M84"));
							if(processing_error)return;
						}
					}
					
					else if (Event.reportObject.index == BUTTON_Z_CALIB_Z2_Down )
					{
						
						float feedrate = homing_feedrate[Z_AXIS];
						current_position[Z_AXIS] += 0.05;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
						SERIAL_PROTOCOLPGM("Z position: ");
						Serial.println(current_position[Z_AXIS]);
						
					}
					
					else if (Event.reportObject.index == BUTTON_Z_CALIB_Z2_Up)
					{
						
						float feedrate = homing_feedrate[Z_AXIS];
						if (current_position[Z_AXIS]>-1.5) current_position[Z_AXIS] -= 0.05;  //Max down is Z=-0.5
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
						SERIAL_PROTOCOLPGM("Z position: ");
						Serial.println(current_position[Z_AXIS]);
						
					}
					
					else if (Event.reportObject.index == BUTTON_Z_CALIB_Z2_OK)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							processing_calib_ZR = false;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
							touchscreen_update();
							SERIAL_PROTOCOLLNPGM("OK second Extruder!");
							extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=(current_position[Z_AXIS]);//Add the difference to the current offset value
							SERIAL_PROTOCOLPGM("Z2 Offset: ");
							Serial.println(extruder_offset[Z_AXIS][RIGHT_EXTRUDER]);
							Config_StoreSettings(); //Store changes
							
							setTargetHotend1(print_temp_r);
							st_synchronize();
							if(processing_error)return;
							
							processing_adjusting = true;
							current_position[Z_AXIS] += 2;
							plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],homing_feedrate[Z_AXIS]/60,active_extruder);
							st_synchronize();
							if(processing_error)return;
							home_axis_from_code(true,false,false);
							if(processing_error)return;
							while (degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-10) || degBed()<(target_temperature_bed)-10){ //Waiting to heat the extruder
								
								manage_heater();
								touchscreen_update();
								if(processing_error)return;
							}
							//delay(6000);
							
							processing_adjusting = false;
							touchscreen_update();
							genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_Z_PRINT,0);
							processing_test = true;
							home_axis_from_code(false,true,true);
							if(processing_error)return;
							right_test_print_code();
							enquecommand_P(PSTR("M84"));
							if(processing_error)return;
						}
					}
					else if((Event.reportObject.index == BUTTON_REDO_LEFT)|| (Event.reportObject.index == BUTTON_REDO_RIGHT)){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM, FORM_REDO_Z_TEST,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_Z_LEFT_SELECT1){
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[2]=0.0;
							manual_fine_calib_offset[3]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_ZR,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_ZR_SKIP,1);
							}
							active_extruder = RIGHT_EXTRUDER;
							zprobe_zoffset+=0.05;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					else if(Event.reportObject.index == BUTTON_Z_LEFT_SELECT2){
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[2]=0.0;
							manual_fine_calib_offset[3]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_ZR,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_ZR_SKIP,1);
							}
							
							active_extruder = RIGHT_EXTRUDER;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
						
					}
					else if(Event.reportObject.index == BUTTON_Z_LEFT_SELECT3){
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[2]=0.0;
							manual_fine_calib_offset[3]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_ZR,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_ZR_SKIP,1);
							}
							
							active_extruder = RIGHT_EXTRUDER;
							zprobe_zoffset-=0.05;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_Z_LEFT_SELECT4){
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[2]=0.0;
							manual_fine_calib_offset[3]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_ZR,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_ZR_SKIP,1);
							}
							active_extruder = RIGHT_EXTRUDER;
							zprobe_zoffset-=0.1;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_Z_LEFT_SELECT5){
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[2]=0.0;
							manual_fine_calib_offset[3]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_ZR,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_ZR_SKIP,1);
							}
							
							active_extruder = RIGHT_EXTRUDER;
							zprobe_zoffset-=0.15;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					
					else if(Event.reportObject.index == BUTTON_RECALIBRATE_Z){
						if (millis() >= waitPeriod_button_press){
							redo_source = 3;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					
					else if(Event.reportObject.index == BUTTON_CLEAN_BED){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							doblocking = true;
							if(redo_source == 0){		 //redo z test print
								if (active_extruder==0){
									genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_Z_PRINT,0);
									processing_test = true;
									home_axis_from_code(true,true,true);
									if(processing_error)return;
									left_test_print_code();
									enquecommand_P(PSTR("M84"));
									if(processing_error)return;
								}
								else{
									genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_Z_PRINT,0);
									processing_test = true;
									home_axis_from_code(true,true,true);
									if(processing_error)return;
									right_test_print_code();
									enquecommand_P(PSTR("M84"));
									if(processing_error)return;
									
								}
							}
							else if(redo_source == 1){ //redo x test print
								genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_Z_PRINT,0);
								processing_test = true;
								home_axis_from_code(true,true,false);
								current_position[Z_AXIS] = 0.2;
								plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
								enquecommand_P(PSTR("G40"));
							}
							else if(redo_source == 2){ //redo y test print
								genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_Z_PRINT,0);
								processing_test = true;
								home_axis_from_code(true,true,false);
								current_position[Z_AXIS] = 0.3;
								plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
								enquecommand_P(PSTR("G41"));
								
							}
							else if(redo_source == 3){	//recalibrate
								if (active_extruder == 0){
									processing = true;
									genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
									current_position[E_AXIS]-=4;
									plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],INSERT_FAST_SPEED/60,LEFT_EXTRUDER);
									st_synchronize();
									if(processing_error)return;
									setTargetHotend0(print_temp_l);
									home_axis_from_code(true,true,true);
									if(processing_error)return;
									enquecommand_P(PSTR("G43"));
									processing = false;
								}
								else{
									processing = true;
									genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
									current_position[E_AXIS]-=4;
									plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],INSERT_FAST_SPEED/60,RIGHT_EXTRUDER);
									st_synchronize();
									if(processing_error)return;
									setTargetHotend1(print_temp_r);
									home_axis_from_code(true,true,true);
									if(processing_error)return;
									enquecommand_P(PSTR("G43"));
									processing = false;
								}
							}
						}
					}
					else if(Event.reportObject.index == BUTTON_REDO_Z_1){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							redo_source = 0;
							if (active_extruder == 0){
								zprobe_zoffset+=0.1;
								Config_StoreSettings(); //Store changes
								gcode_T0_T1_auto(0);
								st_synchronize();
								if(processing_error)return;
								
							}
							else{
								extruder_offset[Z_AXIS][RIGHT_EXTRUDER]+=0.1;
								Config_StoreSettings(); //Store changes
								gcode_T0_T1_auto(1);
								st_synchronize();
								if(processing_error)return;
								
							}
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
						}
						
					}
					else if(Event.reportObject.index == BUTTON_REDO_Z_5){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							
							redo_source = 0;
							if (active_extruder == 0){
								
								zprobe_zoffset-=0.1;
								Config_StoreSettings(); //Store changes
								gcode_T0_T1_auto(0);
								st_synchronize();
								if(processing_error)return;
							}
							else{
								extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=0.1;
								Config_StoreSettings(); //Store changes
								gcode_T0_T1_auto(1);
								st_synchronize();
								if(processing_error)return;
							}
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
						}
					}
					else if(Event.reportObject.index == BUTTON_REDO_Z){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							redo_source = 0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_BED,0);
						}
					}
					
					else if(Event.reportObject.index == BUTTON_Z_RIGHT_SELECT1){
						if (millis() >= waitPeriod_button_press){
							
							manual_fine_calib_offset[3]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_X,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_X_SKIP,1);
							}
							extruder_offset[Z_AXIS][RIGHT_EXTRUDER]+=0.05;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_Z_RIGHT_SELECT2){
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[3]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_X,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_X_SKIP,1);
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					else if(Event.reportObject.index == BUTTON_Z_RIGHT_SELECT3){
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[3]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_X,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_X_SKIP,1);
							}
							extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=0.05;
							Config_StoreSettings(); //Store changes
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_Z_RIGHT_SELECT4){
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[3]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_X,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_X_SKIP,1);
							}
							extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=0.1;
							Config_StoreSettings(); //Store changes
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_Z_RIGHT_SELECT5){
						if (millis() >= waitPeriod_button_press){
							manual_fine_calib_offset[3]=0.0;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_X,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_X_SKIP,1);
							}
							extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=0.15;
							Config_StoreSettings(); //Store changes
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_FULL_CAL_ZL_GO){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							
							doblocking=true;
							
							
							setTargetHotend1(EXTRUDER_RIGHT_CLEAN_TEMP);
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							processing = true;
							active_extruder = LEFT_EXTRUDER;
							//Wait until temperature it's okey
							
							
							home_axis_from_code(true,true,true);
							//changeTool(LEFT_EXTRUDER);
							processing = false;
							
							
							if(degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) || degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5) || degBed()<(max(bed_temp_l,bed_temp_r)-15)){
								int Tref0 = (int)degHotend0();
								int Tref1 = (int)degHotend1();
								int Trefbed = (int)degBed();
								int Tfinal0 = (int)(degTargetHotend(LEFT_EXTRUDER)-5);
								int Tfinal1 = (int)(degTargetHotend(RIGHT_EXTRUDER)-5);
								int Tfinalbed = (int)(degTargetBed()-15);
								long percentage = 0;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
								processing_adjusting =  true;
								while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) || degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5) || degBed()<(max(bed_temp_l,bed_temp_r)-15)){ //Waiting to heat the extruder
									
									manage_heater();
									touchscreen_update();
									if(processing_error)return;
									
									if (millis() >= waitPeriod_s){
										char buffer[25];
										memset(buffer, '\0', sizeof(buffer) );
										int Tinstanthot0, Tinstanthot1, Tinstantbed;
										if(Tref0 > Tfinal0){
											Tref0 = Tfinal0;
											Tinstanthot0 = Tfinal0;
											}else if(Tref0 > (int)degHotend(LEFT_EXTRUDER)){
											Tinstanthot0 = Tref0;
											}else if((int)degHotend(LEFT_EXTRUDER) > Tfinal0){
											Tinstanthot0 = Tfinal0;
											}else{
											Tinstanthot0 = (int)degHotend(LEFT_EXTRUDER);
										}
										if(Tref1 > Tfinal1){
											Tref1 = Tfinal1;
											Tinstanthot1 = Tfinal1;
											}else if(Tref1 > (int)degHotend(RIGHT_EXTRUDER)){
											Tinstanthot1 = Tref1;
											}else if((int)degHotend(RIGHT_EXTRUDER) > Tfinal1){
											Tinstanthot1 = Tfinal1;
											}else{
											Tinstanthot1 = (int)degHotend(RIGHT_EXTRUDER);
										}
										if(Trefbed > Tfinalbed){
											Trefbed = Tfinalbed;
											Tinstantbed = Tfinalbed;
											}else if(Trefbed > (int)degBed()){
											Tinstantbed = Trefbed;
											}else if((int)degBed() > Tfinalbed){
											Tinstantbed = Tfinalbed;
											}else{
											Tinstantbed = (int)degBed();
										}
										
										percentage = (long)Tfinal0+(long)Tfinal1+(long)Tfinalbed-(long)Tref0-(long)Tref1-(long)Trefbed;
										percentage= 100*((long)Tinstanthot0+(long)Tinstanthot1+(long)Tinstantbed-(long)Tref0-(long)Tref1-(long)Trefbed)/percentage;
										
										sprintf(buffer, "%ld%%", percentage);
										genie.WriteStr(STRING_ADJUSTING_TEMPERATURES,buffer);
										waitPeriod_s=2000+millis();
									}
									
									
								}
								processing_adjusting = false;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							}
							
							
							
							processing = true;
							changeTool(0);
							current_position[Z_AXIS] = 60;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 15, LEFT_EXTRUDER);//move bed
							st_synchronize();
							if(processing_error)return;
							current_position[X_AXIS] = 150; current_position[Y_AXIS] = 0;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 200, LEFT_EXTRUDER);//move first extruder
							st_synchronize();
							if(processing_error)return;
							processing = false;
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_NOZZLE_L,0);
							
							
						}
					}
					else if(Event.reportObject.index == BUTTON_FULL_CAL_ZL_SKIP && !Step_First_Start_Wizard){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_ZR,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_ZR_SKIP,1);
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_FULL_CAL_ZR_GO){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							
							doblocking=true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							processing = true;
							active_extruder = RIGHT_EXTRUDER;
							//Wait until temperature it's okey
							setTargetHotend1(EXTRUDER_RIGHT_CLEAN_TEMP);
							home_axis_from_code(true,true,true);
							//changeTool(LEFT_EXTRUDER);
							processing = false;
							
							
							//changeTool(LEFT_EXTRUDER);
							if(degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) || degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5) || degBed()<(max(bed_temp_l,bed_temp_r)-15)){
								int Tref0 = (int)degHotend0();
								int Tref1 = (int)degHotend1();
								int Trefbed = (int)degBed();
								int Tfinal0 = (int)(degTargetHotend(LEFT_EXTRUDER)-5);
								int Tfinal1 = (int)(degTargetHotend(RIGHT_EXTRUDER)-5);
								int Tfinalbed = (int)(degTargetBed()-15);
								long percentage = 0;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
								processing_adjusting =  true;
								while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) || degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5) || degBed()<(max(bed_temp_l,bed_temp_r)-15)){ //Waiting to heat the extruder
									
									manage_heater();
									touchscreen_update();
									if(processing_error)return;
									
									if (millis() >= waitPeriod_s){
										char buffer[25];
										memset(buffer, '\0', sizeof(buffer) );
										int Tinstanthot0, Tinstanthot1, Tinstantbed;
										
										if(Tref0 > Tfinal0){
											Tref0 = Tfinal0;
											Tinstanthot0 = Tfinal0;
											}else if(Tref0 > (int)degHotend(LEFT_EXTRUDER)){
											Tinstanthot0 = Tref0;
											}else if((int)degHotend(LEFT_EXTRUDER) > Tfinal0){
											Tinstanthot0 = Tfinal0;
											}else{
											Tinstanthot0 = (int)degHotend(LEFT_EXTRUDER);
										}
										if(Tref1 > Tfinal1){
											Tref1 = Tfinal1;
											Tinstanthot1 = Tfinal1;
											}else if(Tref1 > (int)degHotend(RIGHT_EXTRUDER)){
											Tinstanthot1 = Tref1;
											}else if((int)degHotend(RIGHT_EXTRUDER) > Tfinal1){
											Tinstanthot1 = Tfinal1;
											}else{
											Tinstanthot1 = (int)degHotend(RIGHT_EXTRUDER);
										}
										if(Trefbed > Tfinalbed){
											Trefbed = Tfinalbed;
											Tinstantbed = Tfinalbed;
											}else if(Trefbed > (int)degBed()){
											Tinstantbed = Trefbed;
											}else if((int)degBed() > Tfinalbed){
											Tinstantbed = Tfinalbed;
											}else{
											Tinstantbed = (int)degBed();
										}
										
										percentage = (long)Tfinal0+(long)Tfinal1+(long)Tfinalbed-(long)Tref0-(long)Tref1-(long)Trefbed;
										percentage= 100*((long)Tinstanthot0+(long)Tinstanthot1+(long)Tinstantbed-(long)Tref0-(long)Tref1-(long)Trefbed)/percentage;
										
										sprintf(buffer, "%ld%%", percentage);
										genie.WriteStr(STRING_ADJUSTING_TEMPERATURES,buffer);
										waitPeriod_s=2000+millis();
									}
									
									
								}
								processing_adjusting = false;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							}
							
							processing = true;
							changeTool(1);
							current_position[Z_AXIS] = 60;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 15, RIGHT_EXTRUDER);//move bed
							st_synchronize();
							if(processing_error)return;
							current_position[X_AXIS] = 170; current_position[Y_AXIS] = 0;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 200, RIGHT_EXTRUDER);//move first extruder
							st_synchronize();
							if(processing_error)return;
							processing = false;
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CLEAN_NOZZLE_R,0);
							
							
							
							
						}
						//home_axis_from_code(true,false,false);
						
					}
					else if(Event.reportObject.index == BUTTON_FULL_CAL_ZR_SKIP  && !Step_First_Start_Wizard){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_X,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_X_SKIP,1);
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_FULL_CAL_X_GO){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
							processing_adjusting =  true;
							doblocking=true;
							home_axis_from_code(true,true,false);
							if(processing_error)return;
							changeTool(0);
							enquecommand_P(PSTR("G40"));
							if(processing_error)return;
						}
					}
					else if(Event.reportObject.index == BUTTON_FULL_CAL_X_SKIP  && !Step_First_Start_Wizard){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_Y,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_Y_SKIP,1);
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_FULL_CAL_Y_GO){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
							processing_adjusting =  true;
							doblocking=true;
							home_axis_from_code(true,true,false);
							changeTool(0);
							enquecommand_P(PSTR("G41"));
							st_synchronize();
							if(processing_error)return;
						}
					}
					else if(Event.reportObject.index == BUTTON_FULL_CAL_Y_SKIP  && !Step_First_Start_Wizard){
						if (millis() >= waitPeriod_button_press){
							
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_SUCCESS,0);
								processing_success_first_run = true;
								FLAG_First_Start_Wizard = 888;
								Step_First_Start_Wizard = false;
								FLAG_CalibFull = false;
								}else{
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
								processing_bed_success=true;
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_CLEAN_NOZZLE_L){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							//processing =  true;
							//home_axis_from_code(true, true , false);
							enquecommand_P(PSTR("G43"));
							flag_continue_calib = false;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_CLEAN_NOZZLE_R){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							
							//home_axis_from_code(true, true , false);
							enquecommand_P(PSTR("G43"));
							flag_continue_calib = false;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
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
					Serial.println(FLAG_CalibBedDone);
					FLAG_CalibFull = true;
					
					//enquecommand_P(PSTR("T0"));
					if(!FLAG_CalibBedDone){  //Do g34
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
					
					
					#pragma region Manual Fine Calibration
					
					else if(Event.reportObject.index == BUTTON_MANUAL_FINE_CALIB_BACK){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIBRATION,0);
							offset_calib_manu[0]=0.0;
							offset_calib_manu[1]=0.0;
							offset_calib_manu[2]=0.0;
							offset_calib_manu[3]=0.0;
							calib_value_selected = 0;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_MANUAL_FINE_CALIB_MENU){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
							offset_calib_manu[0]=0.0;
							offset_calib_manu[1]=0.0;
							offset_calib_manu[2]=0.0;
							offset_calib_manu[3]=0.0;
							calib_value_selected = 0;
							screen_sdcard = false;
							surfing_utilities=false;
							surfing_temps = false;
							SERIAL_PROTOCOLPGM("Surfing 0 \n");
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_MANUAL_FINE_CALIB_OK){
						if (millis() >= waitPeriod_button_press){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_MANUAL_FINE_CALIB_SAVE,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_MANUAL_FINE_CALIB_X){
						if (millis() >= waitPeriod_button_press){
							
							calib_value_selected = 0;
							genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_MANUAL_FINE_CALIB,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_ZR,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_ZL,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_X,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_Y,0);
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_RIGHT,0);
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_LEFT,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_UP,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_DOWN,1);
							float value = 0.0;
							value = manual_fine_calib_offset[calib_value_selected] + offset_calib_manu[calib_value_selected];
							genie.WriteStr(STRING_MANUAL_FINE_CALIB,value,3);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_MANUAL_FINE_CALIB_Y){
						if (millis() >= waitPeriod_button_press){
							
							calib_value_selected = 1;
							genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_MANUAL_FINE_CALIB,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_ZR,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_ZL,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_X,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_Y,1);
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_RIGHT,1);
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_LEFT,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_UP,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_DOWN,0);
							float value = 0.0;
							value = manual_fine_calib_offset[calib_value_selected] + offset_calib_manu[calib_value_selected];
							genie.WriteStr(STRING_MANUAL_FINE_CALIB,value,3);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					else if(Event.reportObject.index == BUTTON_MANUAL_FINE_CALIB_ZL){
						if (millis() >= waitPeriod_button_press){
							
							calib_value_selected = 2;
							genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_MANUAL_FINE_CALIB,2);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_ZR,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_ZL,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_X,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_Y,0);
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_RIGHT,1);
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_LEFT,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_UP,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_DOWN,0);
							float value = 0.0;
							value = manual_fine_calib_offset[calib_value_selected] + offset_calib_manu[calib_value_selected];
							genie.WriteStr(STRING_MANUAL_FINE_CALIB,value,3);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_MANUAL_FINE_CALIB_ZR){
						if (millis() >= waitPeriod_button_press){
							calib_value_selected = 3;
							genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_MANUAL_FINE_CALIB,2);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_ZR,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_ZL,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_X,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_Y,0);
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_RIGHT,1);
							//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_LEFT,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_UP,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_MANUAL_FINE_CALIB_DOWN,0);
							float value = 0.0;
							value = manual_fine_calib_offset[calib_value_selected] + offset_calib_manu[calib_value_selected];
							genie.WriteStr(STRING_MANUAL_FINE_CALIB,value,3);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_MANUAL_FINE_CALIB_UP){
						
						float value = 0.0;
						
						if(calib_value_selected == 1 || calib_value_selected == 0){
							if(offset_calib_manu[calib_value_selected] < 1.975) offset_calib_manu[calib_value_selected] += 0.025;
						}
						else{
							if(offset_calib_manu[calib_value_selected] < 0.200) offset_calib_manu[calib_value_selected] += 0.025;
						}
						
						value = offset_calib_manu[calib_value_selected]+manual_fine_calib_offset[calib_value_selected];
						
						genie.WriteStr(STRING_MANUAL_FINE_CALIB,value,3);
					}
					else if(Event.reportObject.index == BUTTON_MANUAL_FINE_CALIB_DOWN){
						float value = 0.0;
						
						if(calib_value_selected == 1 || calib_value_selected == 0){
							if(offset_calib_manu[calib_value_selected] > -1.975) offset_calib_manu[calib_value_selected] -= 0.025;
						}
						else{
							if(offset_calib_manu[calib_value_selected] > -0.200) offset_calib_manu[calib_value_selected] -= 0.025;
						}
						value = offset_calib_manu[calib_value_selected]+manual_fine_calib_offset[calib_value_selected];
						
						genie.WriteStr(STRING_MANUAL_FINE_CALIB,value,3);
					}
					
					else if(Event.reportObject.index == BUTTON_MANUAL_FINE_CALIB_SAVE_OK){
						if (millis() >= waitPeriod_button_press){
							
							manual_fine_calib_offset[0] += offset_calib_manu[0];
							manual_fine_calib_offset[1] += offset_calib_manu[1];
							manual_fine_calib_offset[2] += offset_calib_manu[2];
							manual_fine_calib_offset[3] += offset_calib_manu[3] - offset_calib_manu[2];
							
							extruder_offset[X_AXIS][RIGHT_EXTRUDER]+= offset_calib_manu[0];
							extruder_offset[Y_AXIS][RIGHT_EXTRUDER]+= offset_calib_manu[1];
							zprobe_zoffset += offset_calib_manu[2];
							extruder_offset[Z_AXIS][RIGHT_EXTRUDER]+= offset_calib_manu[3] - offset_calib_manu[2];
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIBRATION,0);
							offset_calib_manu[0]=0.0;
							offset_calib_manu[1]=0.0;
							offset_calib_manu[2]=0.0;
							offset_calib_manu[3]=0.0;
							calib_value_selected = 0;
							Config_StoreSettings();
							Config_PrintSettings();
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if(Event.reportObject.index == BUTTON_MANUAL_FINE_CALIB_SAVE_NOT){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIBRATION,0);
							offset_calib_manu[0]=0.0;
							offset_calib_manu[1]=0.0;
							offset_calib_manu[2]=0.0;
							offset_calib_manu[3]=0.0;
							calib_value_selected = 0;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					
					#pragma endregion Manual Fine Calibration
					
					
					
					
					
					//***** Info Screens *****
					#pragma region Info Screens
					
					//Backing from INFO SCREENS
					else if (Event.reportObject.index == BACKBUTTON_CALIBRATION)
					{
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					
					//SKIP BED CALIBRATION
					else if (Event.reportObject.index == BUTTON_SKIP_BED)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							
							processing_bed = false;
							processing_bed_first = false;
							touchscreen_update();
							if (FLAG_CalibFull){
								bed_calibration_times = 0;
								
								
								active_extruder = LEFT_EXTRUDER;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								processing = true;
								setTargetHotend0(print_temp_l);
								setTargetHotend1(print_temp_r);
								setTargetBed(max(bed_temp_l,bed_temp_r));
								
								
								home_axis_from_code(true,true,false);
								st_synchronize();
								if(processing_error)return;
								enquecommand_P(PSTR("T0"));
								processing = false;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_FULL_CAL_ZL,0);
								if(Step_First_Start_Wizard){
									genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FULL_CAL_ZL_SKIP,1);
								}
								
								
								
								
							}
							else{
								genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
								processing_bed_success=true;
							}
						}
					}
					else if (Event.reportObject.index == BUTTON_ERROR_OK)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							
							if(printing_error_temps){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
								enquecommand_P(PSTR("G28 X0 Y0")); //Home X and Y
								back_home = true;
								home_made = false;
								processing_error = false;
								screen_sdcard = false;
								surfing_utilities=false;
								surfing_temps = false;
								
								card.sdprinting = false;
								card.sdispaused = false;
								
								processing = false;
								printing_error_temps = false;
							}
							else{
								processing_error = false;
								screen_sdcard = false;
								surfing_utilities=false;
								SERIAL_PROTOCOLPGM("Surfing 0 \n");
								surfing_temps = false;
								HeaterCooldownInactivity(true);
								genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN_SCREEN, 0);
							}
							doblocking = false;
						}
					}
					
					#pragma endregion Info Screens
					#pragma region Setup Assistant
					else if (Event.reportObject.index == BUTTON_FIRST_RUN_WIZARD_YES)
					{
						if (millis() >= waitPeriod_button_press){
							
							surfing_utilities = true;
							Step_First_Start_Wizard = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_STEP_1,0);
							Config_ResetDefault();
							Config_StoreSettings();
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_FIRST_RUN_WIZARD_SKIP)
					{
						if (millis() >= waitPeriod_button_press){
							
							FLAG_First_Start_Wizard = 888;
							Step_First_Start_Wizard = false;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
							Config_StoreSettings();
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_FIRST_RUN_WIZARD_STEP_NEXT_1)
					{
						if (millis() >= waitPeriod_button_press){
							
							which_extruder = 0;
							filament_mode = 'I';
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 1);
							
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SELECT_EXTRUDER_MENU,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_FILAMENT_BACK,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PLA, 1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_ABS, 1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_PVA, 1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUST, 1);
							genie.WriteObject(GENIE_OBJ_FORM, FORM_SELECT_EXTRUDER, 1);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					else if (Event.reportObject.index == BUTTON_FIRST_RUN_WIZARD_STEP_NEXT_2)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
							
							SERIAL_PROTOCOLPGM("INFO: BED CALIB - ");
							Serial.println(FLAG_CalibBedDone);
							FLAG_CalibFull = true;
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							processing = true;
							doblocking = true;
							home_axis_from_code(true,true,true);
							st_synchronize();
							if(processing_error)return;
							enquecommand_P(PSTR("G34"));	//Start BED Calibration Wizard
							changeTool(0);
						}
					}
					else if (Event.reportObject.index == BUTTON_SETUP_ASSISTANT)
					{
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_INIT,0);
							int j = 0;
							static uint32_t waitPeriod = millis(); //Processing back home
							while ( j<GIF_FRAMES_INIT_FIRST_RUN){
								if (millis() >= waitPeriod){
									
									genie.WriteObject(GENIE_OBJ_VIDEO,GIF_FIRST_RUN_WIZARD_INIT,j);
									j+=1;
									waitPeriod = GIF_FRAMERATE+millis();	//Every 5s
								}
								
								
								
							}
							genie.WriteObject(GENIE_OBJ_FORM,FORN_FIRST_RUN_WIZARD_YESNOT,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					#pragma endregion Setup Assistant
					
				}// else
			}
			
			//USERBUTTONS------------------------------------------------------

			
			//FORMS--------------------------------------------------------
			if (Event.reportObject.object == GENIE_OBJ_FORM)
			{
				if (Event.reportObject.index == FORM_SDFILES)
				{
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FOLDER_BACK,0);
						genie.WriteObject(GENIE_OBJ_USERIMAGES, USERIMAGE_FOLDER_FILE,0);
						screen_sdcard = true;
						FLAG_ListFilesInit = true;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				
				else if (Event.reportObject.index == FORM_PRINTING)
				{
					if (millis() >= waitPeriod_button_press){
						
						is_on_printing_screen = true;
						surfing_utilities = false;
						genie.WriteStr(STRINGS_PRINTING_GCODE,namefilegcode);
						FLAG_DataRefresh = true;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				else if (Event.reportObject.index == FORM_MAIN_SCREEN)
				{
					if (millis() >= waitPeriod_button_press){
						
						screen_sdcard = false;
						surfing_utilities=false;
						surfing_temps = false;
						SERIAL_PROTOCOLPGM("Surfing 0 \n");
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				
				else if (Event.reportObject.index == FORM_UTILITIES)
				{
					if (millis() >= waitPeriod_button_press){
						
						surfing_utilities=true;
						SERIAL_PROTOCOLPGM("Surfing 1 \n");
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				else if (Event.reportObject.index == FORM_TEMP_MENU){
					if (millis() >= waitPeriod_button_press){
						
						surfing_temps = true;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				
				else if (Event.reportObject.index == FORM_PURGE)
				{
					if (millis() >= waitPeriod_button_press){
						
						SERIAL_PROTOCOLPGM("Enter in purge mode \n");
						if(purge_extruder_selected == 0) {
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,0);
						}
						else {
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_LEFT,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_RIGHT,1);
						}
						
						char buffer[256];
						sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
						genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);	Serial.println(buffer);
						sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
						genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);	Serial.println(buffer);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
				else if (Event.reportObject.index == FORM_PREHEAT_SETTINGS)
				{
					
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
				/*else if (Event.reportObject.index == FORM_INFO_SIGMA)
				{
				
				char buffer[256];
				
				sprintf(buffer, "%d h",log_hours_print);
				//Serial.println(buffer);
				genie.WriteStr(STRING_INFO_PRINTINGTIME,buffer);
				
				}*/
				else if (Event.reportObject.index == FORM_INFO_UI)
				{
					if (millis() >= waitPeriod_button_press){
						
						char buffer[256];
						sprintf(buffer, "%s%s",VERSION_STRING,BUILD_DATE);
						genie.WriteStr(STRING_INFO_UI_VERSION,buffer);
						if(UI_SerialID0 || UI_SerialID0 || UI_SerialID0){
							sprintf(buffer, "%03d.%03d%03d.%04d",UI_SerialID0, (int)(UI_SerialID1/1000),(int)(UI_SerialID1%1000), UI_SerialID2);
							//sprintf(buffer, "%03d.%03d%03d.%04d",1020, 1151,1021, 10002);
							genie.WriteStr(STRING_INFO_UI_SerialID,buffer);
							}else{
							genie.WriteStr(STRING_INFO_UI_SerialID,UI_SerialID);
						}
						sprintf(buffer, "%d h",log_hours_print);
						//Serial.println(buffer);
						genie.WriteStr(STRING_INFO_PRINTINGTIME,buffer);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
				}
			}
		}
	}



	char* prepareString(char* text, int len){
		SERIAL_PROTOCOLPGM("Text: ");
		Serial.println(text);
		if (String(text).length()>12){
			char buffer[len+1];
			for (int i = 0; i<len ; i++)
			{
				buffer[i]=card.longFilename[i];
			}
			buffer[len]='\0';
			SERIAL_PROTOCOLPGM("Buffer temp: ");
			Serial.println(buffer);
			char* buffer2 = strcat(buffer,"...\0");
			SERIAL_PROTOCOLPGM("Buffer returned: ");
			Serial.println(buffer2);
			return buffer2;
			//buffer2 = strcat(buffer,"...\0");
			}else{
			char* buffer2 = text;
			SERIAL_PROTOCOLPGM("Buffer returned: ");
			Serial.println(buffer2);
			return buffer2;
		}
	}
	inline void setfilenames(int jint){
		int count = 22;
		char buffer[count+3];
		int x = 0;
		memset( buffer, '\0', sizeof(buffer));
		if (String(card.longFilename).length() > count){
			for (int i = 0; i<count ; i++)
			{
				if (card.longFilename[i] == '.') i = count +10; //go out of the for
				else if(i == 0) buffer[i]=card.longFilename[x];
				else {
					buffer[i]=card.longFilename[x];
				}
				x++;
				Serial.print(i);
			}
			buffer[count]='\0';
			char* buffer2 = strcat(buffer,"...\0");
			genie.WriteStr(stringfilename[jint],buffer2);//Printing form
			genie.WriteStr(stringfiledur[jint],listsd.commandline2);//Printing form
			memset( buffer2, '\0', sizeof(buffer2));
		}
		else {
			for (int i = 0; i<String(card.longFilename).length(); i++)	{
				if (card.longFilename[i] == '.') i = String(card.longFilename).length() +10; //go out of the for
				else if(i == 0) buffer[i]=card.longFilename[x];
				else {
					buffer[i]=card.longFilename[x];
				}
				x++;
				Serial.print(i);
			}
			//buffer[count]='\0';
			genie.WriteStr(stringfilename[jint],buffer);//Printing form
			genie.WriteStr(stringfiledur[jint],listsd.commandline2);//Printing form
			//Is a file
			//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
		}
		Serial.println(buffer);
		
		
	}
	inline void ListFilesParsingProcedure(int vecto, int jint){
		char Workdir[20];
		card.getfilename(vecto);
		Serial.println(card.longFilename);
		if (card.filenameIsDir)
		{
			genie.WriteObject(GENIE_OBJ_USERBUTTON,buttonsdselected[jint],1);
			setfoldernames(jint);
			
			if(card.chdir(card.filename)!= -1){
				uint16_t NUMitems = card.getnrfilenames();
				card.updir();
				card.getWorkDirName();
				memset(Workdir, '\0', sizeof(Workdir));
				sprintf_P(Workdir, PSTR("%d items"),NUMitems);
				genie.WriteStr(stringfiledur[jint],Workdir);//Printing form
			}
			else{
				genie.WriteStr(stringfiledur[jint],"       ");//Printing form
			}
		}
		else{
			genie.WriteObject(GENIE_OBJ_USERBUTTON,buttonsdselected[jint],0);
			listsd.get_lineduration(true, NULL);
			if(listsd.get_minutes() == -1){
				sprintf_P(listsd.commandline2, "");
			}
			else{
				sprintf(listsd.commandline2, "%4d:%.2dh / %dg",listsd.get_hours(), listsd.get_minutes(),listsd.get_filgramos1());
			}
			//Serial.println(listsd.commandline);
			setfilenames(jint);
			
		}
	}
	inline void ListFilesUpfunc(){
		
		
		int vecto = 0;
		int jint = 0;
		
		
		if (card.cardOK){
			uint16_t fileCnt = card.getnrfilenames();
			//Declare filepointer
			card.getWorkDirName();
			
			if(fileCnt > SDFILES_LIST_NUM){
				
				if (filepointer == ((fileCnt-1)/SDFILES_LIST_NUM)*SDFILES_LIST_NUM )
				{
					filepointer=0; //First SD file
				}
				else
				{
					filepointer+=SDFILES_LIST_NUM;
				}
				genie.WriteObject(GENIE_OBJ_VIDEO, GIF_SCROLL_BAR,	filepointer*40/(((fileCnt-1)/SDFILES_LIST_NUM)*SDFILES_LIST_NUM));
				
				
				
				
				while(jint < SDFILES_LIST_NUM){
					if(fileCnt > filepointer +  jint){
						
						vecto = filepointer + jint;						
						
						ListFilesParsingProcedure(vecto, jint);
						
					}
					else{
						genie.WriteObject(GENIE_OBJ_USERBUTTON,buttonsdselected[jint],0);
						genie.WriteStr(stringfilename[jint],"        ");//Printing form
						genie.WriteStr(stringfiledur[jint],"           ");//Printing form
						
					}
					jint++;
				}
				
			}
			
			
		}
		
		
		FLAG_FilesUpDown= true;
		
		memset(listsd.commandline2, '\0', sizeof(listsd.commandline2) );
	}
	inline void ListFilesDownfunc(){
				
		int vecto = 0;
		int jint = 0;
		
		if (card.cardOK){
			uint16_t fileCnt = card.getnrfilenames();
			//Declare filepointer
			card.getWorkDirName();
			
			if(fileCnt > SDFILES_LIST_NUM){
				if (filepointer == 0)
				{
					filepointer=((fileCnt-1)/SDFILES_LIST_NUM)*SDFILES_LIST_NUM;
				}
				else{
					filepointer-=SDFILES_LIST_NUM;
				}
				genie.WriteObject(GENIE_OBJ_VIDEO, GIF_SCROLL_BAR,	filepointer*40/(((fileCnt-1)/SDFILES_LIST_NUM)*SDFILES_LIST_NUM));
				
				
				
				
				while(jint < SDFILES_LIST_NUM){
					if(fileCnt > filepointer +  jint){
												
						vecto = filepointer + jint;						
						
						ListFilesParsingProcedure(vecto, jint);
						
					}
					else{
						genie.WriteObject(GENIE_OBJ_USERBUTTON,buttonsdselected[jint],0);
						genie.WriteStr(stringfilename[jint],"        ");//Printing form
						genie.WriteStr(stringfiledur[jint],"           ");//Printing form
						
					}
					
					jint++;
				}
				
			}
			
		}
		FLAG_FilesUpDown= true;

		memset(listsd.commandline2, '\0', sizeof(listsd.commandline2) );
	}
	inline void ListFileListINITSD(){
		genie.WriteObject(GENIE_OBJ_VIDEO, GIF_SCROLL_BAR,0);
		SERIAL_PROTOCOLLNPGM("Form 2!");
		////Check sdcardFiles
		filepointer = 0;
		int vecto = 0;
		int jint = 0;
		card.initsd();
		workDir_vector_lenght = 0;
		if (card.cardOK){
			
			uint16_t fileCnt = card.getnrfilenames();
			//Declare filepointer
			card.getWorkDirName();
			
			while(jint < SDFILES_LIST_NUM){
				
				if(jint < fileCnt){
					
					
					vecto = filepointer + jint;
					ListFilesParsingProcedure(vecto, jint);
					
				}
				else{
					genie.WriteObject(GENIE_OBJ_USERBUTTON,buttonsdselected[jint],0);
					genie.WriteStr(stringfilename[jint],"        ");//Printing form
					genie.WriteStr(stringfiledur[jint],"           ");//Printing form
					
				}
				
				jint++;
			}
			
			
		}
		else{
			#ifndef ErroWindowEnable
			genie.WriteObject(GENIE_OBJ_FORM, FORM_INSERT_SD_CARD, 0);
			screen_sdcard = true;
			#else
			genie.WriteObject(GENIE_OBJ_FORM, FORM_ERROR_SCREEN, 0);
			genie.WriteStr(STRING_ERROR_MESSAGE,"ERROR: INSERT SDCARD");//Printing form
			processing_error =  true;
			screen_sdcard = true;
			#endif
		}
		memset(listsd.commandline2, '\0', sizeof(listsd.commandline2) );

	}
	inline void ListFileSelect_find(){
		card.getfilename(filepointer);
		if (!card.filenameIsDir){
			folder_navigation_register(true);
			genie.WriteObject(GENIE_OBJ_FORM, FORM_SDFILE_CONFIRMATION,0);
			listsd.get_lineduration(true, NULL);
			if(listsd.get_minutes() == -1){
				sprintf_P(listsd.commandline2, PSTR(""));
			}
			else{
				sprintf_P(listsd.commandline2, PSTR("%4d:%.2dh / %dg"),listsd.get_hours(), listsd.get_minutes(),listsd.get_filgramos1());
			}
			setfilenames(6);
			
		}
		else{
			if (card.chdir(card.filename)!=-1){
				folder_navigation_register(true);
				FLAG_ListFileEnterBackFolder = true;
				genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FOLDER_BACK,1);
				genie.WriteObject(GENIE_OBJ_USERIMAGES, USERIMAGE_FOLDER_FILE,1);
			}
			
			
		}
	}
	inline void ListFileSelect0(){
		if(card.cardOK)
		{
			FLAG_FilesUpDown = false;
			ListFileSelect_find();
			FLAG_FilesUpDown = true;
		}
	}
	inline void ListFileSelect1(){
		if(card.cardOK)
		{
			FLAG_FilesUpDown = false;
			uint16_t fileCnt = card.getnrfilenames();
			if(fileCnt > filepointer +  1){
				if (filepointer == card.getnrfilenames()-1)
				{
					filepointer=0; //First SD file
					}else{
					filepointer++;
				}
				ListFileSelect_find();
			}
			FLAG_FilesUpDown = true;
		}
	}
	inline void ListFileSelect2(){
		if(card.cardOK)
		{
			FLAG_FilesUpDown = false;
			uint16_t fileCnt = card.getnrfilenames();
			if(fileCnt >filepointer +   2){
				if (filepointer == card.getnrfilenames()-1)
				{
					filepointer=1; //First SD file
				}
				else if (filepointer == card.getnrfilenames()-2 )
				{
					filepointer=0; //First SD file
				}
				else{
					filepointer+=2;
				}
				ListFileSelect_find();
			}
			FLAG_FilesUpDown = true;
		}
	}
	inline void ListFileSelect3(){
		if(card.cardOK)
		{
			FLAG_FilesUpDown = false;
			uint16_t fileCnt = card.getnrfilenames();
			if(fileCnt >filepointer +   3){
				if (filepointer == card.getnrfilenames()-1)
				{
					filepointer=2; //First SD file
				}
				else if (filepointer == card.getnrfilenames()-2 )
				{
					filepointer=1; //First SD file
				}
				else if (filepointer == card.getnrfilenames()-3 )
				{
					filepointer=0; //First SD file
				}
				else{
					filepointer+=3;
				}
				ListFileSelect_find();
			}
			
			FLAG_FilesUpDown = true;
		}
	}
	inline void ListFileSelect4(){
		if(card.cardOK)
		{
			FLAG_FilesUpDown = false;
			uint16_t fileCnt = card.getnrfilenames();
			if(fileCnt >filepointer +   4){
				if (filepointer == card.getnrfilenames()-1)
				{
					filepointer=3; //First SD file
				}
				else if (filepointer == card.getnrfilenames()-2 )
				{
					filepointer=2; //First SD file
				}
				else if (filepointer == card.getnrfilenames()-3 )
				{
					filepointer=1; //First SD file
				}
				else if (filepointer == card.getnrfilenames()-4 )
				{
					filepointer=0; //First SD file
				}
				else{
					filepointer+=4;
				}
				ListFileSelect_find();
			}
			FLAG_FilesUpDown = true;
		}
	}
	inline void ListFileSelect5(){
		if(card.cardOK)
		{
			FLAG_FilesUpDown = false;
			uint16_t fileCnt = card.getnrfilenames();
			if(fileCnt > filepointer + 5){
				if (filepointer == card.getnrfilenames()-1)
				{
					filepointer=4; //First SD file
				}
				else if (filepointer == card.getnrfilenames()-2 )
				{
					filepointer=3; //First SD file
				}
				else if (filepointer == card.getnrfilenames()-3 )
				{
					filepointer=2; //First SD file
				}
				else if (filepointer == card.getnrfilenames()-4 )
				{
					filepointer=1; //First SD file
				}
				else if (filepointer == card.getnrfilenames()-5 )
				{
					filepointer=0; //First SD file
				}
				else{
					filepointer+=5;
				}
				ListFileSelect_find();
			}
			FLAG_FilesUpDown = true;
		}
	}
	inline void ListFileListENTERBACKFORLDERSD(){
		genie.WriteObject(GENIE_OBJ_VIDEO, GIF_SCROLL_BAR,0);
		filepointer = 0;
		int vecto = 0;
		int jint = 0;
		uint16_t fileCnt = card.getnrfilenames();
		//Declare filepointer
		card.getWorkDirName();
		//Text index starts at 0
		//for(jint = 0; jint < 4; jint++){//
		//genie.WriteStr(STRING_FOLDER_NAME,card.getWorkDirName());//Printing form
		
		
		if (card.cardOK){
			
			uint16_t fileCnt = card.getnrfilenames();
			//Declare filepointer
			card.getWorkDirName();
			
			while(jint < SDFILES_LIST_NUM){
				
				if(jint < fileCnt){
					
					vecto = filepointer + jint;
					
					ListFilesParsingProcedure(vecto, jint);
					
				}
				else{
					genie.WriteObject(GENIE_OBJ_USERBUTTON,buttonsdselected[jint],0);
					genie.WriteStr(stringfilename[jint],"        ");//Printing form
					genie.WriteStr(stringfiledur[jint],"           ");//Printing form
					
				}
				jint++;
			}
			
			
		}
		else{
			#ifndef ErroWindowEnable
			genie.WriteObject(GENIE_OBJ_FORM, FORM_INSERT_SD_CARD, 0);
			screen_sdcard = true;
			#else
			genie.WriteObject(GENIE_OBJ_FORM, FORM_ERROR_SCREEN, 0);
			genie.WriteStr(STRING_ERROR_MESSAGE,"ERROR: INSERT SDCARD");//Printing form
			processing_error =  true;
			screen_sdcard = true;
			#endif
		}
		memset(listsd.commandline2, '\0', sizeof(listsd.commandline2) );
		
	}
	inline void setfoldernames(int jint){
		int count = 22;
		char buffer[count+3];
		int x = 0;
		memset( buffer, '\0', sizeof(buffer));
		if (String(card.longFilename).length() == 0){
			genie.WriteStr(stringfilename[jint],card.filename);//Printing form
		}
		else if (String(card.longFilename).length() > count){
			for (int i = 0; i<count ; i++)
			{
				if (card.longFilename[i] == '.') i = count +10; //go out of the for
				else if(i == 0) buffer[i]=card.longFilename[x];
				else {
					buffer[i]=card.longFilename[x];
				}
				x++;
				Serial.print(i);
			}
			buffer[count]='\0';
			char* buffer2 = strcat(buffer,"...\0");
			
			genie.WriteStr(stringfilename[jint],buffer2);//Printing form
			memset( buffer2, '\0', sizeof(buffer2));
		}
		else {
			for (int i = 0; i<String(card.longFilename).length(); i++)	{
				if (card.longFilename[i] == '.') i = String(card.longFilename).length() +10; //go out of the for
				else if(i == 0) buffer[i]=card.longFilename[x];
				else {
					buffer[i]=card.longFilename[x];
				}
				x++;
				Serial.print(i);
			}
			//buffer[count]='\0';
			genie.WriteStr(stringfilename[jint],buffer);//Printing form
			//Is a file
			//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
		}
		Serial.println(buffer);
		
		
	}
	inline void insertmetod(){
		processing = true;
		if(!card.sdispaused){
			//genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
			doblocking = true;
			if(Step_First_Start_Wizard){
				home_axis_from_code(false,true,false);
				home_axis_from_code(true,false,true);
				}else{
				if (!home_made) home_axis_from_code(true,true,true);
			}
			int feedrate;
			if (!FLAG_FilamentHome){
				
				
				home_axis_from_code(true,true,false);
				
				FLAG_FilamentHome=true;
			}
			
			
			current_position[Z_AXIS]=Z_MAX_POS-15;
			feedrate=homing_feedrate[Z_AXIS];
			plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate*2/60, active_extruder); //check speed
			st_synchronize();
			if(processing_error)return;
			/****************************************************/
		}
		genie.WriteObject(GENIE_OBJ_FORM,FORM_INSERT_FILAMENT_TOP,0);
		processing = false;
	}
	inline void folder_navigation_register(bool upchdir){

		if(upchdir){
			workDir_vector[workDir_vector_lenght] = filepointer;
			workDir_vector_lenght++;
			
			}else{
			workDir_vector_lenght--;

		}
	}

	#endif /* INCLUDE */

	#endif