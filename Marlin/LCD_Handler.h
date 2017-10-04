/*
LCD_Handler.h - A place to hold all interactions between LCD and printer. It is called from Marlin_main.cpp when using genie.DoEvents().
Last Update: 20/06/2017
Author: Alejandro Garcia (S3mt0x)
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
//inline void ListFileSelect5();
inline void setfoldernames(int jint);
inline void folder_navigation_register(bool upchdir);
void myGenieEventHandler();

bool flag_utilities_maintenance_nyloncleaning = false;
bool flag_utilities_maintenance_autotune = false;
bool flag_utilities_filament_home = false;
bool flag_utilities_filament_acceptok = false;
bool flag_utilities_filament_purgeselect0 = false;
bool flag_utilities_filament_purgeselect1 = false;
bool flag_utilities_filament_purgeload = false;
bool flag_utilities_filament_purgeunload = false;
bool flag_utilities_calibration_calibfull = false;
bool flag_utilities_calibration_calibfull_skipZcalib = false;
bool flag_utilities_calibration_calibbeddone = false;
bool flag_utilities_calibration_bedcomensationmode = false;
bool flag_temp_gifhotent0 = false;
bool flag_temp_gifhotent1 = false;
bool flag_temp_gifbed = false;
bool flag_sdprinting_pausepause = false;
bool flag_sdprinting_printstop = false;
bool flag_sdprinting_printsavejob = false;
bool flag_sdprinting_printpause = false;
bool flag_sdprinting_printresume = false;
bool flag_sdprinting_pauseresume = false;
bool flag_sdprinting_settings = false;
bool flag_sdprinting_showdata = false;
bool flag_sdprinting_dararefresh =  false;
bool flag_sdprinting_printsavejobcommand = false;
bool flag_sdlist_filesupdown = true;
bool flag_sdlist_select0 = false;
bool flag_sdlist_select1 = false;
bool flag_sdlist_select2 = false;
bool flag_sdlist_select3 = false;
bool flag_sdlist_select4 = false;
bool flag_sdlist_select5 = false;
bool flag_sdlist_godown = false;
bool flag_sdlist_goup = false;
bool flag_sdlist_goinit = false;
bool flag_sdlist_gofolderback = false;
bool flag_maintenance_zdjust100up = false;
bool flag_maintenance_zdjust10up = false;
bool flag_maintenance_zdjust100down = false;
bool flag_maintenance_zdjust10down = false;
int raft_advise_accept_cancel = -1;//0 cancel ; 1 accept
int Temp_ChangeFilament_Saved = 0;
int Tref1 = 0;
int Tfinal1 = 0;
int  print_setting_tool = 2;
inline void Calib_check_temps(void);
inline void Z_compensation_decisor(void);
inline void Full_calibration_ZL_set(float offset);
inline void Full_calibration_ZR_set(float offset);
inline void Full_calibration_X_set(float offset);
inline void Full_calibration_Y_set(float offset);
inline void Z_compensation_coolingdown(void);

// Bed compensation
inline void Bed_Compensation_Set_Lines(int jint);
inline void Bed_Compensation_Redo_Lines(int jint);
int Bed_compensation_redo_offset = 0;
int8_t Bed_Compensation_Lines_Selected[3] = {0,0,0};
uint8_t Bed_Compensation_state = 0;// state 0: First Bed Calib, state 1: ZL Calib, state 2: Bed Compensation Back, state 3: Bed Compensation Front Left, state 4: Bed Compensation Front Right

// end Bed compensation

float offset_calib_manu[4] = {0.0,0.0,0.0,0.0};
unsigned int calib_value_selected;
float offset_x_calib = 0;
float offset_y_calib = 0;
int  previous_state = FORM_MAIN;
int custom_insert_temp = 210;
int custom_remove_temp = 210;
int custom_print_temp = 210;
int custom_bed_temp = 40;
unsigned int buttonsdselected[6] = {BUTTON_SDLIST_SELECT0, BUTTON_SDLIST_SELECT1, BUTTON_SDLIST_SELECT2, BUTTON_SDLIST_SELECT3, BUTTON_SDLIST_SELECT4, 255};
unsigned int stringfilename[8] = {STRING_SDLIST_NAMEFILE0, STRING_SDLIST_NAMEFILE1, STRING_SDLIST_NAMEFILE2, STRING_SDLIST_NAMEFILE3, STRING_SDLIST_NAMEFILE4, 255,STRING_SDLIST_CONFIRMATION_NAMEFILE,STRING_RECOVERY_PRINT_ASK};
unsigned int stringfiledur[8] = {STRING_SDLIST_DURFILE0, STRING_SDLIST_DURFILE1, STRING_SDLIST_DURFILE2, STRING_SDLIST_DURFILE3, STRING_SDLIST_DURFILE4,255, STRING_SDLIST_CONFIRMATION_DURFILE, STRING_RECOVERY_PRINT_ASK_DUR};


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
				
				switch(Event.reportObject.index) {
					#pragma region Printing_screen
					case BUTTON_SDPRINTING_SETTINGS:
					if (millis() >= waitPeriod_button_press){
						
						flag_sdprinting_settings = true;
						
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_SDPRINTTING_SETINGS_BACK:
					if (millis() >= waitPeriod_button_press){
						
						flag_sdprinting_showdata = true;
						
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_SDPRINTING_PAUSE_SETTINGS:
					
					if (millis() >= waitPeriod_button_press){
						
						flag_sdprinting_settings = true;
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_SDPRINTING_PAUSE_BACKSTATE, 1);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					
					break;
					
					case BUTTON_SDPRINTING_PAUSE_BACKSTATE:
					
					if (millis() >= waitPeriod_button_press){
						
						if(screen_printing_pause_form ==screen_printing_pause_form2){
							screen_printing_pause_form = screen_printing_pause_form1;
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_SDPRINTING_PAUSE_STOP, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_SDPRINTING_PAUSE_RESUME, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_SDPRINTING_PAUSE_SETTINGS, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_SDPRINTING_PAUSE_BACKSTATE, 0);
							surfing_utilities = false;
						}
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_SDPRINTING_STOPPRINT_YES:
					
					if (millis() >= waitPeriod_button_press){
						
						is_on_printing_screen=false;
						
						card.closefile();
						flag_sdprinting_printstop = true;
						cancel_heatup = true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						gif_processing_state = PROCESSING_DEFAULT;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					
					break;
					
					case BUTTON_SDPRINTING_STOPPRINT_NO:
					
					if (millis() >= waitPeriod_button_press){
						
						if(screen_printing_pause_form == screen_printing_pause_form0){
							
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_SDPRINTING,0);
							is_on_printing_screen = true;
							genie.WriteStr(STRING_SDPRINTING_GCODE,namefilegcode);
							flag_sdprinting_dararefresh = true;
							surfing_utilities = false;
							}else{
							genie.WriteObject(GENIE_OBJ_FORM,FORM_SDPRINTING_PAUSE,0);
							is_on_printing_screen = true;
							genie.WriteStr(STRING_SDPRINTING_PAUSE_GCODE,namefilegcode);
							flag_sdprinting_dararefresh = true;
							surfing_utilities = false;
						}
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_SDPRINTING_STOPPRINT_SAVE:
					
					if(!waiting_temps && !card.sdispaused){
						
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_SDPRINTING_SAVEPRINT_SURE,0);
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					break;
					
					case BUTTON_SDPRINTING_SAVEPRINT_SURE_NOT:
					
					if(!waiting_temps){
						
						if (millis() >= waitPeriod_button_press){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_SDPRINTING_STOPPRINT,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					break;
					
					case BUTTON_SDPRINTING_SAVEPRINT_SURE_OK:
					
					if(!waiting_temps){
						
						if (millis() >= waitPeriod_button_press){
							
							
							if(screen_printing_pause_form == screen_printing_pause_form0){
								
								
								is_on_printing_screen=false;
								
								card.sdprinting = false;
								card.sdispaused = true;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
								gif_processing_state = PROCESSING_DEFAULT;
								flag_sdprinting_printsavejobcommand = true;
								
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						
					}
					
					break;
					
					case BUTTON_SDPRINTING_STOP:
					
					if((screen_printing_pause_form == screen_printing_pause_form0)){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_SDPRINTING_STOPPRINT,0);
							
							is_on_printing_screen=false;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					
					break;
					
					case BUTTON_SDPRINTING_PAUSE_STOP:
					
					if((screen_printing_pause_form == screen_printing_pause_form1)){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_SDPRINTING_STOPPRINT,0);
							
							is_on_printing_screen=false;
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						}else if(screen_printing_pause_form == screen_printing_pause_form2){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_UNLOAD,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_UNLOAD_MENU,1);
							filament_mode = 'R';
							surfing_utilities = true;
							is_on_printing_screen=false;
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					break;
					
					case BUTTON_SDPRINTING_PAUSE:
					
					if(card.sdprinting && screen_printing_pause_form == screen_printing_pause_form0){
						if (millis() >= waitPeriod_button_press){
							
							flag_sdprinting_printpause = true;
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					
					break;
					
					case BUTTON_SDPRINTING_PAUSE_RESUME:
					
					if(card.sdispaused){
						if(screen_printing_pause_form == screen_printing_pause_form1){
							if (millis() >= waitPeriod_button_press){
								
								flag_sdprinting_printresume = true;
								waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							}
						}
						else if(screen_printing_pause_form == screen_printing_pause_form2){
							if (millis() >= waitPeriod_button_press){
								is_purging_filament = true;
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_LOAD,0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_UNLOAD,0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_TEMPUP,0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_TEMPDOWN,0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_MENU,1);
								genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_PURGE,0);
								surfing_utilities = true;
								purge_extruder_selected = -1;
								SERIAL_PROTOCOLPGM("Purge Filament \n");
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,0);
								
								char buffer[256];
								sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
								genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_LEFTTARGET,buffer);	Serial.println(buffer);
								sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
								genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_RIGHTTARGET,buffer);	Serial.println(buffer);
								sprintf_P(buffer, PSTR("%3d %cC"),0,0x00B0);
								genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);	Serial.println(buffer);
								
								is_on_printing_screen = false;
								waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							}
						}
					}
					break;
					
					#pragma endregion Printing_screen
					
					//*****Printing Settings*****
					#pragma region Printing Settings
					
					case  BUTTON_SDPRINTTING_SETINGS_SPEED_UP:
					screen_change_speedup = true;
					break;
					
					case BUTTON_SDPRINTTING_SETINGS_SPEED_DOWN:
					screen_change_speeddown = true;
					break;
					
					case BUTTON_SDPRINTTING_SETINGS_LEFT_UP:
					screen_change_nozz1up = true;
					break;
					
					case BUTTON_SDPRINTTING_SETINGS_LEFT_DOWN:
					screen_change_nozz1down = true;
					break;
					
					case BUTTON_SDPRINTTING_SETINGS_RIGHT_UP:
					screen_change_nozz2up = true;
					break;
					
					case BUTTON_SDPRINTTING_SETINGS_RIGHT_DOWN:
					screen_change_nozz2down = true;
					break;
					
					case BUTTON_SDPRINTTING_SETINGS_BED_UP:
					screen_change_bedup = true;
					break;
					
					case BUTTON_SDPRINTTING_SETINGS_BED_DOWN:
					screen_change_beddown = true;
					break;
					
					
					#pragma endregion Printing Settings
					
					#pragma region Insert_Remove_Fil
					
					case BUTTON_UTILITIES_FILAMENT_UNLOAD_BACK:
					
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_SDPRINTING_PAUSE,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_UNLOAD_MENU,0);
						is_on_printing_screen = true;
						surfing_utilities = false;
						genie.WriteStr(STRING_SDPRINTING_PAUSE_GCODE,namefilegcode);
						flag_sdprinting_dararefresh = true;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					
					break;
					
					
					case BUTTON_UTILITIES_FILAMENT_LOAD:
					case BUTTON_UTILITIES_FILAMENT_UNLOAD:
					if (millis() >= waitPeriod_button_press){
						
						if (Event.reportObject.index == BUTTON_UTILITIES_FILAMENT_LOAD) filament_mode = 'I'; //Insert Mode
						else if (Event.reportObject.index == BUTTON_UTILITIES_FILAMENT_UNLOAD) filament_mode = 'R'; //Remove Mode
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT1, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0, 0);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_LOAD,0);
						which_extruder = -1;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					
					break;
					case BUTTON_UTILITIES_FILAMENT_UNLOAD_SELECTLEFT:
					case BUTTON_UTILITIES_FILAMENT_UNLOAD_SELECTRIGHT:
					
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						if (Event.reportObject.index == BUTTON_UTILITIES_FILAMENT_UNLOAD_SELECTLEFT) //Left Nozzle
						{
							
							which_extruder=0;
							
						}
						else //Right Nozzle
						{
							
							which_extruder=1;
						}
						Temp_ChangeFilament_Saved = target_temperature[which_extruder];
						if(which_extruder == 0) setTargetHotend(max(unload_temp_l,old_unload_temp_l),which_extruder);
						else setTargetHotend(max(unload_temp_r,old_unload_temp_r),which_extruder);
						gif_processing_state = PROCESSING_DEFAULT;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_UNLOAD_MENU,0);
						
						current_position[Z_AXIS]=Z_MAX_POS-15;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder); //check speed
						st_synchronize();
						current_position[Y_AXIS]=10;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Y_AXIS], active_extruder); //check speed
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						
						gif_processing_state = PROCESSING_STOP;
						
						touchscreen_update();
						
						genie.WriteStr(STRING_UTILITIES_FILAMENT_CHANGEFILAMENT_TEMPS,"0%");
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_CHANGEFILAMENT_TEMPS,0);
						Tref1 = (int)degHotend(which_extruder);
						Tfinal1 = (int)degTargetHotend(which_extruder);
						
						
						gif_processing_state = PROCESSING_CHANGE_FILAMENT_TEMPS;
						touchscreen_update();
						delay(500);
						is_changing_filament=true; //We are changing filament
						
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_PLA:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						if (which_extruder == 1) // Need to pause
						{
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							print_temp_r = PLA_PRINT_TEMP;
							load_temp_r = PLA_LOAD_TEMP;
							unload_temp_r = PLA_UNLOAD_TEMP;
							bed_temp_r = PLA_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend1(load_temp_r);
							insertmetod();
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_MENU,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_BACK,0);
							
						}
						else if(which_extruder == 0){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							print_temp_l = PLA_PRINT_TEMP;
							load_temp_l = PLA_LOAD_TEMP;
							unload_temp_l = PLA_UNLOAD_TEMP;
							bed_temp_l = PLA_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend0(load_temp_l);
							insertmetod();
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_MENU,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_BACK,0);
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_ABS:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						if (which_extruder == 1) // Need to pause
						{
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							
							
							print_temp_r = ABS_PRINT_TEMP;
							load_temp_r = ABS_LOAD_TEMP;
							unload_temp_r = ABS_UNLOAD_TEMP;
							bed_temp_r = ABS_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend1(load_temp_r);
							insertmetod();
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_MENU,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_BACK,0);
						}
						else if(which_extruder == 0){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							
							print_temp_l = ABS_PRINT_TEMP;
							load_temp_l = ABS_LOAD_TEMP;
							unload_temp_l = ABS_UNLOAD_TEMP;
							bed_temp_l = ABS_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend0(load_temp_l);
							insertmetod();
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_MENU,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_BACK,0);
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_PVA:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						if (which_extruder == 1) // Need to pause
						{
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							
							
							print_temp_r = PVA_PRINT_TEMP;
							load_temp_r = PVA_LOAD_TEMP;
							unload_temp_r = PVA_UNLOAD_TEMP;
							bed_temp_r = PVA_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend1(load_temp_r);
							insertmetod();
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_MENU,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_BACK,0);
						}
						else if(which_extruder == 0){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							
							print_temp_l = PVA_PRINT_TEMP;
							load_temp_l = PVA_LOAD_TEMP;
							unload_temp_l = PVA_UNLOAD_TEMP;
							bed_temp_l = PVA_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend0(load_temp_l);
							insertmetod();
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_MENU,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_BACK,0);
						}
					}
					break;
					
					//CUSTOM MATERIAL BUTTONS
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM:
					if (millis() >= waitPeriod_button_press){
						
						if (which_extruder == 1 || which_extruder == 0) // Need to pause
						{
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_LOAD_CUSTOM,0);
							char buffer[10];
							char buffer1[10];
							char buffer2[10];
							char buffer3[256];
							sprintf(buffer, "%d %cC",custom_insert_temp,0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_LOAD	,buffer);
							sprintf(buffer1, "%d %cC",custom_remove_temp,0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_UNLOAD,buffer1);
							sprintf(buffer2, "%d %cC",custom_print_temp,0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_PRINT,buffer2);
							sprintf(buffer3, "%d %cC",custom_bed_temp,0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_BED,buffer3);
							
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_BACK:
					if (millis() >= waitPeriod_button_press){
						
						
						/*genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE2, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_FILAMENT_NOZZLE1, 0);*/
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_LOAD,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						//which_extruder = -1;
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_INS_LESS:
					if (custom_insert_temp > 0){
						char buffer[256];
						custom_insert_temp -= 10;
						sprintf(buffer, "%1d %cC",custom_insert_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_LOAD	,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_INS_MORE:
					if (custom_insert_temp < HEATER_0_MAXTEMP-5){
						char buffer[256];
						custom_insert_temp += 10;
						sprintf(buffer, "%1d %cC",custom_insert_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_LOAD	,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_REM_LESS:
					if (custom_remove_temp > 0){
						char buffer[256];
						custom_remove_temp -= 10;
						sprintf(buffer, "%1d %cC",custom_remove_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_UNLOAD,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_REM_MORE:
					if (custom_remove_temp < HEATER_0_MAXTEMP-5){
						char buffer[256];
						custom_remove_temp += 10;
						sprintf(buffer, "%1d %cC",custom_remove_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_UNLOAD,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_PRINT_LESS:
					if (custom_print_temp > 0){
						char buffer[256];
						custom_print_temp -= 10;
						sprintf(buffer, "%1d %cC",custom_print_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_PRINT,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_PRINT_MORE:
					if (custom_print_temp < HEATER_0_MAXTEMP-5){
						char buffer[256];
						custom_print_temp += 10;
						sprintf(buffer, "%1d %cC",custom_print_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_PRINT,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_BED_LESS:
					if (custom_bed_temp > 0){
						char buffer[256];
						custom_bed_temp -= 10;
						sprintf(buffer, "%1d %cC",custom_bed_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_BED,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_BED_MORE:
					if (custom_bed_temp < BED_MAXTEMP -5){
						char buffer[256];
						custom_bed_temp += 10;
						sprintf(buffer, "%1d %cC",custom_bed_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_BED,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_ACCEPT:
					
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						if(which_extruder == 0){
							if (custom_print_temp <= HEATER_0_MAXTEMP) print_temp_l = custom_print_temp;
							else print_temp_l = HEATER_0_MAXTEMP;
							if (custom_insert_temp <= HEATER_0_MAXTEMP) load_temp_l = custom_insert_temp;
							else load_temp_l = HEATER_0_MAXTEMP;
							if (custom_remove_temp <= HEATER_0_MAXTEMP) unload_temp_l = custom_remove_temp;
							else unload_temp_l = HEATER_0_MAXTEMP;
							if (custom_bed_temp <= BED_MAXTEMP) bed_temp_l = custom_bed_temp;
							else bed_temp_l = BED_MAXTEMP;
							setTargetHotend0(load_temp_l);
						}
						else{
							if (custom_print_temp <= HEATER_1_MAXTEMP)print_temp_r = custom_print_temp;
							else print_temp_r = HEATER_1_MAXTEMP;
							if (custom_insert_temp <= HEATER_1_MAXTEMP)load_temp_r = custom_insert_temp;
							else print_temp_r = HEATER_1_MAXTEMP;
							if (custom_remove_temp <= HEATER_1_MAXTEMP)unload_temp_r = custom_remove_temp;
							else unload_temp_r = HEATER_1_MAXTEMP;
							if (custom_bed_temp <= BED_MAXTEMP)bed_temp_r = custom_bed_temp;
							else bed_temp_r = BED_MAXTEMP;
							setTargetHotend1(load_temp_r);
						}
						Config_StoreSettings();
						insertmetod();
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_MENU,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_BACK,0);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_HANDS:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						//ATTENTION : Order here is important
						genie.WriteStr(STRING_UTILITIES_FILAMENT_CHANGEFILAMENT_TEMPS,"0%");
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_CHANGEFILAMENT_TEMPS,0);
						//genie.WriteStr(STRING_ADVISE_FILAMENT,"");
						//genie.WriteStr(STRING_ADVISE_FILAMENT,"Insert the filament until you feel it stops, \n then while you keep inserting around \n 10 mm of filament, press the clip");
						gif_processing_state = PROCESSING_CHANGE_FILAMENT_TEMPS;
						touchscreen_update();
						if (which_extruder==0) setTargetHotend(max(load_temp_l,old_load_temp_l),which_extruder);
						else setTargetHotend(max(load_temp_r,old_load_temp_r),which_extruder);
						//delay(3500);
						
						
						
						current_position[Y_AXIS] = 100;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
						st_synchronize();
						
						
						Tref1 = (int)degHotend(which_extruder);
						Tfinal1 = (int)degTargetHotend(which_extruder);
						
						touchscreen_update();
						delay(500);
						
						is_changing_filament=true;
					}
					break;
					
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_KEEPPUSHING_NEXT:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						
						if (filament_mode =='I')
						{ //Inserting...
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							gif_processing_state = PROCESSING_DEFAULT;
							delay(550);
							plan_set_position(extruder_offset[X_AXIS][which_extruder], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
							#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
							current_position[X_AXIS] = 155;
							#else
							current_position[X_AXIS] = 155 + X_OFFSET_CALIB_PROCEDURES;
							#endif
							plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
							st_synchronize();
							SERIAL_PROTOCOLPGM("Loading:\n");
							current_position[E_AXIS] += 30;//Extra extrusion at low feedrate
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  700/60, which_extruder); //850/60
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							current_position[E_AXIS] += ((BOWDEN_LENGTH-EXTRUDER_LENGTH)-15);//BOWDEN_LENGTH-300+340);
							Serial.println(current_position[E_AXIS]);
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, which_extruder);
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							current_position[E_AXIS] += EXTRUDER_LENGTH;//Extra extrusion at low feedrate
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_SLOW_SPEED/60, which_extruder);
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							gif_processing_state = PROCESSING_STOP;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_ADJUST,0);
						}
					}
					break;
					
					
					case BUTTON_UTILITIES_FILAMENT_UNLOAD_ROLLTHESPOOL_NEXT:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						
						if (filament_mode =='R')
						{ //Removing...
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							gif_processing_state = PROCESSING_DEFAULT;
							delay(550);
							current_position[E_AXIS] -= (BOWDEN_LENGTH + EXTRUDER_LENGTH + 100);//Extra extrusion at fast feedrate
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_FAST_SPEED/60, which_extruder);
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							previous_state = FORM_UTILITIES_FILAMENT;
							
							gif_processing_state = PROCESSING_STOP;
							printer_state = STATE_LOADUNLOAD_FILAMENT;
							touchscreen_update();
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_SUCCESS,0);
							genie.WriteObject(GENIE_OBJ_VIDEO,GIF_UTILITIES_FILAMENT_SUCCESS,0);
							gif_processing_state = PROCESSING_SUCCESS;
							if (which_extruder == 0){
								old_load_temp_l = load_temp_l;
								old_unload_temp_l = unload_temp_l;
								old_print_temp_l  = print_temp_l;
							}
							else{
								old_load_temp_r = load_temp_r;
								old_unload_temp_r = unload_temp_r;
								old_print_temp_r  = print_temp_r;
							}
						}
					}
					break;
					#pragma endregion Insert_Remove_Fil
					
					
					#pragma region AdjustFilament
					case BUTTON_UTILITIES_FILAMENT_ADJUST_ACCEPT:
					
					if(!flag_utilities_filament_acceptok)
					{
						
						if (millis() >= waitPeriod_button_press){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							gif_processing_state = PROCESSING_DEFAULT;
							current_position[X_AXIS] = extruder_offset[X_AXIS][which_extruder];
							plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
							plan_set_position(extruder_offset[X_AXIS][active_extruder], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
							current_position[X_AXIS] = extruder_offset[X_AXIS][active_extruder];
							setTargetHotend((float)Temp_ChangeFilament_Saved, which_extruder);
							st_synchronize();
							gif_processing_state = PROCESSING_STOP;
							touchscreen_update();
							printer_state = STATE_LOADUNLOAD_FILAMENT;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_SUCCESS,0);
							genie.WriteObject(GENIE_OBJ_VIDEO,GIF_UTILITIES_FILAMENT_SUCCESS,0);
							gif_processing_state = PROCESSING_SUCCESS;
							
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_ADJUST_LOAD:
					if(!flag_utilities_filament_acceptok){
						if(!blocks_queued()){
							flag_utilities_filament_purgeload = 1;
							}else{
							quickStop();
						}
						
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_ADJUST_UNLOAD:
					if(!flag_utilities_filament_acceptok){
						if(!blocks_queued()){
							flag_utilities_filament_purgeunload = 1;
							}else{
							quickStop();
						}
					}
					
					#pragma endregion AdjustFilament
					
					#pragma region PURGEpause
					
					//****************PURGE BUTTONS******
					case BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0:
					if (!blocks_queued()){
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_LOAD,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_UNLOAD,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_TEMPUP,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_TEMPDOWN,1);
						if (purge_extruder_selected == 1){
							purge_extruder_selected = 0;
							if(target_temperature[0] == 0){
								setTargetHotend0(print_temp_l);
							}
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,0);
							char buffer[256];
							sprintf(buffer, "%d %cC",target_temperature[0],0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);
							Serial.println(buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_LEFTTARGET,buffer);
						}
						else{
							char buffer[256];
							purge_extruder_selected = 0;
							if(target_temperature[0] == 0){
								setTargetHotend0(print_temp_l);
							}
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,0);
							sprintf_P(buffer, PSTR("%3d %cC"),target_temperature[0],0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_LEFTTARGET,buffer);
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1:
					if (!blocks_queued()){
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_LOAD,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_UNLOAD,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_TEMPUP,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_TEMPDOWN,1);
						if (purge_extruder_selected == 0){
							purge_extruder_selected = 1;
							if(target_temperature[1] == 0){
								setTargetHotend1(print_temp_r);
							}
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,1);
							char buffer[256];
							sprintf_P(buffer, PSTR("%3d %cC"),target_temperature[1],0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);
							Serial.println(buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_RIGHTTARGET,buffer);
						}
						else{
							purge_extruder_selected = 1;
							if(target_temperature[1] == 0){
								setTargetHotend1(print_temp_r);
							}
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,1);
							char buffer[256];
							sprintf_P(buffer, PSTR("%3d %cC"),target_temperature[1],0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_RIGHTTARGET,buffer);
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_PURGE_TEMPUP:
					if(purge_extruder_selected != -1){
						
						if (target_temperature[purge_extruder_selected] < HEATER_0_MAXTEMP){
							target_temperature[purge_extruder_selected] += 5;
							setTargetHotend0(target_temperature[0]);
							setTargetHotend1(target_temperature[1]);
							char buffer[256];
							sprintf_P(buffer, PSTR("%3d %cC"),int(target_temperature[purge_extruder_selected]),0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_PURGE_TEMPDOWN:
					if(purge_extruder_selected != -1){
						if (target_temperature[purge_extruder_selected] > 0){
							target_temperature[purge_extruder_selected] -= 5;
							setTargetHotend0(target_temperature[0]);
							setTargetHotend1(target_temperature[1]);
							char buffer[256];
							sprintf_P(buffer, PSTR("%3d %cC"),int(target_temperature[purge_extruder_selected]),0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);
						}
					}
					break;
					//***MOVING
					
					case BUTTON_UTILITIES_FILAMENT_PURGE_LOAD:
					if(purge_extruder_selected != -1){
						if(!blocks_queued()){
							flag_utilities_filament_purgeselect0 = 1;
							}else{
							quickStop();
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_PURGE_UNLOAD:
					if(purge_extruder_selected != -1){
						if(!blocks_queued()){
							flag_utilities_filament_purgeselect1 = 1;
							}else{
							quickStop();
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_PURGE_BACK:
					if(!blocks_queued()){
						//quickStop();
						if (millis() >= waitPeriod_button_press){
							is_purging_filament = false;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_SDPRINTING_PAUSE,0);
							touchscreen_update();
							is_on_printing_screen = true;
							surfing_utilities = false;
							genie.WriteStr(STRING_SDPRINTING_PAUSE_GCODE,namefilegcode);
							flag_sdprinting_dararefresh = true;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
						//setTargetHotend0(0);
						//setTargetHotend1(0);
						
					}
					break;
					//************************************
					#pragma endregion PURGEpause
					#pragma region SuccessScreensPrint
					case BUTTON_UTILITIES_FILAMENT_SUCCESS:
					if (printer_state == STATE_LOADUNLOAD_FILAMENT)
					{
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							
							//enquecommand_P((PSTR("G28 X0 Y0")));
							SERIAL_PROTOCOLPGM("SUCCESS\n");
							gif_processing_state = PROCESSING_STOP;
							flag_utilities_filament_acceptok = false;
							if (filament_mode == 'R')
							{
								
								filament_mode = 'I';
								if(which_extruder ==  1){
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT1, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0, 0);
								}
								else{
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT1, 0);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0, 1);
								}
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_MENU,1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_BACK,1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PLA, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_ABS, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PVA, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM, 1);
								
								genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_LOAD,0);
								//which_extruder = -1;
							}
							else if (filament_mode == 'I')
							{
								
								gif_processing_state = PROCESSING_DEFAULT;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
								current_position[Y_AXIS] = saved_position[Y_AXIS];
								plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Y_AXIS]/60, active_extruder);//Purge
								st_synchronize();
								if(gif_processing_state == PROCESSING_ERROR)return;
								
								//home_axis_from_code(true, true, false);
								
								current_position[Z_AXIS] = saved_position[Z_AXIS] + 10;
								plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],homing_feedrate[Z_AXIS] ,saved_active_extruder);
								st_synchronize();
								if(gif_processing_state == PROCESSING_ERROR)return;
								gif_processing_state = PROCESSING_STOP;
								
								
								
								
								
								genie.WriteObject(GENIE_OBJ_FORM,FORM_SDPRINTING_PAUSE,0);
								is_on_printing_screen = true;
								surfing_utilities = false;
								genie.WriteStr(STRING_SDPRINTING_PAUSE_GCODE,namefilegcode);
								flag_sdprinting_dararefresh = true;
							}
							printer_state = STATE_NONE;
							//doblocking =true;
						}
						
						
					}
					break;
					#pragma endregion SuccessScreensPrin
					
					

					case BUTTON_ERROR_OK:
					
					if (millis() >= waitPeriod_button_press){
						
						if(FLAG_thermal_runaway_screen){
							gif_processing_state = PROCESSING_STOP;
							FLAG_thermal_runaway_screen = false;
							flag_sdprinting_showdata = true;
							if(screen_printing_pause_form == screen_printing_pause_form0){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_SDPRINTING,0);
								}else{
								genie.WriteObject(GENIE_OBJ_FORM,FORM_SDPRINTING_PAUSE,0);
							}
							
							}else{
							gif_processing_state = PROCESSING_STOP;
							screen_sdcard = false;
							surfing_utilities=false;
							SERIAL_PROTOCOLPGM("Surfing 0\n");
							surfing_temps = false;
							HeaterCooldownInactivity(true);
							genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN, 0);
						}
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					
					break;
				}//endswitch
			}
			else if(flag_utilities_calibration_bedcomensationmode && (Bed_Compensation_state > 1)){
				/*SERIAL_PROTOCOLPGM("Bed_Compensation_Lines_Selected[0]");
				Serial.println(Bed_Compensation_Lines_Selected[0]);
				SERIAL_PROTOCOLPGM("Bed_Compensation_Lines_Selected[1]");
				Serial.println(Bed_Compensation_Lines_Selected[1]);
				SERIAL_PROTOCOLPGM("Bed_Compensation_Lines_Selected[2]");
				Serial.println(Bed_Compensation_Lines_Selected[2]);*/
				int8_t vuitensL = 0;
				int8_t vuitensR = 0;
				switch(Event.reportObject.index){
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL_SELECT1:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Bed_Compensation_Set_Lines(-2);
						
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL_SELECT2:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Bed_Compensation_Set_Lines(-1);
						
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL_SELECT3:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Bed_Compensation_Set_Lines(0);
						
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL_SELECT4:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Bed_Compensation_Set_Lines(1);
						
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL_SELECT5:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Bed_Compensation_Set_Lines(2);
						
					}
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDOZL_BEST1:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Bed_compensation_redo_offset = -3;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDOZL_BEST5:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Bed_compensation_redo_offset = 3;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDOZL:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Bed_compensation_redo_offset = 0;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						gif_processing_state = PROCESSING_DEFAULT;
						home_axis_from_code(true,true,true);
						gif_processing_state = PROCESSING_STOP;
						st_synchronize();
						Calib_check_temps();
						Bed_Compensation_Redo_Lines(Bed_compensation_redo_offset);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBBED_ADJUSTSCREWASKFIRST_NEXT:
					gif_processing_state = PROCESSING_STOP;
					vuitensL = Bed_Compensation_Lines_Selected[1]-Bed_Compensation_Lines_Selected[0];
					vuitensR = Bed_Compensation_Lines_Selected[2]-Bed_Compensation_Lines_Selected[0];
					bed_offset_left_screw = -0.1*vuitensL;
					bed_offset_right_screw = -0.1*vuitensR;
					if(vuitensL != 0){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBBED_SCREW2,0);
						if(vuitensL < 0){
							vuitensL = abs(vuitensL) + 8;
						}
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_CALIBBED_SCREW2,vuitensL);
						
					}
					else if(vuitensR != 0){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBBED_SCREW3,0);
						if(vuitensR < 0){
							vuitensR = abs(vuitensR) + 8;
						}
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_CALIBBED_SCREW3,vuitensR);
					}
					else{
						printer_state = STATE_CALIBRATION;
						genie.WriteObject(GENIE_OBJ_VIDEO,GIF_UTILITIES_CALIBRATION_SUCCESS,0);
						genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES_CALIBRATION_SUCCESS, 0);
						gif_processing_state = PROCESSING_BED_SUCCESS;
					}
					bed_offset_version = VERSION_NUMBER;
					Config_StoreSettings();
					
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBBED_SCREW2_NEXT:
					vuitensR = 0;
					vuitensR = Bed_Compensation_Lines_Selected[2]-Bed_Compensation_Lines_Selected[0];
					if(vuitensR != 0){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBBED_SCREW3,0);
						if(vuitensR < 0){
							vuitensR = abs(vuitensR) + 8;
						}
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_CALIBBED_SCREW3,vuitensR);
					}
					else{
						printer_state = STATE_CALIBRATION;
						genie.WriteObject(GENIE_OBJ_VIDEO,GIF_UTILITIES_CALIBRATION_SUCCESS,0);
						genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES_CALIBRATION_SUCCESS, 0);
						gif_processing_state = PROCESSING_BED_SUCCESS;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBBED_SCREW3_NEXT:
					printer_state = STATE_CALIBRATION;
					genie.WriteObject(GENIE_OBJ_VIDEO,GIF_UTILITIES_CALIBRATION_SUCCESS,0);
					genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES_CALIBRATION_SUCCESS, 0);
					gif_processing_state = PROCESSING_BED_SUCCESS;
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_SUCCESS:
					if(printer_state == STATE_CALIBRATION)
					{
						Bed_Compensation_state = 0;
						flag_utilities_calibration_bedcomensationmode = 0;
						gif_processing_state = PROCESSING_STOP;
						setTargetBed(0);
						setTargetHotend0(0);
						setTargetHotend1(0);
						screen_sdcard = false;
						surfing_utilities=false;
						SERIAL_PROTOCOLPGM("Surfing 0\n");
						surfing_temps = false;
						HeaterCooldownInactivity(false);
						genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN, 0);
						printer_state = STATE_NONE;
					}
					break;
				}//endswitch
				
			}
			else{//All that has to be done out of the printing room
				int tHotend;
				int tHotend1;
				int tBed;
				char buffer[256];
				float feedrate;
				float value;
				switch(Event.reportObject.index){
					//*****SD Gcode Selection*****
					#pragma region SD Gcode Selector
					
					case BUTTON_SDLIST_SELECT0:
					flag_sdlist_select0 = true;
					break;
					
					case BUTTON_SDLIST_SELECT1:
					flag_sdlist_select1 = true;
					break;
					
					case BUTTON_SDLIST_SELECT2:
					flag_sdlist_select2 = true;
					break;
					
					case BUTTON_SDLIST_SELECT3:
					flag_sdlist_select3 = true;
					break;
					
					case BUTTON_SDLIST_SELECT4:
					flag_sdlist_select4 = true;
					break;
															
					case BUTTON_SDLIST_FOLDERBACK:
					if (millis() >= waitPeriod_button_press){
						
						int updir = card.updir();
						
						if (updir==0){
							flag_sdlist_gofolderback = true;
							folder_navigation_register(false);
						}
						else if(updir==1){
							flag_sdlist_gofolderback = true;
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_SDLIST_FOLDERBACK,0);
							genie.WriteObject(GENIE_OBJ_USERIMAGES, USERIMAGE_SDLIST_FOLDERFILE,0);
							folder_navigation_register(false);
						}
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_INSERT_SD_CARD:
					if (millis() >= waitPeriod_button_press){
						
						screen_sdcard = false;
						surfing_utilities=false;
						SERIAL_PROTOCOLPGM("Surfing 0\n");
						surfing_temps = false;
						HeaterCooldownInactivity(true);
						genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN, 0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_SDLIST_CONFIRMATION_YES:
					if (millis() >= waitPeriod_button_press){
						
						
						if(card.cardOK)
						{
							//doblocking = true;
							#if BCN3D_SCREEN_VERSION_SETUP == BCN3D_SIGMA_PRINTER_DEVMODE_1
							if(listsd.check_extract_ensure_duplication_print()){
								if(abs(extruder_offset[Z_AXIS][RIGHT_EXTRUDER]) > RAFT_Z_THRESHOLD){
									genie.WriteObject(GENIE_OBJ_FORM,FORM_RAFT_ADVISE, 0);
									sprintf_P(buffer, PSTR("The first layer printed with the %s Hotend\nwill be distorted by %d.%1d%1d mm"),
									((extruder_offset[Z_AXIS][RIGHT_EXTRUDER] < 0)?"left":"right"),
									(int)abs(extruder_offset[Z_AXIS][RIGHT_EXTRUDER]),(int)(abs(extruder_offset[Z_AXIS][RIGHT_EXTRUDER])*10)%10, (int)(abs(extruder_offset[Z_AXIS][RIGHT_EXTRUDER])*100)%10);
									//((extruder_offset[Z_AXIS][RIGHT_EXTRUDER] < 0)?"right":"left"),
									Serial.println(buffer);
									//sprintf_P(buffer, PSTR("Info: First layer printed with"));
									genie.WriteStr(STRING_RAFT_ADVISE_Z_OFFSET,buffer);
									return;
									//sprintf_P(buffer, PSTR("Info: First layer printed with %s Hotend will be %d.%1d%1d mm higher. You can avoid this compensation using gauges.\n")
								}
							}
							#endif
							if (!card.filenameIsDir){ //If the filename is a gcode we start printing
								card.getfilename(filepointer);
								char cmd[4 + strlen(card.filename) + 1]; // Room for "M23 ", filename, and null
								sprintf_P(cmd, PSTR("M23 %s"), card.filename);
								for (char *c = &cmd[4]; *c; c++) *c = tolower(*c);
								enquecommand(cmd);
								enquecommand_P(PSTR("M24")); // It also sends you to PRINTING screen
							}
							
						}
					}
					waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					break;
					
					#if BCN3D_SCREEN_VERSION_SETUP == BCN3D_SIGMA_PRINTER_DEVMODE_1
					case BUTTON_RAFT_ADVISE_ACCEPT:
					if (millis() >= waitPeriod_button_press){
						
						
						if(card.cardOK)
						{
							
							if (!card.filenameIsDir){ //If the filename is a gcode we start printing
								card.getfilename(filepointer);
								char cmd[4 + strlen(card.filename) + 1]; // Room for "M23 ", filename, and null
								sprintf_P(cmd, PSTR("M23 %s"), card.filename);
								for (char *c = &cmd[4]; *c; c++) *c = tolower(*c);
								enquecommand(cmd);
								enquecommand_P(PSTR("M24")); // It also sends you to PRINTING screen
							}
							
						}
					}
					waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					break;
					
					case BUTTON_RAFT_ADVISE_INSTALL_CANCEL:
					case BUTTON_SDLIST_CONFIRMATION_NO:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_SDLIST_FOLDERBACK,0);
						genie.WriteObject(GENIE_OBJ_USERIMAGES, USERIMAGE_SDLIST_FOLDERFILE,0);
						genie.WriteObject(GENIE_OBJ_FORM, FORM_SDLIST,0);
						screen_sdcard = true;
						flag_sdlist_goinit = true;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					#endif
					
					case BUTTON_SDLIST_GOUP:
					if (millis() >= waitPeriod_button_press){
						flag_sdlist_goup = true;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					case BUTTON_SDLIST_GODOWN:
					if (millis() >= waitPeriod_button_press){
						flag_sdlist_godown = true;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					#pragma endregion SD Gcode Selector
					
					#pragma region PREHEAT
					case BUTTON_UTILITIES_MAINTENANCE:
					if (millis() >= waitPeriod_button_press){
						genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES_MAINTENANCE, 0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_MAIN_TEMPS:
					if (millis() >= waitPeriod_button_press){
						genie.WriteObject(GENIE_OBJ_FORM, FORM_TEMP, 0);
						HeaterCooldownInactivity(false);
						int tHotend=target_temperature[0];
						int tHotend1=target_temperature[1];
						int tBed=target_temperature_bed;
						
						
						flag_temp_gifhotent0=false;
						flag_temp_gifhotent1 = false;
						flag_temp_gifbed = false;
						surfing_temps = true;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_TEMP_BACK:
					if (millis() >= waitPeriod_button_press){
						screen_sdcard = false;
						surfing_utilities=false;
						SERIAL_PROTOCOLPGM("Surfing 0\n");
						surfing_temps = false;
						touchscreen_update();
						HeaterCooldownInactivity(true);
						genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN, 0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_TEMP_LEXTR:
					tHotend=target_temperature[0];
					if(tHotend != 0){
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PREHEAT_LEXTR,0); //<GIFF
						setTargetHotend0(0);
					}
					else{
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PREHEAT_LEXTR,1); //<GIFF
						setTargetHotend0(print_temp_l);
					}
					flag_temp_gifhotent0 = false;
					break;
					
					case BUTTON_TEMP_REXTR:
					tHotend1=target_temperature[1];
					if(tHotend1 != 0)
					{
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PREHEAT_REXTR,0); //<GIFF
						setTargetHotend1(0);
					}
					else{
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PREHEAT_REXTR,1); //<GIFF
						setTargetHotend1(print_temp_r);
					}
					flag_temp_gifhotent1 = false;
					break;
					
					case BUTTON_TEMP_BED:
					tBed=target_temperature_bed;
					if(tBed != 0){
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PREHEAT_BED,0); //<GIFF
						setTargetBed(0);
					}
					else {
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PREHEAT_BED,1); //<GIFF
						setTargetBed( max(bed_temp_l,bed_temp_r));
						
					}
					flag_temp_gifbed = false;
					break;
					
					#pragma endregion PREHEAT
					
					#pragma region RecoveyPrint
					
					case BUTTON_RECOVERY_PRINT_ASK_ACCEPT:
					if (millis() >= waitPeriod_button_press){
						enquecommand_P(PSTR("M34"));
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_RECOVERY_PRINT_ASK_CANCEL:
					if (millis() >= waitPeriod_button_press){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_RECOVERYPRINT_TOBELOST,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_RECOVERYPRINT_TOBELOST_ACCEPT:
					if (millis() >= waitPeriod_button_press){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN,0);
						screen_sdcard = false;
						surfing_utilities=false;
						SERIAL_PROTOCOLPGM("Surfing 0\n");
						surfing_temps = false;
						saved_print_flag = 888;
						Config_StoreSettings();
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_RECOVERYPRINT_TOBELOST_BACK:
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
									SERIAL_PROTOCOLLNPGM("Found");
									}else{
									if (card.chdir(card.filename)!=-1){
									}
								}
							}
							setfilenames(7);
						}
						
					}
					break;
					
					#pragma endregion RecoveyPrint
					
					#pragma region Maintenance
					
					case BUTTON_UTILITIES_MAINTENANCE_ZADJUST:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						if(saved_print_flag==1888){
							saved_print_flag = 888;
							Config_StoreSettings();
						}
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						gif_processing_state = PROCESSING_DEFAULT;
						if(home_made_Z){
							home_axis_from_code(true,true,false);
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
						}
						else{
							home_axis_from_code(true,true,true);
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
						}
						HeaterCooldownInactivity(true);
						gif_processing_state = PROCESSING_STOP;
						enquecommand_P((PSTR("T0")));
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						genie.WriteObject(GENIE_OBJ_FORM, FORM_MAINTENANCE_ZADJUST, 0);
						
					}
					break;
					
					case BUTTON_UTILITIES_MAINTENANCE_BACK:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES, 0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_MAINTENANCE_AUTOTUNEHOTENDS:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						genie.WriteObject(GENIE_OBJ_FORM, FORM_ADJUSTING_TEMPERATURES, 0);
						flag_utilities_maintenance_autotune = true;
						gif_processing_state = PROCESSING_ADJUSTING;
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
						gif_processing_state = PROCESSING_STOP;
						touchscreen_update();
						genie.WriteObject(GENIE_OBJ_VIDEO,GIF_UTILITIES_CALIBRATION_SUCCESS,0);
						genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES_CALIBRATION_SUCCESS, 0);
						printer_state = STATE_CALIBRATION;
						gif_processing_state = PROCESSING_BED_SUCCESS;
						
					}
					break;
					
					case BUTTON_UTILITIES_MAINTENANCE_MENU:
					if (millis() >= waitPeriod_button_press){
						screen_sdcard = false;
						surfing_utilities=false;
						SERIAL_PROTOCOLPGM("Surfing 0\n");
						surfing_temps = false;
						touchscreen_update();
						genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN, 0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING:
					if (millis() >= waitPeriod_button_press){
						
						filament_mode = 'R';
						flag_utilities_maintenance_nyloncleaning = true;
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SELECTLEFT, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SELECTRIGHT, 0);
						genie.WriteObject(GENIE_OBJ_USERIMAGES, IMAG_UTILITIES_MAINTENANCE_NYLONCLEANING_TEXTBUTTON, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_UNLOADSKIP, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_UNLOADGO, 0);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING,0);
						
						which_extruder = 255;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
					}
					break;
					
					#pragma region Nylon
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_BACK:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES_MAINTENANCE, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SELECTLEFT, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SELECTRIGHT, 0);
						genie.WriteObject(GENIE_OBJ_USERIMAGES, IMAG_UTILITIES_MAINTENANCE_NYLONCLEANING_TEXTBUTTON, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_UNLOADSKIP, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_UNLOADGO, 0);
						flag_utilities_maintenance_nyloncleaning = false;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_MENU:
					if (millis() >= waitPeriod_button_press){
						
						screen_sdcard = false;
						surfing_utilities=false;
						SERIAL_PROTOCOLPGM("Surfing 0\n");
						surfing_temps = false;
						genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SELECTLEFT, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SELECTRIGHT, 0);
						genie.WriteObject(GENIE_OBJ_USERIMAGES, IMAG_UTILITIES_MAINTENANCE_NYLONCLEANING_TEXTBUTTON, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_UNLOADSKIP, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_UNLOADGO, 0);
						flag_utilities_maintenance_nyloncleaning = false;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SELECTLEFT:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SELECTLEFT, 1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SELECTRIGHT, 0);
						genie.WriteObject(GENIE_OBJ_USERIMAGES, IMAG_UTILITIES_MAINTENANCE_NYLONCLEANING_TEXTBUTTON, 1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_UNLOADSKIP, 1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_UNLOADGO, 1);
						which_extruder = 0;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SELECTRIGHT:
					if (millis() >= waitPeriod_button_press){
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SELECTLEFT, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SELECTRIGHT, 1);
						genie.WriteObject(GENIE_OBJ_USERIMAGES, IMAG_UTILITIES_MAINTENANCE_NYLONCLEANING_TEXTBUTTON, 1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_UNLOADSKIP, 1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_UNLOADGO, 1);
						which_extruder = 1;
						
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_UNLOADGO:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						if(saved_print_flag==1888){
							saved_print_flag = 888;
							Config_StoreSettings();
						}
						if(which_extruder != 255){
							
							if(which_extruder == 0) setTargetHotend(max(unload_temp_l,old_unload_temp_l),which_extruder);
							else setTargetHotend(max(unload_temp_r,old_unload_temp_r),which_extruder);
							
							doblocking = true;
							gif_processing_state = PROCESSING_DEFAULT;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							
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
							if(gif_processing_state == PROCESSING_ERROR)return;
							
							gif_processing_state = PROCESSING_STOP;
							touchscreen_update();
							genie.WriteStr(STRING_UTILITIES_FILAMENT_CHANGEFILAMENT_TEMPS,"0%");
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_CHANGEFILAMENT_TEMPS,0);
							Tref1 = (int)degHotend(which_extruder);
							Tfinal1 = (int)degTargetHotend(which_extruder);
							
							gif_processing_state = PROCESSING_CHANGE_FILAMENT_TEMPS;
							touchscreen_update();
							delay(500);
							is_changing_filament=true; //We are changing filament
							
							
						}
						
					}
					break;
					
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_UNLOADSKIP:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						if(saved_print_flag==1888){
							saved_print_flag = 888;
							Config_StoreSettings();
						}
						if(which_extruder != 255){
							doblocking = true;
							setTargetHotend(NYLON_TEMP_HEATUP_THRESHOLD,which_extruder);
							gif_processing_state = PROCESSING_DEFAULT;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							
							if (home_made_Z){
								home_axis_from_code(true,true,false);
							}
							else{
								home_axis_from_code(true,true,true);
							}
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							current_position[Z_AXIS]=Z_MAX_POS-15;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder); //check speed
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							current_position[Y_AXIS]=10;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Y_AXIS], active_extruder); //check speed
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
																					
							if (which_extruder == 0) changeTool(0);
							else changeTool(1);
							
							#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
							current_position[X_AXIS] = 155;
							#else
							current_position[X_AXIS] = 155 + 100;
							#endif
							
							
							plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							gif_processing_state = PROCESSING_STOP;
							touchscreen_update();
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP0,0);
							
						}
						
					}
					break;
					
					#pragma endregion Nylon
					#pragma region Zset
					
					case BUTTON_MAINTENANCE_ZADJUST_TOP:
					if (millis() >= waitPeriod_button_press){
						flag_maintenance_zdjust100up = true;
						flag_maintenance_zdjust100down = false;
						flag_maintenance_zdjust10down = false;
						flag_maintenance_zdjust10up = false;
						waitPeriod_button_press = millis()+250;
					}
					break;
					
					case BUTTON_MAINTENANCE_ZADJUST_BOT:
					if (millis() >= waitPeriod_button_press){
						flag_maintenance_zdjust100up = false;
						flag_maintenance_zdjust100down = true;
						flag_maintenance_zdjust10down = false;
						flag_maintenance_zdjust10up = false;
						waitPeriod_button_press = millis()+250;
					}
					break;
					
					case BUTTON_MAINTENANCE_ZADJUST_DOWN:
					//if(current_position[Z_AXIS]!=Z_MIN_POS){
					if (millis() >= waitPeriod_button_press){
						flag_maintenance_zdjust100up = false;
						flag_maintenance_zdjust100down = false;
						flag_maintenance_zdjust10down = true;
						flag_maintenance_zdjust10up = false;
						waitPeriod_button_press = millis()+250;
					}
					break;
					
					case BUTTON_MAINTENANCE_ZADJUST_UP:
					//if(current_position[Z_AXIS]!=Z_MAX_POS-15){
					if (millis() >= waitPeriod_button_press){
						flag_maintenance_zdjust100up = false;
						flag_maintenance_zdjust100down = false;
						flag_maintenance_zdjust10down = false;
						flag_maintenance_zdjust10up = true;
						waitPeriod_button_press = millis()+250;
					}
					break;
					
					case BUTTON_MAINTENANCE_ZADJUST_BACK:
					if (millis() >= waitPeriod_button_press){
						processing_z_set = 255;
						touchscreen_update();
						genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES_MAINTENANCE, 0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_MAINTENANCE_ZADJUST_ACCEPT:
					if (millis() >= waitPeriod_button_press){
						processing_z_set = 255;
						touchscreen_update();
						genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES_MAINTENANCE, 0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					#pragma endregion Zset
					
					#pragma endregion Maintenance
					
					#pragma region PURGE
					//****************PURGE BUTTONS******
					case BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0:
					if (millis() >= waitPeriod_button_press){
						
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_INSERT,1);
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_Retract,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_TEMPUP,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_TEMPDOWN,1);
						if (purge_extruder_selected == 1){
							purge_extruder_selected = 0;
							if(target_temperature[0] == 0){
								setTargetHotend0(print_temp_l);
							}
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,0);
							sprintf(buffer, "%d %cC",target_temperature[0],0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);
							Serial.println(buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_LEFTTARGET,buffer);
						}
						else{
							purge_extruder_selected = 0;
							if(target_temperature[0] == 0){
								setTargetHotend0(print_temp_l);
							}
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,1);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,0);
							sprintf(buffer, "%d %cC",target_temperature[0],0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_LEFTTARGET,buffer);
						}
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					case BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1:
					if (millis() >= waitPeriod_button_press){
						
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_INSERT,1);
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PURGE_Retract,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_TEMPUP,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_TEMPDOWN,1);
						if (purge_extruder_selected == 0){
							purge_extruder_selected = 1;
							
							if(target_temperature[1] == 0){
								setTargetHotend1(print_temp_r);
							}
							
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,1);
							sprintf_P(buffer, PSTR("%3d %cC"),target_temperature[1],0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);
							Serial.println(buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_RIGHTTARGET,buffer);
						}
						else{
							purge_extruder_selected = 1;
							if(target_temperature[1] == 0){
								setTargetHotend1(print_temp_r);
							}
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,1);
							sprintf_P(buffer, PSTR("%3d %cC"),target_temperature[1],0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);
							sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_RIGHTTARGET,buffer);
						}
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_PURGE_TEMPUP:
					if(purge_extruder_selected != -1){
						if (target_temperature[purge_extruder_selected] < HEATER_0_MAXTEMP){
							target_temperature[purge_extruder_selected] += 5;
							setTargetHotend0(target_temperature[0]);
							setTargetHotend1(target_temperature[1]);
							sprintf_P(buffer, PSTR("%3d %cC"),int(target_temperature[purge_extruder_selected]),0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_PURGE_TEMPDOWN:
					if(purge_extruder_selected != -1){
						if (target_temperature[purge_extruder_selected] > 0){
							target_temperature[purge_extruder_selected] -= 5;
							setTargetHotend0(target_temperature[0]);
							setTargetHotend1(target_temperature[1]);
							sprintf_P(buffer, PSTR("%3d %cC"),int(target_temperature[purge_extruder_selected]),0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_PURGE_LOAD:
					if(purge_extruder_selected != -1){
						if(!blocks_queued()){
							flag_utilities_filament_purgeselect0 = 1;
							}else{
							quickStop();
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_PURGE_UNLOAD:
					if(purge_extruder_selected != -1){
						if(!blocks_queued()){
							flag_utilities_filament_purgeselect1 = 1;
							}else{
							quickStop();
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_PURGE:
					if (millis() >= waitPeriod_button_press){
						is_purging_filament = true;
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_LOAD,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_UNLOAD,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_TEMPUP,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_TEMPDOWN,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_MENU,0);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_PURGE,0);
						
						purge_extruder_selected = -1;
						SERIAL_PROTOCOLPGM("Purge Filament\n");
						/*setTargetHotend0(print_temp_l);
						setTargetHotend1(print_temp_r);*/
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT0,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_PURGE_SELECT1,0);
						
						sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(0)),0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_LEFTTARGET,buffer);	Serial.println(buffer);
						sprintf_P(buffer, PSTR("%3d %cC"),int(degHotend(1)),0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_RIGHTTARGET,buffer);	Serial.println(buffer);
						sprintf_P(buffer, PSTR("%3d %cC"),0,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_PURGE_SELECTEDTEMP,buffer);	Serial.println(buffer);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					
					break;
					
					case BUTTON_UTILITIES_FILAMENT_PURGE_BACK:
					if (millis() >= waitPeriod_button_press){
						is_purging_filament = false;
						quickStop();
						HeaterCooldownInactivity(true);
						touchscreen_update();
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT,0);
						
						//setTargetHotend0(0);
						//setTargetHotend1(0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_PURGE_MENU:
					if (millis() >= waitPeriod_button_press){
						is_purging_filament = false;
						quickStop();
						screen_sdcard = false;
						surfing_utilities=false;
						surfing_temps = false;
						SERIAL_PROTOCOLPGM("Surfing 0\n");
						HeaterCooldownInactivity(true);
						touchscreen_update();
						genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					//************************************
					#pragma endregion PURGE
					
					//*****INSERT/REMOVE FILAMENT*****
					#pragma region Insert_Remove_Fil
					
					case BUTTON_UTILITIES_FILAMENT_BACK:
					if (millis() >= waitPeriod_button_press){
						
						flag_utilities_filament_home=false;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					case BUTTON_UTILITIES_FILAMENT_UNLOAD:
					if (millis() >= waitPeriod_button_press){
						filament_mode = 'R';//Remove Mode
						which_extruder = -1;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_UNLOAD,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					case BUTTON_UTILITIES_FILAMENT_LOAD:
					if (millis() >= waitPeriod_button_press){
						
						filament_mode = 'I'; //Insert Mode
						
						/*if (!FLAG_FilamentHome){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
						
						}*/
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT1, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0, 0);
						
						
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PLA, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_ABS, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PVA, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM, 0);
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_LOAD,0);
						which_extruder = -1;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_UNLOAD_SELECTLEFT:
					case BUTTON_UTILITIES_FILAMENT_UNLOAD_SELECTRIGHT:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						if (Event.reportObject.index == BUTTON_UTILITIES_FILAMENT_UNLOAD_SELECTLEFT) //Left Nozzle
						{
							
							which_extruder=0;
							
						}
						else //Right Nozzle
						{
							
							which_extruder=1;
						}
						
						if(which_extruder == 0) setTargetHotend(max(unload_temp_l,old_unload_temp_l),which_extruder);
						else setTargetHotend(max(unload_temp_r,old_unload_temp_r),which_extruder);
						gif_processing_state = PROCESSING_DEFAULT;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						
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
						if(gif_processing_state == PROCESSING_ERROR)return;
						
						gif_processing_state = PROCESSING_STOP;
						touchscreen_update();
						genie.WriteStr(STRING_UTILITIES_FILAMENT_CHANGEFILAMENT_TEMPS,"0%");
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_CHANGEFILAMENT_TEMPS,0);
						Tref1 = (int)degHotend(which_extruder);
						Tfinal1 = (int)degTargetHotend(which_extruder);
						touchscreen_update();
						gif_processing_state = PROCESSING_CHANGE_FILAMENT_TEMPS;
						
						delay(500);
						is_changing_filament=true; //We are changing filament
						
					}
					
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_BACK:
					if (!Step_First_Start_Wizard){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_UNLOAD_BACK:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_UNLOAD_MENU:
					if (millis() >= waitPeriod_button_press){
						screen_sdcard = false;
						surfing_utilities=false;
						surfing_temps = false;
						SERIAL_PROTOCOLPGM("Surfing 0\n");
						genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN,0);
						
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}

					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_MENU:
					if(!Step_First_Start_Wizard){
						if (millis() >= waitPeriod_button_press){
							
							screen_sdcard = false;
							surfing_utilities=false;
							surfing_temps = false;
							SERIAL_PROTOCOLPGM("Surfing 0\n");
							genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN,0);
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					break;

					case BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0:
					case BUTTON_UTILITIES_FILAMENT_LOAD_SELECT1:
					if(!Step_First_Start_Wizard){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							if (Event.reportObject.index == BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0) //Left Nozzle
							{
								
								which_extruder=0;
								
							}
							else //Right Nozzle
							{
								
								which_extruder=1;
							}
							if (filament_mode == 'I') {
								if (which_extruder == 0){
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT1, 0);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PLA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_ABS, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PVA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM, 1);
								}
								else{
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT1, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0, 0);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PLA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_ABS, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PVA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM, 1);
								}
							}
							
							
							
							else {
								//*********Move the bed down and the extruders inside
								if(which_extruder == 0) setTargetHotend(unload_temp_l,which_extruder);
								else setTargetHotend(unload_temp_r,which_extruder);
								gif_processing_state = PROCESSING_DEFAULT;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
								
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
								if(gif_processing_state == PROCESSING_ERROR)return;
								
								gif_processing_state = PROCESSING_STOP;
								touchscreen_update();
								//genie.WriteObject(GENIE_OBJ_USERIMAGES,10,1);
								genie.WriteStr(STRING_UTILITIES_FILAMENT_CHANGEFILAMENT_TEMPS,"0%");
								genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_CHANGEFILAMENT_TEMPS,0);
								Tref1 = (int)degHotend(which_extruder);
								Tfinal1 = (int)degTargetHotend(which_extruder);
								/****************************************************/
								
								//ATTENTION : Order here is important
								
								
								//Serial.println("REMOVING");
								//genie.WriteStr(STRING_ADVISE_FILAMENT,"");
								/*if (filament_mode == 'I') genie.WriteObject(GENIE_OBJ_USERIMAGES,10,0);
								else if (filament_mode == 'R') genie.WriteObject(GENIE_OBJ_USERIMAGES,10,1);
								else genie.WriteObject(GENIE_OBJ_USERIMAGES,10,1);*/
								gif_processing_state = PROCESSING_CHANGE_FILAMENT_TEMPS;
								//delay(3500);
								/*if(which_extruder == 0) setTargetHotend(max(remove_temp_l,old_remove_temp_l),which_extruder);
								else setTargetHotend(max(remove_temp_r,old_remove_temp_r),which_extruder);*/
								touchscreen_update();
								delay(500);
								is_changing_filament=true; //We are changing filament
								
							}
							
						}
					}
					break;
					case BUTTON_UTILITIES_FILAMENT_LOAD_PLA:
					if (millis() >= waitPeriod_button_press){
						
						saved_print_flag = 888;
						if (which_extruder == 1) // Need to pause
						{
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							
							
							print_temp_r = PLA_PRINT_TEMP;
							load_temp_r = PLA_LOAD_TEMP;
							unload_temp_r = PLA_UNLOAD_TEMP;
							bed_temp_r = PLA_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend1(load_temp_r);
							insertmetod();
						}
						else if(which_extruder == 0){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							
							print_temp_l = PLA_PRINT_TEMP;
							load_temp_l = PLA_LOAD_TEMP;
							unload_temp_l = PLA_UNLOAD_TEMP;
							bed_temp_l = PLA_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend0(load_temp_l);
							insertmetod();
						}
						
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_ABS:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						saved_print_flag = 888;
						if (which_extruder == 1) // Need to pause
						{
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							
							
							print_temp_r = ABS_PRINT_TEMP;
							load_temp_r = ABS_LOAD_TEMP;
							unload_temp_r = ABS_UNLOAD_TEMP;
							bed_temp_r = ABS_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend1(load_temp_r);
							insertmetod();
						}
						else if(which_extruder == 0){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							
							print_temp_l = ABS_PRINT_TEMP;
							load_temp_l = ABS_LOAD_TEMP;
							unload_temp_l = ABS_UNLOAD_TEMP;
							bed_temp_l = ABS_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend0(load_temp_l);
							insertmetod();
						}
						
						
					}
					break;
					case BUTTON_UTILITIES_FILAMENT_LOAD_PVA:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						saved_print_flag = 888;
						if (which_extruder == 1) // Need to pause
						{
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							
							
							print_temp_r = PVA_PRINT_TEMP;
							load_temp_r = PVA_LOAD_TEMP;
							unload_temp_r = PVA_UNLOAD_TEMP;
							bed_temp_r = PVA_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend1(load_temp_r);
							insertmetod();
						}
						else if(which_extruder == 0){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							
							print_temp_l = PVA_PRINT_TEMP;
							load_temp_l = PVA_LOAD_TEMP;
							unload_temp_l = PVA_UNLOAD_TEMP;
							bed_temp_l = PVA_BED_TEMP;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_FIL_INSERTED,0);
							Config_StoreSettings();
							setTargetHotend0(load_temp_l);
							insertmetod();
						}
						
					}
					break;
					
					//CUSTOM MATERIAL BUTTONS
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM:
					if (millis() >= waitPeriod_button_press){
						
						if (which_extruder == 1 || which_extruder == 0) // Need to pause
						{
							if(Step_First_Start_Wizard){
								//genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_CUSTOM_MENU, 1);
							}
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_LOAD_CUSTOM,0);
							
							
							sprintf(buffer, "%d %cC",custom_insert_temp,0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_LOAD	,buffer);
							sprintf(buffer, "%d %cC",custom_remove_temp,0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_UNLOAD,buffer);
							sprintf(buffer, "%d %cC",custom_print_temp,0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_PRINT,buffer);
							sprintf(buffer, "%d %cC",custom_bed_temp,0x00B0);
							genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_BED,buffer);
						}
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_BACK:
					if (millis() >= waitPeriod_button_press){
						
						if(Step_First_Start_Wizard){
							if(which_extruder == 0){
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT1, 0);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0, 1);
								
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_MENU,1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_BACK,1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PLA, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_ABS, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PVA, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM, 1);
								}else if (which_extruder == 1){
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT1, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0, 0);
								
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_MENU,1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_BACK,1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PLA, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_ABS, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PVA, 1);
								genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM, 1);
							}
							
							}else{
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT1, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PLA, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_ABS, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PVA, 0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM, 0);
							
							
							which_extruder = -1;
						}
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_LOAD,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_INS_LESS:
					if (custom_insert_temp > 0){
						custom_insert_temp -= 10;
						sprintf(buffer, "%1d %cC",custom_insert_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_LOAD	,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_INS_MORE:
					if (custom_insert_temp < HEATER_0_MAXTEMP-5){
						custom_insert_temp += 10;
						sprintf(buffer, "%1d %cC",custom_insert_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_LOAD	,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_REM_LESS:
					if (custom_remove_temp > 0){
						custom_remove_temp -= 10;
						sprintf(buffer, "%1d %cC",custom_remove_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_UNLOAD,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_REM_MORE:
					if (custom_remove_temp < HEATER_0_MAXTEMP-5){
						custom_remove_temp += 10;
						sprintf(buffer, "%1d %cC",custom_remove_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_UNLOAD,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_PRINT_LESS:
					if (custom_print_temp > 0){
						custom_print_temp -= 10;
						sprintf(buffer, "%1d %cC",custom_print_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_PRINT,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_PRINT_MORE:
					if (custom_print_temp < HEATER_0_MAXTEMP-5){
						custom_print_temp += 10;
						sprintf(buffer, "%1d %cC",custom_print_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_PRINT,buffer);
					}
					break;
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_BED_LESS:
					if (custom_bed_temp > 0){
						custom_bed_temp -= 10;
						sprintf(buffer, "%1d %cC",custom_bed_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_BED,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_BED_MORE:
					if (custom_bed_temp < BED_MAXTEMP -5){
						custom_bed_temp += 10;
						sprintf(buffer, "%1d %cC",custom_bed_temp,0x00B0);
						genie.WriteStr(STRING_UTILITIES_FILAMENT_LOAD_CUSTOM_BED,buffer);
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM_ACCEPT:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						saved_print_flag = 888;
						if(which_extruder == 0){
							if (custom_print_temp <= HEATER_0_MAXTEMP) print_temp_l = custom_print_temp;
							else print_temp_l = HEATER_0_MAXTEMP;
							if (custom_insert_temp <= HEATER_0_MAXTEMP) load_temp_l = custom_insert_temp;
							else load_temp_l = HEATER_0_MAXTEMP;
							if (custom_remove_temp <= HEATER_0_MAXTEMP) unload_temp_l = custom_remove_temp;
							else unload_temp_l = HEATER_0_MAXTEMP;
							if (custom_bed_temp <= BED_MAXTEMP) bed_temp_l = custom_bed_temp;
							else bed_temp_l = BED_MAXTEMP;
							setTargetHotend0(load_temp_l);
						}
						else{
							if (custom_print_temp <= HEATER_1_MAXTEMP)print_temp_r = custom_print_temp;
							else print_temp_r = HEATER_1_MAXTEMP;
							if (custom_insert_temp <= HEATER_1_MAXTEMP)load_temp_r = custom_insert_temp;
							else print_temp_r = HEATER_1_MAXTEMP;
							if (custom_remove_temp <= HEATER_1_MAXTEMP)unload_temp_r = custom_remove_temp;
							else unload_temp_r = HEATER_1_MAXTEMP;
							if (custom_bed_temp <= BED_MAXTEMP)bed_temp_r = custom_bed_temp;
							else bed_temp_r = BED_MAXTEMP;
							setTargetHotend1(load_temp_r);
						}
						Config_StoreSettings();
						insertmetod();
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_HANDS:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						//ATTENTION : Order here is important
						genie.WriteStr(STRING_UTILITIES_FILAMENT_CHANGEFILAMENT_TEMPS,"0%");
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_CHANGEFILAMENT_TEMPS,0);
						Tref1 = (int)degHotend(which_extruder);
						Tfinal1 = (int)degTargetHotend(which_extruder);
						//genie.WriteStr(STRING_ADVISE_FILAMENT,"");
						//genie.WriteStr(STRING_ADVISE_FILAMENT,"Insert the filament until you feel it stops, \n then while you keep inserting around \n 10 mm of filament, press the clip");
						gif_processing_state = PROCESSING_CHANGE_FILAMENT_TEMPS;
						
						if (which_extruder==0) setTargetHotend(max(load_temp_l,old_load_temp_l),which_extruder);
						else setTargetHotend(max(load_temp_r,old_load_temp_r),which_extruder);
						//delay(3500);
						
						
						if (which_extruder == 0) changeTool(0);
						else changeTool(1);
						
						current_position[Y_AXIS] = 100;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						touchscreen_update();
						delay(500);
						is_changing_filament=true;
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_LOAD_KEEPPUSHING_NEXT:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						if (filament_mode =='I')
						{ //Inserting...
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							gif_processing_state = PROCESSING_DEFAULT;
							delay(550);
							#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
							current_position[X_AXIS] = 155;
							#else
							current_position[X_AXIS] = 155 + X_OFFSET_CALIB_PROCEDURES;
							#endif
							plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
							st_synchronize();
							SERIAL_PROTOCOLPGM("Loading\n");
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
							
							if(gif_processing_state == PROCESSING_ERROR)return;
							gif_processing_state = PROCESSING_STOP;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_ADJUST,0);
						}
					}
					break;
					
					
					case BUTTON_UTILITIES_FILAMENT_UNLOAD_ROLLTHESPOOL_NEXT:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						if (filament_mode =='R')
						{ //Removing...
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							gif_processing_state = PROCESSING_DEFAULT;
							delay(550);
							current_position[E_AXIS] -= (BOWDEN_LENGTH + EXTRUDER_LENGTH + 100);//Extra extrusion at fast feedrate
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_FAST_SPEED/60, which_extruder);
							previous_state = FORM_UTILITIES_FILAMENT;
							
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							gif_processing_state = PROCESSING_STOP;
							touchscreen_update();
							printer_state = STATE_LOADUNLOAD_FILAMENT;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_SUCCESS,0);
							genie.WriteObject(GENIE_OBJ_VIDEO,GIF_UTILITIES_FILAMENT_SUCCESS,0);
							gif_processing_state = PROCESSING_SUCCESS;
							if (which_extruder == 0){
								old_load_temp_l = load_temp_l;
								old_unload_temp_l = unload_temp_l;
								old_print_temp_l  = print_temp_l;
							}
							else{
								old_load_temp_r = load_temp_r;
								old_unload_temp_r = unload_temp_r;
								old_print_temp_r  = print_temp_r;
							}
						}
					}
					break;
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP0:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS,0);
						gif_processing_state = PROCESSING_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS;
						int Tref = (int)degHotend(which_extruder);
						int Tfinal = (int)(degTargetHotend(which_extruder)-NYLON_TEMP_HYSTERESIS);
						int percentage = 0;
						while (degHotend(which_extruder)<(degTargetHotend(which_extruder)-NYLON_TEMP_HYSTERESIS)){ //Waiting to heat the extruder
							if (millis() >= waitPeriod_s){
								int Tinstant;
								memset(buffer, '\0', sizeof(buffer) );
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
								genie.WriteStr(STRING_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS,buffer);
								waitPeriod_s=2000+millis();
							}
							manage_heater();
							touchscreen_update();
							if(gif_processing_state == PROCESSING_ERROR)return;
						}
						gif_processing_state = PROCESSING_STOP;
						touchscreen_update();
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP2,0);
						
					}
					break;
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP2:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						setTargetHotend(0.0,which_extruder);
						if(which_extruder == 0)digitalWrite(FAN_PIN, 1);
						else digitalWrite(FAN2_PIN, 1);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP3,0);
						//gif_processing_state = PROCESSING_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS;
						gif_processing_state = PROCESSING_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP3;
						int Tref = (int)degHotend(which_extruder);
						int Tfinal = 160;
						int percentage = 0;
						while (degHotend(which_extruder)>160.0){ //Waiting to heat the extruder
							//previous_millis_cmd = millis();
							memset(buffer, '\0', sizeof(buffer) );
							if (millis() >= waitPeriod_s){
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
								genie.WriteStr(STRING_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP3,buffer);
								waitPeriod_s=2000+millis();
							}
							manage_heater();
							touchscreen_update();
							if(gif_processing_state == PROCESSING_ERROR)return;
						}
						gif_processing_state = PROCESSING_STOP;
						touchscreen_update();
						fanSpeed=255;
						printer_state = STATE_NYLONCLEANING;
						#if BCN3D_SCREEN_VERSION_SETUP == BCN3D_SIGMA_PRINTER_DEVMODE_1
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP4,1);
						#endif
						
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP4,0);
						gif_processing_state = PROCESSING_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP4;
					}
					break;
					#if BCN3D_SCREEN_VERSION_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP4:
					if(printer_state == STATE_NYLONCLEANING){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							#if BCN3D_SCREEN_VERSION_SETUP == BCN3D_SIGMA_PRINTER_DEVMODE_1
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP4,0);
							#endif
							if(which_extruder == 0)digitalWrite(FAN_PIN, 1);
							else digitalWrite(FAN2_PIN, 1);
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS,0);
							gif_processing_state = PROCESSING_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS;
							int Tref = (int)degHotend(which_extruder);
							int Tfinal = NYLON_TEMP_COOLDOWN_THRESHOLD;
							int percentage = 0;
							while (degHotend(which_extruder)>Tfinal){ //Waiting to heat the extruder
								if (millis() >= waitPeriod_s){
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
									genie.WriteStr(STRING_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS,buffer);
									waitPeriod_s=2000+millis();
								}
								//previous_millis_cmd = millis();
								manage_heater();
								touchscreen_update();
								if(gif_processing_state == PROCESSING_ERROR)return;
								
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
									genie.WriteStr(STRING_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS,buffer);
									waitPeriod_s=2000+millis();
								}
								manage_heater();
								touchscreen_update();
								if(gif_processing_state == PROCESSING_ERROR)return;
								
							}
							gif_processing_state = PROCESSING_STOP;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP5,0);
							printer_state = STATE_NONE;
						}
					}
					break;
					#endif
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP5:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP6,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_REPEAT:
					
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						setTargetHotend(NYLON_TEMP_HEATUP_THRESHOLD,which_extruder);
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS,0);
						gif_processing_state = PROCESSING_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS;
						int Tref = (int)degHotend(which_extruder);
						int Tfinal = (int)(degTargetHotend(which_extruder)-NYLON_TEMP_HYSTERESIS);
						int percentage = 0;
						while (degHotend(which_extruder)<(degTargetHotend(which_extruder)-NYLON_TEMP_HYSTERESIS)){ //Waiting to heat the extruder
							if (millis() >= waitPeriod_s){
								memset(buffer, '\0', sizeof(buffer) );
								
								percentage = Tfinal-Tref;
								percentage = 100*((int)degHotend(which_extruder)-Tref)/percentage;
								sprintf(buffer, "%d%%", percentage);
								genie.WriteStr(STRING_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS,buffer);
								waitPeriod_s=2000+millis();
							}
							manage_heater();
							touchscreen_update();
							if(gif_processing_state == PROCESSING_ERROR)return;
						}
						gif_processing_state = PROCESSING_STOP;
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP2,0);
					}
					break;
					
					case BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_SUCCESS:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						doblocking = false;
						setTargetHotend0(0);
						setTargetHotend1(0);
						HeaterCooldownInactivity(true);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						gif_processing_state = PROCESSING_DEFAULT;
						home_axis_from_code(true, true, false);
						gif_processing_state == PROCESSING_STOP;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE,0);
						flag_utilities_maintenance_nyloncleaning = false;
					}
					break;
					
					#pragma endregion Insert_Remove_Fil
					
					//*****AdjustFilament******
					#pragma region AdjustFilament
					case BUTTON_UTILITIES_FILAMENT_ADJUST_ACCEPT:
					if(!flag_utilities_filament_acceptok){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							if(blocks_queued()) quickStop();
							if(which_extruder == 0) setTargetHotend(print_temp_l,which_extruder);
							else setTargetHotend(print_temp_r,which_extruder);
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							flag_utilities_filament_acceptok = true;
							home_made = false;
							gif_processing_state = PROCESSING_DEFAULT;
							home_axis_from_code(true,true,false);
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_SUCCESS_FILAMENT,0);
						}
						
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_ADJUST_LOAD:
					if(!flag_utilities_filament_acceptok){
						if(!blocks_queued()){
							flag_utilities_filament_purgeload = 1;
							}else{
							quickStop();
						}
						
					}
					break;
					
					case BUTTON_UTILITIES_FILAMENT_ADJUST_UNLOAD:
					if(!flag_utilities_filament_acceptok){
						if(!blocks_queued()){
							flag_utilities_filament_purgeunload = 1;
							}else{
							quickStop();
						}
					}
					break;
					#pragma endregion AdjustFilament
					
					//Extruder Calibrations-------------------------------------------------
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						bed_calibration_times = 0;
						if(saved_print_flag==1888){
							saved_print_flag = 888;
							Config_StoreSettings();
						}
						
						SERIAL_PROTOCOLPGM("INFO: BED CALIB - ");
						Serial.println(flag_utilities_calibration_calibbeddone);
						flag_utilities_calibration_calibfull = true;
						
						//enquecommand_P(PSTR("T0"));
						if(!flag_utilities_calibration_calibbeddone){  //Do g34
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							gif_processing_state = PROCESSING_DEFAULT;
							doblocking= true;
							home_axis_from_code(true,true,true);
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							enquecommand_P(PSTR("G34"));	//Start BED Calibration Wizard
							changeTool(0);
							
							
						}
						else{
							
							active_extruder = LEFT_EXTRUDER;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							gif_processing_state = PROCESSING_DEFAULT;
							setTargetHotend0(CALIBFULL_HOTEND_STANDBY_TEMP);
							setTargetHotend1(CALIBFULL_HOTEND_STANDBY_TEMP);
							setTargetBed(max(bed_temp_l,bed_temp_r));
							
							
							home_axis_from_code(true,true,false);
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							enquecommand_P(PSTR("T0"));
							gif_processing_state = PROCESSING_STOP;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZL,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZL_SKIP,1);
							}
							
						}
					}
					break;
					case BUTTON_UTILITIES_CALIBRATION_CALIBMANUALFINE:
					if (millis() >= waitPeriod_button_press){
						
						offset_calib_manu[0]=0.0;
						offset_calib_manu[1]=0.0;
						offset_calib_manu[2]=0.0;
						offset_calib_manu[3]=0.0;
						calib_value_selected = 0;
						sprintf(buffer, "0.000");
						
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_MANUAL,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_ZR,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_ZL,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_X,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_Y,0);
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_RIGHT,0);
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_LEFT,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_UP,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_DOWN,1);
						genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES_CALIBRATION_MANUAL,0);
						genie.WriteStr(STRING_UTILITIES_CALIBRATION_MANUAL,manual_fine_calib_offset[0],3);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
					}
					break;
					
					
					//*****Bed Calibration*****
					#pragma region Bed Calibration
					case BUTTON_UTILITIES_CALIBRATION_CALIBBED:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						if(saved_print_flag==1888){
							saved_print_flag = 888;
							Config_StoreSettings();
						}
						doblocking = true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						gif_processing_state = PROCESSING_DEFAULT;
						bed_calibration_times = 0;
						flag_utilities_calibration_calibfull = false;
						home_axis_from_code(true,true,true);
						enquecommand_P(PSTR("G34"));	//Start BED Calibration Wizard
						changeTool(0);
						previous_state = FORM_UTILITIES_CALIBRATION;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBBED_SCREW3_NEXT:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						doblocking = true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						gif_processing_state = PROCESSING_DEFAULT;
						home_axis_from_code(true,true,false);
						changeTool(0);
						enquecommand_P((PSTR("G34")));
						previous_state = FORM_UTILITIES_CALIBRATION;
						flag_utilities_calibration_calibbeddone = true;
					}
					break;
					
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBBED_SCREW2_NEXT:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						if (vuitens3!=0){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBBED_SCREW3,0);
							sprintf(buffer, " %d / 8",vuitens3); //Printing how to calibrate on screen
							//genie.WriteStr(STRING_BED_SCREW3,buffer);
							if (sentit3>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_CALIBBED_SCREW3,vuitens3);} //The direction is inverted in Sigma's bed screws
							else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_CALIBBED_SCREW3,vuitens3+8);}
							}else{
							doblocking = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							gif_processing_state = PROCESSING_DEFAULT;
							home_axis_from_code(true,true,false);
							changeTool(0);
							enquecommand_P((PSTR("G34")));
							previous_state = FORM_UTILITIES_CALIBRATION;
							
							flag_utilities_calibration_calibbeddone = true;
						}
					}
					break;
					case BUTTON_UTILITIES_CALIBRATION_CALIBBED_ADJUSTSCREWASK_NEXT:
					case BUTTON_UTILITIES_CALIBRATION_CALIBBED_ADJUSTSCREWASKFIRST_NEXT:
					if (millis() >= waitPeriod_button_press){
						
						gif_processing_state = PROCESSING_STOP;
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
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBBED_SCREW2,0);
							sprintf(buffer, " %d / 8",vuitens2); //Printing how to calibrate on screen
							//genie.WriteStr(STRING_BED_SCREW2,buffer);
							if (vuitens3==0) genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_CALIBBED_SCREW2_NEXT,0);
							if (sentit2>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_CALIBBED_SCREW2,vuitens2);} //The direction is inverted in Sigma's bed screws
							else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_CALIBBED_SCREW2,vuitens2+8);}
						}
						else if (vuitens3!= 0){
							SERIAL_PROTOCOLPGM("Jump over screw1 and screw2 \n");
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBBED_SCREW3,0);
							sprintf(buffer, " %d / 8",vuitens3); //Printing how to calibrate on screen
							//genie.WriteStr(STRING_BED_SCREW3,buffer);
							if (sentit3>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_CALIBBED_SCREW3,vuitens3);} //The direction is inverted in Sigma's bed screws
							else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_CALIBBED_SCREW3,vuitens3+8);}
						}
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					#pragma endregion Bed Calibration
					//*****Success Screens*****
					#pragma region SuccessScreens
					case BUTTON_UTILITIES_CALIBRATION_SUCCESS: // or BUTTON_SUCCESS_FILAMENT_OK
					if (printer_state == STATE_CALIBRATION){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							gif_processing_state = PROCESSING_STOP;
							if(flag_utilities_maintenance_autotune){
								genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE,0);
								flag_utilities_maintenance_autotune = false;
							}
							else{
								
								//enquecommand_P((PSTR("G28 X0 Y0")));
								genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
								gif_processing_state = PROCESSING_DEFAULT;
								setTargetHotend0(0);
								setTargetHotend1(0);
								setTargetBed(0);
								home_axis_from_code(true, true, false);
								enquecommand_P((PSTR("T0")));
								st_synchronize();
								if(gif_processing_state == PROCESSING_ERROR)return;
								SERIAL_PROTOCOLPGM("Calibration Successful\n");
								gif_processing_state = PROCESSING_STOP;
								
								genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION,0);
								
								flag_utilities_calibration_calibbeddone = true;
								
								doblocking=false;
							}
							
						}
						printer_state = STATE_NONE;
					}
					else if(printer_state == STATE_LOADUNLOAD_FILAMENT){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							gif_processing_state = PROCESSING_STOP;
							if(Step_First_Start_Wizard){
								if(which_extruder == 0){
									enquecommand_P((PSTR("T0")));
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT1, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0, 0);
									genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_MENU,1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_BACK,1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PLA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_ABS, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PVA, 1);
									genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM, 1);
									genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_LOAD,0);
									
									which_extruder = 1;
								}
								else if (which_extruder == 1){
									enquecommand_P((PSTR("T0")));
									genie.WriteObject(GENIE_OBJ_FORM,FORN_SETUPASSISTANT_STEP_2,0);
									which_extruder = 0;
								}
							}
							else if(!flag_utilities_maintenance_nyloncleaning){
								enquecommand_P((PSTR("T0")));
								genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT,0);
								HeaterCooldownInactivity(true);
							}
							else{
								genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
								gif_processing_state = PROCESSING_DEFAULT;
								setTargetHotend0(0);
								setTargetHotend1(0);
								home_axis_from_code(true, true, false);
								setTargetHotend(NYLON_TEMP_HEATUP_THRESHOLD,which_extruder);
								if (which_extruder == 0) changeTool(0);
								else changeTool(1);
								
								current_position[Y_AXIS] = 10;
								#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
								current_position[X_AXIS] = 155;
								#else
								current_position[X_AXIS] = 155 + 100;
								#endif
								doblocking = true;
								plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED*1.5,which_extruder);
								st_synchronize();
								if(gif_processing_state == PROCESSING_ERROR)return;
								gif_processing_state = PROCESSING_STOP;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP0,0);
							}
							flag_utilities_filament_acceptok = false;
							printer_state = STATE_NONE;
						}
					}
					#if BCN3D_SCREEN_VERSION_SETUP == BCN3D_SIGMA_PRINTER_DEVMODE_1
					else if(printer_state == STATE_NYLONCLEANING){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP4,0);
							if(which_extruder == 0)digitalWrite(FAN_PIN, 1);
							else digitalWrite(FAN2_PIN, 1);
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS,0);
							gif_processing_state = PROCESSING_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS;
							int Tref = (int)degHotend(which_extruder);
							int Tfinal = NYLON_TEMP_COOLDOWN_THRESHOLD;
							int percentage = 0;
							while (degHotend(which_extruder)>Tfinal){ //Waiting to heat the extruder
								if (millis() >= waitPeriod_s){
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
									genie.WriteStr(STRING_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS,buffer);
									waitPeriod_s=2000+millis();
								}
								//previous_millis_cmd = millis();
								manage_heater();
								touchscreen_update();
								if(gif_processing_state == PROCESSING_ERROR)return;
								
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
									genie.WriteStr(STRING_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS,buffer);
									waitPeriod_s=2000+millis();
								}
								manage_heater();
								touchscreen_update();
								if(gif_processing_state == PROCESSING_ERROR)return;
								
							}
							gif_processing_state = PROCESSING_STOP;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP5,0);
							printer_state = STATE_NONE;
						}
					}
					#endif
					break;
					#pragma endregion SuccessScreens
					
					//***** Calibration XYZ *****
					#pragma region CalibrationsXYZ
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSX_SELECT1:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_X_set(0.5);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSX_SELECT2:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_X_set(0.4);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSX_SELECT3:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_X_set(0.3);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSX_SELECT4:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_X_set(0.2);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSX_SELECT5:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_X_set(0.1);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSX_SELECT6:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_X_set(0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSX_SELECT7:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_X_set(-0.1);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSX_SELECT8:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_X_set(-0.2);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSX_SELECT9:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_X_set(-0.3);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSX_SELECT10:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_X_set(-0.4);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDO_X_LEFT:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
						redo_source = 1;
						float calculus = extruder_offset[X_AXIS][1] + 0.5;
						SERIAL_PROTOCOLPGM("Calculus:  ");
						Serial.println(calculus);
						extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
						Config_StoreSettings(); //Store changes
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDO_X_RIGHT:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
						
						redo_source = 1;
						float calculus = extruder_offset[X_AXIS][1] -0.4;
						SERIAL_PROTOCOLPGM("Calculus:  ");
						Serial.println(calculus);
						extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
						Config_StoreSettings(); //Store changes
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDO_X:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
						redo_source = 1;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSY_SELECT1:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Full_calibration_Y_set(0.5);
						
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSY_SELECT2:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Full_calibration_Y_set(0.4);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSY_SELECT3:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Full_calibration_Y_set(0.3);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSY_SELECT4:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Full_calibration_Y_set(0.2);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSY_SELECT5:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Full_calibration_Y_set(0.1);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSY_SELECT6:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Full_calibration_Y_set(0);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSY_SELECT7:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Full_calibration_Y_set(-0.1);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSY_SELECT8:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Full_calibration_Y_set(-0.2);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSY_SELECT9:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Full_calibration_Y_set(-0.3);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSY_SELECT10:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Full_calibration_Y_set(-0.4);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDO_Y_UP:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
						redo_source = 2;
						float calculus = extruder_offset[Y_AXIS][1] + 0.5;
						SERIAL_PROTOCOLPGM("Calculus:  ");
						Serial.println(calculus);
						extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
						Config_StoreSettings(); //Store changes
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDO_Y_DOWN:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
						redo_source = 2;
						float calculus = extruder_offset[Y_AXIS][1] -0.4;
						SERIAL_PROTOCOLPGM("Calculus:  ");
						Serial.println(calculus);
						extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
						Config_StoreSettings(); //Store changes
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDO_Y:
					if (millis() >= waitPeriod_button_press){
						redo_source = 2;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_CALIBZL_DOWN:
					if (millis() >= waitPeriod_button_press){
						
						feedrate = homing_feedrate[Z_AXIS];
						current_position[Z_AXIS] += 0.05;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
						SERIAL_PROTOCOLPGM("Z position: ");
						Serial.println(current_position[Z_AXIS]);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_CALIBZL_UP:
					if (millis() >= waitPeriod_button_press){
						feedrate = homing_feedrate[Z_AXIS];
						if (current_position[Z_AXIS]>-1.5) current_position[Z_AXIS] -= 0.05; //Max down is Z=-0.5
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
						SERIAL_PROTOCOLPGM("Z position: ");
						Serial.println(current_position[Z_AXIS]);
						
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_CALIBZL_ACCEPT:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						gif_processing_state = PROCESSING_STOP;
						touchscreen_update();
						setTargetHotend0(print_temp_l);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);						
						gif_processing_state = PROCESSING_DEFAULT;
						
						SERIAL_PROTOCOLPGM("OK first Extruder! \n");
						//We have to override z_prove_offset
						zprobe_zoffset-=(current_position[Z_AXIS]); //We are putting more offset if needed
						extruder_offset[Z_AXIS][LEFT_EXTRUDER]=0.0;//It is always the reference
						SERIAL_PROTOCOLPGM("Z1 Probe offset: ");
						Serial.println(zprobe_zoffset);
						Config_StoreSettings(); //Store changes
						
						current_position[Z_AXIS] += 2;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],homing_feedrate[Z_AXIS]/60,active_extruder);
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						home_axis_from_code(true,false,false);
						gif_processing_state = PROCESSING_STOP;
						touchscreen_update();
						flag_utilities_calibration_bedcomensationmode = true;
						Calib_check_temps();
						flag_utilities_calibration_bedcomensationmode = false;
						gif_processing_state = PROCESSING_STOP;
						touchscreen_update();
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_PRINTINGTEST,0);
						gif_processing_state = PROCESSING_TEST;
						
						
						home_axis_from_code(false,true,true);
						if(gif_processing_state == PROCESSING_ERROR)return;
						z_test_print_code(LEFT_EXTRUDER,0);
						enquecommand_P(PSTR("M84"));
						if(gif_processing_state == PROCESSING_ERROR)return;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_CALIBZR_DOWN:
					feedrate = homing_feedrate[Z_AXIS];
					current_position[Z_AXIS] += 0.05;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					SERIAL_PROTOCOLPGM("Z position: ");
					Serial.println(current_position[Z_AXIS]);
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_CALIBZR_UP:
					feedrate = homing_feedrate[Z_AXIS];
					if (current_position[Z_AXIS]>-1.5) current_position[Z_AXIS] -= 0.05;  //Max down is Z=-0.5
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					SERIAL_PROTOCOLPGM("Z position: ");
					Serial.println(current_position[Z_AXIS]);
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_CALIBZR_ACCEPT:
					if (millis() >= waitPeriod_button_press){
						gif_processing_state = PROCESSING_STOP;
						touchscreen_update();
						setTargetHotend1(print_temp_r);
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						gif_processing_state = PROCESSING_DEFAULT;
						
						
						SERIAL_PROTOCOLLNPGM("OK second Extruder!");
						extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=(current_position[Z_AXIS]);//Add the difference to the current offset value
						SERIAL_PROTOCOLPGM("Z2 Offset: ");
						Serial.println(extruder_offset[Z_AXIS][RIGHT_EXTRUDER]);
						Config_StoreSettings(); //Store changes
						
						current_position[Z_AXIS] += 2;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],homing_feedrate[Z_AXIS]/60,active_extruder);
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						home_axis_from_code(true,false,false);
						gif_processing_state = PROCESSING_STOP;
						touchscreen_update();
						flag_utilities_calibration_bedcomensationmode = true;
						Calib_check_temps();
						flag_utilities_calibration_bedcomensationmode = false;
						gif_processing_state = PROCESSING_STOP;
						touchscreen_update();
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_PRINTINGTEST,0);
						gif_processing_state = PROCESSING_TEST;
						
						
						
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						
						home_axis_from_code(false,true,true);
						if(gif_processing_state == PROCESSING_ERROR)return;
						z_test_print_code(RIGHT_EXTRUDER,32);
						enquecommand_P(PSTR("M84"));
						if(gif_processing_state == PROCESSING_ERROR)return;
					}
					break;
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL_REDO:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES_CALIBRATION_CALIBFULL_REDOZL,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZR_REDO:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES_CALIBRATION_CALIBFULL_REDOZR,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL_SELECT1:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_ZL_set(0.1);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL_SELECT2:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_ZL_set(0.05);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL_SELECT3:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_ZL_set(0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL_SELECT4:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_ZL_set(-0.05);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL_SELECT5:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_ZL_set(-0.1);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDOZL_RECALIBRATE:
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDOZR_RECALIBRATE:
					if (millis() >= waitPeriod_button_press){
						redo_source = 3;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
						current_position[E_AXIS] -= 4;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 15, active_extruder);//move first extruder
						st_synchronize();
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						doblocking = true;
						if(redo_source == 0){		 //redo z test print
							if (active_extruder==0){
								setTargetHotend0(print_temp_l);
								flag_utilities_calibration_bedcomensationmode = true;
								Calib_check_temps();
								flag_utilities_calibration_bedcomensationmode = false;
								gif_processing_state = PROCESSING_STOP;
								touchscreen_update();
								genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_PRINTINGTEST,0);
								gif_processing_state = PROCESSING_TEST;
								home_axis_from_code(true,true,true);
								if(gif_processing_state == PROCESSING_ERROR)return;
								z_test_print_code(LEFT_EXTRUDER,0);
								enquecommand_P(PSTR("M84"));
								if(gif_processing_state == PROCESSING_ERROR)return;
							}
							else{
								setTargetHotend1(print_temp_r);
								flag_utilities_calibration_bedcomensationmode = true;
								Calib_check_temps();
								flag_utilities_calibration_bedcomensationmode = false;
								gif_processing_state = PROCESSING_STOP;
								touchscreen_update();
								genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_PRINTINGTEST,0);
								gif_processing_state = PROCESSING_TEST;
								home_axis_from_code(true,true,true);
								if(gif_processing_state == PROCESSING_ERROR)return;
								z_test_print_code(RIGHT_EXTRUDER,32);
								enquecommand_P(PSTR("M84"));
								if(gif_processing_state == PROCESSING_ERROR)return;
								
							}
						}
						else if(redo_source == 1){ //redo x test print
							setTargetHotend1(print_temp_r);
							setTargetHotend0(print_temp_l);
							flag_utilities_calibration_bedcomensationmode = true;
							Calib_check_temps();
							flag_utilities_calibration_bedcomensationmode = false;
							gif_processing_state = PROCESSING_STOP;
							touchscreen_update();
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_PRINTINGTEST,0);
							gif_processing_state = PROCESSING_TEST;
							home_axis_from_code(true,true,false);
							current_position[Z_AXIS] = 0.2;
							plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
							enquecommand_P(PSTR("G40"));
						}
						else if(redo_source == 2){ //redo y test print
							setTargetHotend1(print_temp_r);
							setTargetHotend0(print_temp_l);
							flag_utilities_calibration_bedcomensationmode = true;
							Calib_check_temps();
							flag_utilities_calibration_bedcomensationmode = false;
							gif_processing_state = PROCESSING_STOP;
							touchscreen_update();
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_PRINTINGTEST,0);
							gif_processing_state = PROCESSING_TEST;
							home_axis_from_code(true,true,false);
							current_position[Z_AXIS] = 0.3;
							plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
							enquecommand_P(PSTR("G41"));
							
						}
						else if(redo_source == 3){	//recalibrate
							if (active_extruder == 0){
								gif_processing_state = PROCESSING_DEFAULT;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
								current_position[E_AXIS]-=4;
								plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],7.5,LEFT_EXTRUDER);
								st_synchronize();
								home_axis_from_code(true,true,true);
								if(gif_processing_state == PROCESSING_ERROR)return;
								enquecommand_P(PSTR("G43"));
								gif_processing_state = PROCESSING_STOP;
							}
							else{
								gif_processing_state = PROCESSING_DEFAULT;
								genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
								current_position[E_AXIS]-=4;
								plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],7.5,RIGHT_EXTRUDER);
								st_synchronize();
								if(gif_processing_state == PROCESSING_ERROR)return;
								home_axis_from_code(true,true,true);
								if(gif_processing_state == PROCESSING_ERROR)return;
								enquecommand_P(PSTR("G43"));
								gif_processing_state = PROCESSING_STOP;
							}
						}
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDOZL_BEST1:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						redo_source = 0;
						zprobe_zoffset+=0.1;
						Config_StoreSettings(); //Store changes
						gcode_T0_T1_auto(0);
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDOZR_BEST1:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						redo_source = 0;
						extruder_offset[Z_AXIS][RIGHT_EXTRUDER]+=0.1;
						Config_StoreSettings(); //Store changes
						gcode_T0_T1_auto(1);
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDOZL_BEST5:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						redo_source = 0;
						zprobe_zoffset-=0.1;
						Config_StoreSettings(); //Store changes
						gcode_T0_T1_auto(0);
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDOZR_BEST5:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						redo_source = 0;
						extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=0.1;
						Config_StoreSettings(); //Store changes
						gcode_T0_T1_auto(1);
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDOZL:
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_REDOZR:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						redo_source = 0;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZR_SELECT1:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_ZR_set(0.1);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZR_SELECT2:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_ZR_set(0.05);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZR_SELECT3:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_ZR_set(0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZR_SELECT4:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_ZR_set(-0.05);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZR_SELECT5:
					if (millis() >= waitPeriod_button_press){
						Full_calibration_ZR_set(-0.1);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZL_GO:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						doblocking=true;
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						gif_processing_state = PROCESSING_DEFAULT;
						active_extruder = LEFT_EXTRUDER;
						//Wait until temperature it's okey
						
						
						home_axis_from_code(true,true,true);
						//changeTool(LEFT_EXTRUDER);
						gif_processing_state = PROCESSING_STOP;
						st_synchronize();
						setTargetHotend0(CALIBFULL_HOTEND_STANDBY_TEMP);
						setTargetHotend1(CALIBFULL_HOTEND_STANDBY_TEMP);
						setTargetBed(max(bed_temp_l,bed_temp_r));
						Calib_check_temps();						
						gif_processing_state = PROCESSING_DEFAULT;
						changeTool(0);
						current_position[Z_AXIS] = 60;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 15, LEFT_EXTRUDER);//move bed
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						
						#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
						current_position[X_AXIS] = 150;
						#else
						current_position[X_AXIS] = 150 + 100;
						#endif
						
						current_position[Y_AXIS] = 0;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 200, LEFT_EXTRUDER);//move first extruder
						st_synchronize();
						current_position[E_AXIS] -= 4;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 7.5, LEFT_EXTRUDER);//move first extruder
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						gif_processing_state = PROCESSING_STOP;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANNOZZLE0,0);
						
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZL_SKIP:
					if(!Step_First_Start_Wizard && !flag_utilities_calibration_bedcomensationmode){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZR,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZR_SKIP,1);
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZR_GO:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						doblocking=true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						gif_processing_state = PROCESSING_DEFAULT;
						active_extruder = RIGHT_EXTRUDER;
						//Wait until temperature it's okey
						
						home_axis_from_code(true,true,true);
						//changeTool(LEFT_EXTRUDER);
						gif_processing_state = PROCESSING_STOP;
						
						st_synchronize();
						setTargetHotend0(CALIBFULL_HOTEND_STANDBY_TEMP);
						setTargetHotend1(CALIBFULL_HOTEND_STANDBY_TEMP);
						setTargetBed(max(bed_temp_l,bed_temp_r));
						Calib_check_temps();
						
						gif_processing_state = PROCESSING_DEFAULT;
						changeTool(1);
						current_position[Z_AXIS] = 60;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 15, RIGHT_EXTRUDER);//move bed
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
						current_position[X_AXIS] = 170;
						#else
						current_position[X_AXIS] = 170 + 100;
						#endif
						
						current_position[Y_AXIS] = 0;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 200, RIGHT_EXTRUDER);//move first extruder
						st_synchronize();
						current_position[E_AXIS] -= 4;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 7.5, RIGHT_EXTRUDER);//move first extruder
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						gif_processing_state = PROCESSING_STOP;
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANNOZZLE1,0);
						
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZR_SKIP:
					if(!Step_First_Start_Wizard){
						if (millis() >= waitPeriod_button_press){
							Z_compensation_decisor();
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					break;

					#if BCN3D_SCREEN_VERSION_SETUP == BCN3D_SIGMA_PRINTER_DEVMODE_1
					case BUTTON_Z_COMPENSATION_SKIP:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBX,0);
						if(Step_First_Start_Wizard){
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBX_SKIP,1);
						}
					}
					break;
					
					case BUTTON_Z_COMPENSATION_INSTALL:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						flag_utilities_calibration_zcomensationmode_gauges = 1888;
						Z_compensation_coolingdown();
					}
					break;
					
					case BUTTON_Z_COMPENSATION_COMFIRMATION_YES:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						setTargetHotend0(CALIBFULL_HOTEND_STANDBY_TEMP);
						setTargetHotend1(CALIBFULL_HOTEND_STANDBY_TEMP);
						setTargetBed(max(bed_temp_l,bed_temp_r));
						if(extruder_offset[Z_AXIS][RIGHT_EXTRUDER]<0){
								extruder_offset[Z_AXIS][RIGHT_EXTRUDER] = 0;
								}else{
								zprobe_zoffset +=extruder_offset[Z_AXIS][RIGHT_EXTRUDER];
								extruder_offset[Z_AXIS][RIGHT_EXTRUDER] = 0;
							}
						
						surfing_utilities = true;
						
						if(flag_utilities_calibration_zcomensationmode_gauges == 1888){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBX,0);
							
							}else if(flag_utilities_calibration_zcomensationmode_gauges == 2888){
							
							bed_calibration_times = 0;
							if(saved_print_flag==1888){
								saved_print_flag = 888;
								Config_StoreSettings();
							}
							
							flag_utilities_calibration_calibfull_skipZcalib = true;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							gif_processing_state = PROCESSING_DEFAULT;
							doblocking= true;
							home_axis_from_code(true,true,true);
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							enquecommand_P(PSTR("G34"));	//Start BED Calibration Wizard
							changeTool(0);
						}
						flag_utilities_calibration_zcomensationmode_gauges = 888;
						Config_StoreSettings();
						
						
						
					}
					break;
					
					case BUTTON_Z_COMPENSATION_COMFIRMATION_NOT:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_Z_COMPENSATION_COMFIRMATION_SURECANCEL,0);
					}
					break;
					
					case BUTTON_Z_COMPENSATION_COMFIRMATION_SURECANCEL_YES:
					if (millis() >= waitPeriod_button_press){
						if (millis() >= waitPeriod_button_press){
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
							if(flag_utilities_calibration_zcomensationmode_gauges == 1888){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBX,0);
							setTargetHotend0(print_temp_l);
							setTargetHotend1(print_temp_r);
							setTargetBed(max(bed_temp_l,bed_temp_r));
							flag_utilities_calibration_zcomensationmode_gauges = 888;
							Config_StoreSettings();
							surfing_utilities = true;
							}else if(flag_utilities_calibration_zcomensationmode_gauges == 2888){
							genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN,0);
							flag_utilities_calibration_zcomensationmode_gauges = 888;
							Config_StoreSettings();
							
							}
						}
					}
					break;
					
					case BUTTON_RAFT_ADVISE_CANCEL:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_RAFT_ADVISE_INSTALL,0);
					}
					break;
					
					case BUTTON_RAFT_ADVISE_INSTALL_ACCEPT:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						flag_utilities_calibration_zcomensationmode_gauges = 2888;
						Z_compensation_coolingdown();
					}
					break;
					
					case BUTTON_Z_COMPENSATION_COMFIRMATION_SURECANCEL_NOT:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						Z_compensation_coolingdown();
					}
					break;
					
					#endif
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBX_GO:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						doblocking=true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						gif_processing_state = PROCESSING_DEFAULT;
						home_axis_from_code(true,true,true);
						gif_processing_state = PROCESSING_STOP;
						st_synchronize();
						setTargetHotend0(print_temp_l);
						setTargetHotend1(print_temp_r);
						setTargetBed(max(bed_temp_l,bed_temp_r));
						Calib_check_temps();
						gif_processing_state = PROCESSING_DEFAULT;
						if(gif_processing_state == PROCESSING_ERROR)return;
						changeTool(0);
						enquecommand_P(PSTR("G40"));
						if(gif_processing_state == PROCESSING_ERROR)return;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBX_SKIP:
					if(!Step_First_Start_Wizard){
						if (millis() >= waitPeriod_button_press){
							
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBY,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBY_SKIP,1);
							}
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBY_GO:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						doblocking=true;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						gif_processing_state = PROCESSING_DEFAULT;
						home_axis_from_code(true,true,true);
						gif_processing_state = PROCESSING_STOP;
						setTargetHotend0(print_temp_l);
						setTargetHotend1(print_temp_r);
						setTargetBed(max(bed_temp_l,bed_temp_r));
						st_synchronize();
						Calib_check_temps();
						gif_processing_state = PROCESSING_DEFAULT;
						changeTool(0);
						if(gif_processing_state == PROCESSING_ERROR)return;
						enquecommand_P(PSTR("G41"));
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBY_SKIP:
					if(!Step_First_Start_Wizard){
						if (millis() >= waitPeriod_button_press){
							
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_FORM,FORN_SETUPASSISTANT_SUCCESS,0);
								gif_processing_state = PROCESSING_SUCCESS_FIRST_RUN;
								FLAG_First_Start_Wizard = 888;
								Step_First_Start_Wizard = false;
								
								}else{
								printer_state = STATE_CALIBRATION;
								genie.WriteObject(GENIE_OBJ_VIDEO,GIF_UTILITIES_CALIBRATION_SUCCESS,0);
								genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_SUCCESS,0);
								gif_processing_state = PROCESSING_BED_SUCCESS;
							}
							flag_utilities_calibration_calibfull = false;
							waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						}
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_CLEANNOZZLE0:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						//processing =  true;
						//home_axis_from_code(true, true , false);
						enquecommand_P(PSTR("G43"));
						flag_continue_calib = false;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_CALIBFULL_CLEANNOZZLE1:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						
						//home_axis_from_code(true, true , false);
						enquecommand_P(PSTR("G43"));
						flag_continue_calib = false;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					#pragma endregion CalibrationsXYZ
					
					
					#pragma region Manual Fine Calibration
					
					case BUTTON_UTILITIES_CALIBRATION_MANUAL_BACK:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION,0);
						offset_calib_manu[0]=0.0;
						offset_calib_manu[1]=0.0;
						offset_calib_manu[2]=0.0;
						offset_calib_manu[3]=0.0;
						calib_value_selected = 0;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_MANUAL_MENU:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN,0);
						offset_calib_manu[0]=0.0;
						offset_calib_manu[1]=0.0;
						offset_calib_manu[2]=0.0;
						offset_calib_manu[3]=0.0;
						calib_value_selected = 0;
						screen_sdcard = false;
						surfing_utilities=false;
						surfing_temps = false;
						SERIAL_PROTOCOLPGM("Surfing 0\n");
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_MANUAL_OK:
					if (millis() >= waitPeriod_button_press){
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_MANUAL_SAVE,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_MANUAL_X:
					if (millis() >= waitPeriod_button_press){
						
						calib_value_selected = 0;
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_MANUAL,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_ZR,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_ZL,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_X,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_Y,0);
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_RIGHT,0);
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_LEFT,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_UP,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_DOWN,1);
						value = 0.0;
						value = manual_fine_calib_offset[calib_value_selected] + offset_calib_manu[calib_value_selected];
						genie.WriteStr(STRING_UTILITIES_CALIBRATION_MANUAL,value,3);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_MANUAL_Y:
					if (millis() >= waitPeriod_button_press){
						
						calib_value_selected = 1;
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_MANUAL,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_ZR,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_ZL,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_X,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_Y,1);
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_RIGHT,1);
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_LEFT,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_UP,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_DOWN,0);
						value = 0.0;
						value = manual_fine_calib_offset[calib_value_selected] + offset_calib_manu[calib_value_selected];
						genie.WriteStr(STRING_UTILITIES_CALIBRATION_MANUAL,value,3);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_MANUAL_ZL:
					if (millis() >= waitPeriod_button_press){
						
						calib_value_selected = 2;
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_MANUAL,2);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_ZR,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_ZL,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_X,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_Y,0);
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_RIGHT,1);
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_LEFT,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_UP,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_DOWN,0);
						value = 0.0;
						value = manual_fine_calib_offset[calib_value_selected] + offset_calib_manu[calib_value_selected];
						genie.WriteStr(STRING_UTILITIES_CALIBRATION_MANUAL,value,3);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_MANUAL_ZR:
					if (millis() >= waitPeriod_button_press){
						calib_value_selected = 3;
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_UTILITIES_CALIBRATION_MANUAL,2);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_ZR,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_ZL,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_X,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_Y,0);
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_RIGHT,1);
						//genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_LEFT,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_UP,0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_MANUAL_DOWN,0);
						value = 0.0;
						value = manual_fine_calib_offset[calib_value_selected] + offset_calib_manu[calib_value_selected];
						genie.WriteStr(STRING_UTILITIES_CALIBRATION_MANUAL,value,3);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_MANUAL_UP:
					
					value = 0.0;
					
					if(calib_value_selected == 1 || calib_value_selected == 0){
						if(offset_calib_manu[calib_value_selected] < 1.975) offset_calib_manu[calib_value_selected] += 0.025;
					}
					else{
						if(offset_calib_manu[calib_value_selected] < 0.200) offset_calib_manu[calib_value_selected] += 0.025;
					}
					value = offset_calib_manu[calib_value_selected]+manual_fine_calib_offset[calib_value_selected];
					genie.WriteStr(STRING_UTILITIES_CALIBRATION_MANUAL,value,3);
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_MANUAL_DOWN:
					value = 0.0;
					
					if(calib_value_selected == 1 || calib_value_selected == 0){
						if(offset_calib_manu[calib_value_selected] > -1.975) offset_calib_manu[calib_value_selected] -= 0.025;
					}
					else{
						if(offset_calib_manu[calib_value_selected] > -0.200) offset_calib_manu[calib_value_selected] -= 0.025;
					}
					value = offset_calib_manu[calib_value_selected]+manual_fine_calib_offset[calib_value_selected];
					
					genie.WriteStr(STRING_UTILITIES_CALIBRATION_MANUAL,value,3);
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_MANUAL_SAVE_OK:
					if (millis() >= waitPeriod_button_press){
						
						manual_fine_calib_offset[0] += offset_calib_manu[0];
						manual_fine_calib_offset[1] += offset_calib_manu[1];
						manual_fine_calib_offset[2] += offset_calib_manu[2];
						manual_fine_calib_offset[3] += offset_calib_manu[3];
						
						extruder_offset[X_AXIS][RIGHT_EXTRUDER]+= offset_calib_manu[0];
						extruder_offset[Y_AXIS][RIGHT_EXTRUDER]+= offset_calib_manu[1];
						zprobe_zoffset += offset_calib_manu[2];
						extruder_offset[Z_AXIS][RIGHT_EXTRUDER]+= offset_calib_manu[3] - offset_calib_manu[2];
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION,0);
						offset_calib_manu[0]=0.0;
						offset_calib_manu[1]=0.0;
						offset_calib_manu[2]=0.0;
						offset_calib_manu[3]=0.0;
						calib_value_selected = 0;
						Config_StoreSettings();
						Config_PrintSettings();
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_UTILITIES_CALIBRATION_MANUAL_SAVE_NOT:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION,0);
						offset_calib_manu[0]=0.0;
						offset_calib_manu[1]=0.0;
						offset_calib_manu[2]=0.0;
						offset_calib_manu[3]=0.0;
						calib_value_selected = 0;
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					
					#pragma endregion Manual Fine Calibration
					
					
					
					
					
					//***** Info Screens *****
					#pragma region Info Screens
					
					//Backing from INFO SCREENS
					case BUTTON_UTILITIES_CALIBRATION_BACK:
					if (millis() >= waitPeriod_button_press){
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					
					//SKIP BED CALIBRATION
					case BUTTON_UTILITIES_CALIBRATION_CALIBBED_ADJUSTSCREWASK_SKIP:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						gif_processing_state = PROCESSING_STOP;
						touchscreen_update();
						if (flag_utilities_calibration_calibfull){
							bed_calibration_times = 0;
							
							active_extruder = LEFT_EXTRUDER;
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							gif_processing_state = PROCESSING_DEFAULT;
							setTargetHotend0(print_temp_l);
							setTargetHotend1(print_temp_r);
							setTargetBed(max(bed_temp_l,bed_temp_r));
							
							
							home_axis_from_code(true,true,false);
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							enquecommand_P(PSTR("T0"));
							gif_processing_state = PROCESSING_STOP;
							touchscreen_update();
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZL,0);
							genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZL_SKIP,0);
							if(Step_First_Start_Wizard){
								genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZL_SKIP,1);
							}
							
						}
						else if(flag_utilities_calibration_bedcomensationmode){
							active_extruder = which_extruder;
							//genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
							//gif_processing_state = PROCESSING_DEFAULT;
							
							st_synchronize();
							
							Bed_Compensation_state = 2;
							Bed_compensation_redo_offset = 0;
							if(gif_processing_state == PROCESSING_ERROR)return;
							if(which_extruder==0){
								enquecommand_P(PSTR("T0"));
								}else{
								enquecommand_P(PSTR("T1"));
							}
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_CLEANBED,0);
						}
						else if(flag_utilities_calibration_calibfull_skipZcalib){
							flag_utilities_calibration_calibfull_skipZcalib = false;
							active_extruder = LEFT_EXTRUDER;
							setTargetHotend0(CALIBFULL_HOTEND_STANDBY_TEMP);
							setTargetHotend1(CALIBFULL_HOTEND_STANDBY_TEMP);
							setTargetBed(max(bed_temp_l,bed_temp_r));
							
							st_synchronize();
							if(gif_processing_state == PROCESSING_ERROR)return;
							enquecommand_P(PSTR("T0"));
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBX,0);
							
						}
						else{
							printer_state = STATE_CALIBRATION;
							genie.WriteObject(GENIE_OBJ_VIDEO,GIF_UTILITIES_CALIBRATION_SUCCESS,0);
							genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_SUCCESS,0);
							gif_processing_state = PROCESSING_BED_SUCCESS;
						}
					}
					break;
					
					case BUTTON_ERROR_OK:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
						
						if(printing_error_temps){
							gif_processing_state = PROCESSING_STOP;
							touchscreen_update();
							genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
							enquecommand_P(PSTR("G28 X0 Y0")); //Home X and Y
							back_home = true;
							home_made = false;
							screen_sdcard = false;
							surfing_utilities=false;
							surfing_temps = false;
							
							card.sdprinting = false;
							card.sdispaused = false;
							
							gif_processing_state = PROCESSING_DEFAULT;
							printing_error_temps = false;
						}
						else{
							gif_processing_state = PROCESSING_STOP;
							screen_sdcard = false;
							surfing_utilities=false;
							SERIAL_PROTOCOLPGM("Surfing 0\n");
							surfing_temps = false;
							HeaterCooldownInactivity(true);
							genie.WriteObject(GENIE_OBJ_FORM, FORM_MAIN, 0);
						}
						doblocking = false;
					}
					break;
					
					#pragma endregion Info Screens
					
					#pragma region Setup Assistant
					case BUTTON_SETUPASSISTANT_YES:
					if (millis() >= waitPeriod_button_press){
						
						surfing_utilities = true;
						Step_First_Start_Wizard = true;
						genie.WriteObject(GENIE_OBJ_FORM,FORN_SETUPASSISTANT_STEP_1,0);
						Config_ResetDefault();
						Config_StoreSettings();
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_SETUPASSISTANT_SKIP:
					if (millis() >= waitPeriod_button_press){
						
						FLAG_First_Start_Wizard = 888;
						Step_First_Start_Wizard = false;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN,0);
						Config_StoreSettings();
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_SETUPASSISTANT_STEP_NEXT_1:
					if (millis() >= waitPeriod_button_press){
						
						which_extruder = 0;
						filament_mode = 'I';
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT1, 0);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_SELECT0, 1);
						
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_MENU,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_FILAMENT_LOAD_BACK,1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PLA, 1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_ABS, 1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_PVA, 1);
						genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_UTILITIES_FILAMENT_LOAD_CUSTOM, 1);
						genie.WriteObject(GENIE_OBJ_FORM, FORM_UTILITIES_FILAMENT_LOAD, 1);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					case BUTTON_SETUPASSISTANT_STEP_NEXT_2:
					if (millis() >= waitPeriod_button_press){
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON2;
						
						SERIAL_PROTOCOLPGM("INFO: BED CALIB - ");
						Serial.println(flag_utilities_calibration_calibbeddone);
						flag_utilities_calibration_calibfull = true;
						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
						gif_processing_state = PROCESSING_DEFAULT;
						doblocking = true;
						home_axis_from_code(true,true,true);
						st_synchronize();
						if(gif_processing_state == PROCESSING_ERROR)return;
						enquecommand_P(PSTR("G34"));	//Start BED Calibration Wizard
						changeTool(0);
					}
					break;
					
					case BUTTON_INFO_SETUPASSISTANT:
					if (millis() >= waitPeriod_button_press){
						genie.WriteObject(GENIE_OBJ_VIDEO,GIF_SETUPASSISTANT_INIT,0);
						genie.WriteObject(GENIE_OBJ_FORM,FORN_SETUPASSISTANT_INIT,0);
						int j = 0;
						static uint32_t waitPeriod = millis(); //Processing back home
						while ( j<GIF_FRAMES_INIT_FIRST_RUN){
							if (millis() >= waitPeriod){
								
								genie.WriteObject(GENIE_OBJ_VIDEO,GIF_SETUPASSISTANT_INIT,j);
								j+=1;
								waitPeriod = GIF_FRAMERATE+millis();	//Every 5s
							}
							
							
							
						}
						genie.WriteObject(GENIE_OBJ_FORM,FORN_SETUPASSISTANT_YESNOT,0);
						waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
					}
					break;
					
					#pragma endregion Setup Assistant
					
					
					
					
				}//endswitch
				
				
				
			}// else
		}
		
		//USERBUTTONS------------------------------------------------------

		
		//FORMS--------------------------------------------------------
		if (Event.reportObject.object == GENIE_OBJ_FORM)
		{
			char buffer[256];
			int tHotend;
			int tHotend1;
			int tBed;
			switch(Event.reportObject.index){
				
				case FORM_SDLIST:
				if (millis() >= waitPeriod_button_press){
					
					genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_SDLIST_FOLDERBACK,0);
					genie.WriteObject(GENIE_OBJ_USERIMAGES, USERIMAGE_SDLIST_FOLDERFILE,0);
					screen_sdcard = true;
					flag_sdlist_goinit = true;
					waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
				}
				break;
				
				case FORM_SDPRINTING:
				if (millis() >= waitPeriod_button_press){
					
					is_on_printing_screen = true;
					surfing_utilities = false;
					genie.WriteStr(STRING_SDPRINTING_GCODE,namefilegcode);
					flag_sdprinting_dararefresh = true;
					waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
				}
				break;
				
				case FORM_MAIN:
				if (millis() >= waitPeriod_button_press){
					
					screen_sdcard = false;
					surfing_utilities=false;
					surfing_temps = false;
					processing_z_set = 255;
					touchscreen_update();
					SERIAL_PROTOCOLPGM("Surfing 0\n");
					waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
				}
				break;
				
				case FORM_UTILITIES:
				if (millis() >= waitPeriod_button_press){
					
					surfing_utilities=true;
					SERIAL_PROTOCOLPGM("Surfing 1\n");
					waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
				}
				break;
				
				case FORM_TEMP:
				if (millis() >= waitPeriod_button_press){
					
					surfing_temps = true;
					waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
				}
				break;
				
				case FORM_UTILITIES_FILAMENT_PURGE:
				if (millis() >= waitPeriod_button_press){
					
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
					waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
				}
				break;
				
				case FORM_INFO_UI:
				if (millis() >= waitPeriod_button_press){
					
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
					waitPeriod_button_press=millis()+WAITPERIOD_PRESS_BUTTON;
				}
				break;
			}
		}
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
			//Serial.print(i);
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
			//Serial.print(i);
		}
		//buffer[count]='\0';
		genie.WriteStr(stringfilename[jint],buffer);//Printing form
		genie.WriteStr(stringfiledur[jint],listsd.commandline2);//Printing form
		//Is a file
		//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
	}
	//Serial.println(buffer);
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
			filepointer = (filepointer == ((fileCnt-1)/SDFILES_LIST_NUM)*SDFILES_LIST_NUM )? 0 : filepointer + SDFILES_LIST_NUM;
			
			genie.WriteObject(GENIE_OBJ_VIDEO, GIF_SDLIST_SCROLLBAR,	filepointer*40/(((fileCnt-1)/SDFILES_LIST_NUM)*SDFILES_LIST_NUM));
			
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
	
	
	flag_sdlist_filesupdown= true;
	
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
			filepointer = (filepointer == 0) ? ((fileCnt-1)/SDFILES_LIST_NUM)*SDFILES_LIST_NUM : filepointer-SDFILES_LIST_NUM;
			
			genie.WriteObject(GENIE_OBJ_VIDEO, GIF_SDLIST_SCROLLBAR,	filepointer*40/(((fileCnt-1)/SDFILES_LIST_NUM)*SDFILES_LIST_NUM));
			
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
	flag_sdlist_filesupdown= true;

	memset(listsd.commandline2, '\0', sizeof(listsd.commandline2) );
}
inline void ListFileListINITSD(){
	genie.WriteObject(GENIE_OBJ_VIDEO, GIF_SDLIST_SCROLLBAR,0);
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
		gif_processing_state == PROCESSING_ERROR =  true;
		screen_sdcard = true;
		#endif
	}
	memset(listsd.commandline2, '\0', sizeof(listsd.commandline2) );

}
inline void ListFileSelect_find(){
	card.getfilename(filepointer);
	if (!card.filenameIsDir){
		folder_navigation_register(true);
		genie.WriteObject(GENIE_OBJ_FORM, FORM_SDLIST_CONFIRMATION,0);
		listsd.get_lineduration(true, NULL);
		(listsd.get_minutes() == -1) ? sprintf_P(listsd.commandline2, PSTR("")) : sprintf_P(listsd.commandline2, PSTR("%4d:%.2dh / %dg"),listsd.get_hours(), listsd.get_minutes(),listsd.get_filgramos1());
		setfilenames(6);
		
	}
	else{
		if (card.chdir(card.filename)!=-1){
			folder_navigation_register(true);
			flag_sdlist_gofolderback = true;
			genie.WriteObject(GENIE_OBJ_USERBUTTON, BUTTON_SDLIST_FOLDERBACK,1);
			genie.WriteObject(GENIE_OBJ_USERIMAGES, USERIMAGE_SDLIST_FOLDERFILE,1);
		}
		
		
	}
}
inline void ListFileSelect0(){
	if(card.cardOK)
	{
		flag_sdlist_filesupdown = false;
		ListFileSelect_find();
		flag_sdlist_filesupdown = true;
	}
}
inline void ListFileSelect1(){
	if(card.cardOK)
	{
		flag_sdlist_filesupdown = false;
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
		flag_sdlist_filesupdown = true;
	}
}
inline void ListFileSelect2(){
	if(card.cardOK)
	{
		flag_sdlist_filesupdown = false;
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
		flag_sdlist_filesupdown = true;
	}
}
inline void ListFileSelect3(){
	if(card.cardOK)
	{
		flag_sdlist_filesupdown = false;
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
		
		flag_sdlist_filesupdown = true;
	}
}
inline void ListFileSelect4(){
	if(card.cardOK)
	{
		flag_sdlist_filesupdown = false;
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
		flag_sdlist_filesupdown = true;
	}
}
/*
inline void ListFileSelect5(){
	if(card.cardOK)
	{
		flag_sdlist_filesupdown = false;
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
		flag_sdlist_filesupdown = true;
	}
}*/
inline void ListFileListENTERBACKFORLDERSD(){
	genie.WriteObject(GENIE_OBJ_VIDEO, GIF_SDLIST_SCROLLBAR,0);
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
		gif_processing_state == PROCESSING_ERROR =  true;
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
			//Serial.print(i);
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
			//Serial.print(i);
		}
		//buffer[count]='\0';
		genie.WriteStr(stringfilename[jint],buffer);//Printing form
		//Is a file
		//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
	}
	//Serial.println(buffer);
	
	
}
inline void insertmetod(){
	gif_processing_state = PROCESSING_DEFAULT;
	if(!card.sdispaused){
		//genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
		doblocking = true;
		if(Step_First_Start_Wizard){
			home_axis_from_code(false,true,false);
			home_axis_from_code(true,false,true);
			}else{
			if (!home_made) home_axis_from_code(true,true,true);
			else home_axis_from_code(true,true,false);
		}
		int feedrate;
		
		current_position[Z_AXIS]=Z_MAX_POS-15;
		feedrate=homing_feedrate[Z_AXIS];
		plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate*2/60, active_extruder); //check speed
		st_synchronize();
		if(gif_processing_state == PROCESSING_ERROR)return;
		/****************************************************/
	}
	gif_processing_state = PROCESSING_STOP;
	genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_FILAMENT_LOAD_FEELSTOPS,0);
	
}
inline void folder_navigation_register(bool upchdir){

	if(upchdir){
		workDir_vector[workDir_vector_lenght] = filepointer;
		workDir_vector_lenght++;
		
		}else{
		workDir_vector_lenght--;

	}
}
inline void Bed_Compensation_Set_Lines(int jint){
	if(Bed_Compensation_state == 2){
		Bed_Compensation_state = 3;
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_PRINTINGTEST,0);
		gif_processing_state = PROCESSING_TEST;
		Bed_Compensation_Lines_Selected[0]+=jint;
		#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
		bed_test_print_code(-84.0,-220.0, 0);
		#else
		bed_test_print_code(-184.0,-220.0, 0);
		#endif
		
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL,0);
		gif_processing_state = PROCESSING_STOP;
	}
	else if(Bed_Compensation_state == 3){
		Bed_Compensation_state = 4;
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_PRINTINGTEST,0);
		gif_processing_state = PROCESSING_TEST;
		Bed_Compensation_Lines_Selected[1]+=jint;
		#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
		bed_test_print_code(82.0,-220.0, 0);
		#else
		bed_test_print_code(192.0,-220.0, 0);
		#endif
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL,0);
		gif_processing_state = PROCESSING_STOP;
	}
	else if(Bed_Compensation_state == 4){
		Bed_Compensation_Lines_Selected[2]+=jint;
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBBED_ADJUSTSCREWASKFIRST,0);
		gif_processing_state = PROCESSING_BED_FIRST;
		
	}
	
}
inline void Bed_Compensation_Redo_Lines(int jint){
	if(Bed_Compensation_state == 2){
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_PRINTINGTEST,0);
		gif_processing_state = PROCESSING_TEST;
		Bed_Compensation_Lines_Selected[0]+=jint;
		bed_test_print_code(0, 0, Bed_Compensation_Lines_Selected[0]);
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL,0);
		gif_processing_state = PROCESSING_STOP;
	}
	else if(Bed_Compensation_state == 3){
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_PRINTINGTEST,0);
		gif_processing_state = PROCESSING_TEST;
		Bed_Compensation_Lines_Selected[1]+=jint;
		#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
		bed_test_print_code(-84.0,-220.0, Bed_Compensation_Lines_Selected[1]);
		#else
		bed_test_print_code(-184.0,-220.0, Bed_Compensation_Lines_Selected[1]);
		#endif
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL,0);
		gif_processing_state = PROCESSING_STOP;
	}
	else if(Bed_Compensation_state == 4){
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_PRINTINGTEST,0);
		gif_processing_state = PROCESSING_TEST;
		Bed_Compensation_Lines_Selected[2]+=jint;
		#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
		bed_test_print_code(82.0,-220.0, Bed_Compensation_Lines_Selected[2]);
		#else
		bed_test_print_code(192.0,-220.0, Bed_Compensation_Lines_Selected[2]);
		#endif
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_RESULTSZL,0);
		gif_processing_state = PROCESSING_STOP;
	}
	
}
inline void Z_compensation_decisor(void){
	#if BCN3D_SCREEN_VERSION_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
	genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBX,0);
	if(Step_First_Start_Wizard){
		genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBX_SKIP,1);
	}
	
	#elif BCN3D_SCREEN_VERSION_SETUP == BCN3D_SIGMA_PRINTER_DEVMODE_1
	if(abs(extruder_offset[Z_AXIS][RIGHT_EXTRUDER]) <=RAFT_Z_THRESHOLD){
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBX,0);
		if(Step_First_Start_Wizard){
			genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBX_SKIP,1);
		}
		}else{
		char offset_string[250]="";
		int offset_aprox;
		offset_aprox = (int)(abs(extruder_offset[Z_AXIS][RIGHT_EXTRUDER])*100.01)/5.0;
		char buffer[80];
		//sprintf_P(offset_string, PSTR("Your Sigma Z axis has been calibrated\n\nTo avoid first layer Z compensation in Mirror/Duplication Mode:\n1.Turn off the machine and install gauges \n    on %s Hotend to correct %d.%1d%1dmm\n2. Re-run a Full Calibration\n\nWarning: Hotends may be hot when turning off the machine\n "),
		sprintf_P(offset_string, PSTR("Install %d %s on the %s hotend"), offset_aprox, ((offset_aprox > 1)?"shims":"shim") , ((extruder_offset[Z_AXIS][RIGHT_EXTRUDER] < 0)?"right":"left"));
		/*if ((extruder_offset[Z_AXIS][RIGHT_EXTRUDER])<0){
		sprintf_P(offset_string, PSTR("RIGHT HOTEND %d.%1d%1d"),(int)(5*offset_aprox)/100,(int)((5*offset_aprox)/10)%10,(int)(5*offset_aprox)%10);
		}else{
		sprintf_P(offset_string, PSTR("LEFT HOTEND %d.%1d%1d"),(int)(5*offset_aprox)/100,(int)((5*offset_aprox)/10)%10,(int)(5*offset_aprox)%10);
		}*/
		
		genie.WriteObject(GENIE_OBJ_FORM,FORM_Z_COMPENSATION,0);
		genie.WriteStr(STRING_Z_OFFSET_BETWEEN_NOZZLES,offset_string);
	}
	
	#endif
}
inline void Calib_check_temps(void){
	static long waitPeriod_s = millis();
	if((abs(degHotend(LEFT_EXTRUDER)-degTargetHotend(LEFT_EXTRUDER))>5) || (abs(degHotend(RIGHT_EXTRUDER)-degTargetHotend(RIGHT_EXTRUDER))>5) || ((degTargetBed()-degBed())-degBed()> 2)){
		int Tref0 = (int)degHotend0();
		int Tref1 = (int)degHotend1();
		int Trefbed = (int)degBed();
		int Tfinal0 = (int)degTargetHotend(LEFT_EXTRUDER);
		int Tfinal1 = (int)degTargetHotend(RIGHT_EXTRUDER);
		int Tfinalbed = (int)(degTargetBed()-2);
		long percentage = 0;
		genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
		gif_processing_state = PROCESSING_ADJUSTING;
		while (((abs(degHotend(LEFT_EXTRUDER)-degTargetHotend(LEFT_EXTRUDER))>5) && Tfinal0!=0) || ((abs(degHotend(RIGHT_EXTRUDER)-degTargetHotend(RIGHT_EXTRUDER))>5) && Tfinal1!=0) || ((degTargetBed()-degBed())> 2)){ //Waiting to heat the extruder
			
			manage_heater();
			touchscreen_update();
			if(gif_processing_state == PROCESSING_ERROR)return;
			
			if (millis() >= waitPeriod_s){
				char buffer[25];
				memset(buffer, '\0', sizeof(buffer) );
				int Tinstanthot0, Tinstanthot1, Tinstantbed;
				
				if(Tref0<Tfinal0){
					
					if(Tref0 > (int)degHotend(LEFT_EXTRUDER)){
						Tinstanthot0 = Tref0;
						}else if((int)degHotend(LEFT_EXTRUDER) > Tfinal0){
						Tinstanthot0 = Tfinal1;
						}else if(Tfinal0==0){
						Tinstanthot0 = 0;
						Tref0 = 0;	
						}else{
						Tinstanthot0 = (int)degHotend(LEFT_EXTRUDER);
					}
				}
				else{
					if(Tref0 < (int)degHotend(LEFT_EXTRUDER)){
						Tinstanthot0 = Tref0;
						}else if((int)degHotend(LEFT_EXTRUDER) < Tfinal0){
						Tinstanthot0 = Tfinal0;
						}else if(Tfinal0==0){
						Tinstanthot0 = 0;
						Tref0 = 0;	
						}else{
						Tinstanthot0 = (int)degHotend(LEFT_EXTRUDER);
					}
				}
				if(Tref1<Tfinal1){
					
					if(Tref1 > (int)degHotend(RIGHT_EXTRUDER)){
						Tinstanthot1 = Tref1;
						}else if((int)degHotend(RIGHT_EXTRUDER) > Tfinal1){
						Tinstanthot1 = Tfinal1;
						}else if(Tfinal1==0){
						Tinstanthot1 = 0;
						Tref1 = 0;
						}else{
						Tinstanthot1 = (int)degHotend(RIGHT_EXTRUDER);
					}
				}
				else{
					if(Tref1 < (int)degHotend(RIGHT_EXTRUDER)){
						Tinstanthot1 = Tref1;
						}else if((int)degHotend(RIGHT_EXTRUDER) < Tfinal1){
						Tinstanthot1 = Tfinal1;
						}else if(Tfinal1==0){
						Tinstanthot1 = 0;
						Tref1 = 0;
						}else{
						Tinstanthot1 = (int)degHotend(RIGHT_EXTRUDER);
					}
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
								
				percentage = abs((long)Tfinal0-(long)Tref0)+abs((long)Tfinal1-(long)Tref1)+abs((long)Tfinalbed*5-(long)Trefbed*5);
				percentage= 100*(abs((long)Tinstanthot0-(long)Tref0)+abs((long)Tinstanthot1-(long)Tref1)+abs((long)Tinstantbed*5-(long)Trefbed*5))/percentage;
				
				sprintf(buffer, "%ld%%", percentage);
				genie.WriteStr(STRING_ADJUSTING_TEMPERATURES,buffer);
				waitPeriod_s=2000+millis();
			}
			
			
		}
		if(!flag_utilities_calibration_bedcomensationmode){
			gif_processing_state = PROCESSING_STOP;
			st_synchronize();
			genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
			gif_processing_state = PROCESSING_DEFAULT;
		}
	}
}
inline void Full_calibration_ZL_set(float offset){
	manual_fine_calib_offset[2]=0.0;
	manual_fine_calib_offset[3]=0.0;
	genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZR,0);
	if(Step_First_Start_Wizard){
		genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBZR_SKIP,1);
	}
	
	active_extruder = RIGHT_EXTRUDER;
	zprobe_zoffset+=offset;
	Config_StoreSettings(); //Store changes
}
inline void Full_calibration_ZR_set(float offset){
	manual_fine_calib_offset[3]=0.0;
	extruder_offset[Z_AXIS][RIGHT_EXTRUDER]+=offset;
	Z_compensation_decisor();
	Config_StoreSettings(); //Store changes
}
inline void Full_calibration_X_set(float offset){
	manual_fine_calib_offset[0]=0.0;
	genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBY,0);
	if(Step_First_Start_Wizard){
		genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_UTILITIES_CALIBRATION_CALIBFULL_GOCALIBY_SKIP,1);
	}
	float calculus = extruder_offset[X_AXIS][1]+offset;
	SERIAL_PROTOCOLPGM("Calculus:  ");
	Serial.println(calculus);
	extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
	Config_StoreSettings(); //Store changes
}
inline void Full_calibration_Y_set(float offset){
	setTargetHotend0(0);
	setTargetHotend1(0);
	setTargetBed(0);
	manual_fine_calib_offset[1]=0.0;
	if (!Step_First_Start_Wizard){
		printer_state = STATE_CALIBRATION;
		genie.WriteObject(GENIE_OBJ_VIDEO,GIF_UTILITIES_CALIBRATION_SUCCESS,0);
		genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES_CALIBRATION_SUCCESS,0);
		gif_processing_state = PROCESSING_BED_SUCCESS;
		
		//char buffer[30];
		float calculus = extruder_offset[Y_AXIS][1] + offset;
		SERIAL_PROTOCOLPGM("Calculus:  ");
		Serial.println(calculus);
		//sprintf(buffer, "M218 T1 X%f",calculus); //
		//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
		//enquecommand(buffer);
		extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
		//enquecommand_P((PSTR("M218 T1 X-0.5")));
		Config_StoreSettings(); //Store changes
		//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
		flag_utilities_calibration_calibfull = false;
		
		
		}else{
		genie.WriteObject(GENIE_OBJ_FORM,FORN_SETUPASSISTANT_SUCCESS,0);
		gif_processing_state = PROCESSING_SUCCESS_FIRST_RUN;
		FLAG_First_Start_Wizard = 888;
		Step_First_Start_Wizard = false;
		//char buffer[30];
		float calculus = extruder_offset[Y_AXIS][1] + offset;
		SERIAL_PROTOCOLPGM("Calculus:  ");
		Serial.println(calculus);
		//sprintf(buffer, "M218 T1 X%f",calculus); //
		//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
		//enquecommand(buffer);
		extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
		//enquecommand_P((PSTR("M218 T1 X-0.5")));
		Config_StoreSettings(); //Store changes
		//genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
		flag_utilities_calibration_calibfull = false;
	}
}
#if BCN3D_SCREEN_VERSION_SETUP == BCN3D_SIGMA_PRINTER_DEVMODE_1
inline void Z_compensation_coolingdown(void){
	char buffer[256];
	static long waitPeriod_s = millis();
	setTargetHotend1(0);
	setTargetHotend0(0);
	setTargetBed(0);
	Config_StoreSettings();
	which_extruder = (extruder_offset[Z_AXIS][RIGHT_EXTRUDER]<0) ? 1:0;
	if(degHotend(which_extruder)>NYLON_TEMP_COOLDOWN_THRESHOLD){
		genie.WriteObject(GENIE_OBJ_FORM,FORM_ADJUSTING_TEMPERATURES,0);
		if(which_extruder == 0)digitalWrite(FAN_PIN, 1);
		else digitalWrite(FAN2_PIN, 1);
		gif_processing_state = PROCESSING_ADJUSTING;
		int Tref = (int)degHotend(which_extruder);
		int Tfinal = NYLON_TEMP_COOLDOWN_THRESHOLD;
		int percentage = 0;
		while (degHotend(which_extruder)>Tfinal){ //Waiting to heat the extruder
			if (millis() >= waitPeriod_s){
				memset(buffer, '\0', sizeof(buffer) );
				int Tinstant;
				if(Tref < (int)degHotend(which_extruder)){
					Tinstant = Tref;
					}else if((int)degHotend(which_extruder) < Tfinal){
					Tinstant = Tfinal;
					}else{
					Tinstant = (int)degHotend(which_extruder);
				}
				
				percentage = ((Tref-Tfinal)-(Tinstant-Tfinal))*100; //<<<<<<<<<<<<<  0% TO 100%
				percentage = percentage/(Tref-Tfinal);
				sprintf(buffer, "%d%%", percentage);
				genie.WriteStr(STRING_ADJUSTING_TEMPERATURES,buffer);
				waitPeriod_s=2000+millis();
			}
			//previous_millis_cmd = millis();
			manage_heater();
			touchscreen_update();
			if(gif_processing_state == PROCESSING_ERROR)return;
			
		}
		gif_processing_state = PROCESSING_STOP;
		touchscreen_update();
	}
	char offset_string[250]="";
	int offset_aprox;
	offset_aprox = (int)(abs(extruder_offset[Z_AXIS][RIGHT_EXTRUDER])*100.01)/5.0;
	//sprintf_P(offset_string, PSTR("Your Sigma Z axis has been calibrated\n\nTo avoid first layer Z compensation in Mirror/Duplication Mode:\n1.Turn off the machine and install gauges \n    on %s Hotend to correct %d.%1d%1dmm\n2. Re-run a Full Calibration\n\nWarning: Hotends may be hot when turning off the machine\n "),
	/*sprintf_P(offset_string, PSTR("Remember install gauges\non %s Hotend to correct %d.%1d%1dmm"),
	((extruder_offset[Z_AXIS][RIGHT_EXTRUDER] < 0)?"right":"left"),
	(int)(5*offset_aprox)/100,(int)((5*offset_aprox)/10)%10,(int)(5*offset_aprox)%10);*/
	genie.WriteObject(GENIE_OBJ_FORM,FORM_PROCESSING,0);
	gif_processing_state = PROCESSING_DEFAULT;
	if(home_made_Z){
		home_axis_from_code(true, true, false);	
		}else{
			home_axis_from_code(true, true, true);		
	}
	changeTool(which_extruder);
	current_position[Z_AXIS]= 180;
	plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],15,which_extruder);
	current_position[Y_AXIS] = 100;
	#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_DEFAULT
	plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],XY_TRAVEL_SPEED*1.5,which_extruder);
	st_synchronize();
	current_position[X_AXIS] = 155;
	#else
	current_position[X_AXIS] = 155 + X_OFFSET_CALIB_PROCEDURES;
	#endif
	plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],XY_TRAVEL_SPEED*1.5,which_extruder);
	st_synchronize();
	sprintf_P(offset_string, PSTR("Install %d %s on the %s hotend"), offset_aprox, ((offset_aprox > 1)?"shims":"shim") , ((extruder_offset[Z_AXIS][RIGHT_EXTRUDER] < 0)?"right":"left"));
	Serial.println(offset_string);
	genie.WriteObject(GENIE_OBJ_FORM,FORM_Z_COMPENSATION_SHUTDOWN,0);
	genie.WriteStr(STRING_Z_COMPENSATION_SHUTDOWN,offset_string);
}
#endif
#endif /* INCLUDE */

#endif