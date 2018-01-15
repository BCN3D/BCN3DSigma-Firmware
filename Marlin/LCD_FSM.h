/*
- LCD_FSM.h - A class that manages the FSM related with some printer process
Last Update: 15/01/2017
Author: Alejandro Garcia (S3mt0x)
*/
#ifndef _LCD_FSM_h
#define _LCD_FSM_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


#include "Touch_Screen_Definitions.h"
#include "genieArduino.h"
#include "Marlin.h"
#include "Configuration.h"
#include "stepper.h"
#include "temperature.h"
#include "SD_ListFiles.h"
#include "language.h"




//#include "ultralcd.h"
extern void setfilenames(int jint);
extern void insertmetod();
extern void ListFilesParsingProcedure(uint16_t vecto, int jint);
extern void ListFilesUpfunc();
extern void ListFilesDownfunc();
extern void ListFileListINITSD();
extern void ListFileListENTERBACKFORLDERSD();
extern void ListFileSelect_find();
extern void ListFileSelect0();
extern void ListFileSelect1();
extern void ListFileSelect2();
extern void ListFileSelect3();
extern void ListFileSelect4();
extern void lcd_animation_handler();
extern void update_screen_printing();
extern void update_screen_sdcard();
extern void update_screen_endinggcode();
extern void update_screen_noprinting();
//inline void ListFileSelect5();
extern void setfoldernames(int jint);
extern void folder_navigation_register(bool upchdir);
extern bool flag_utilities_maintenance_nyloncleaning;
extern bool flag_utilities_maintenance_autotune;
extern bool flag_utilities_filament_home;
extern bool flag_utilities_filament_acceptok;
extern bool flag_utilities_filament_purgeselect0;
extern bool flag_utilities_filament_purgeselect1;
extern bool flag_utilities_filament_purgeload;
extern bool flag_utilities_filament_purgeunload;
extern bool flag_utilities_calibration_calibfull;
extern bool flag_utilities_calibration_calibfull_skipZcalib;
extern bool flag_utilities_calibration_calibbeddone;
extern bool flag_utilities_calibration_bedcomensationmode;
extern bool flag_temp_gifhotent0;
extern bool flag_temp_gifhotent1;
extern bool flag_temp_gifbed;
extern bool flag_sdprinting_pausepause;
extern bool flag_sdprinting_printstop;
extern bool flag_sdprinting_printsavejob;
extern bool flag_sdprinting_printpause;
extern bool flag_sdprinting_printresume;
extern bool flag_sdprinting_pauseresume;
extern bool flag_sdprinting_settings;
extern bool flag_sdprinting_showdata;
extern bool flag_sdprinting_dararefresh;
extern bool flag_sdprinting_printsavejobcommand;
extern bool flag_sdlist_filesupdown;
extern bool flag_sdlist_select0;
extern bool flag_sdlist_select1;
extern bool flag_sdlist_select2;
extern bool flag_sdlist_select3;
extern bool flag_sdlist_select4;
extern bool flag_sdlist_select5;
extern bool flag_sdlist_godown;
extern bool flag_sdlist_goup;
extern bool flag_sdlist_goinit;
extern bool flag_sdlist_gofolderback;
extern bool flag_maintenance_zdjust100up;
extern bool flag_maintenance_zdjust10up;
extern bool flag_maintenance_zdjust100down;
extern bool flag_maintenance_zdjust10down;
extern bool lcd_busy;
extern int raft_advise_accept_cancel;//0 cancel ; 1 accept
extern int Temp_ChangeFilament_Saved;
extern int Tref1;
extern int Tfinal1;
extern int  print_setting_tool;
extern void Calib_check_temps(void);
extern void Z_compensation_decisor(void);
extern void Full_calibration_ZL_set(float offset);
extern void Full_calibration_ZR_set(float offset);
extern void Full_calibration_X_set(float offset);
extern void Full_calibration_Y_set(float offset);
extern void Z_compensation_coolingdown(void);

// Bed compensation
extern void Bed_Compensation_Set_Lines(int jint);
extern void Bed_Compensation_Redo_Lines(int jint);
extern int Bed_compensation_redo_offset;
extern int8_t Bed_Compensation_Lines_Selected[3];
extern uint8_t Bed_Compensation_state;// state 0: First Bed Calib, state 1: ZL Calib, state 2: Bed Compensation Back, state 3: Bed Compensation Front Left, state 4: Bed Compensation Front Right

// end Bed compensation

extern int redo_source;
extern void lcd_fsm_lcd_input_logic();
extern void lcd_fsm_output_logic();
extern long LCD_FSM_input_buton_flag;
extern int redo_source;

#endif

