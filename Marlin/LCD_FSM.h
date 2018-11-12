/*
- LCD_FSM.h - A class that manages the FSM related with some printer process
Last Update: 01/08/2018
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
#include "BCN3D_customregisters.h"



//#include "ultralcd.h"
extern int get_nummaxchars(bool isfilegcode, unsigned int totalpixels);
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
extern bool flag_temp_gifhotent0;
extern bool flag_temp_gifhotent1;
extern bool flag_temp_gifbed;
extern bool lcd_busy;
extern uint16_t flag_sdprinting_register;
extern uint8_t flag_utilities_filament_register;
extern uint8_t flag_utilities_maintenance_register;
extern uint16_t flag_sdlist_resgiter;
extern uint8_t flag_utilities_calibration_register;
extern int raft_advise_accept_cancel;//0 cancel ; 1 accept
extern int Temp_ChangeFilament_Saved;
extern int Tref1;
extern int Tfinal1;
extern int Tpercentaje_old;
extern int calib_confirmation;
extern int8_t calib_zxy;
extern int  print_setting_tool;
extern void Calib_check_temps(void);
extern void Z_compensation_decisor(void);
extern void Full_calibration_ZL_set(float offset);
extern void Full_calibration_ZR_set(float offset);
extern void Full_calibration_X_set(float offset);
extern void Full_calibration_Y_set(float offset);
extern void Coolingdown_Shutdown(int mode);

// Bed compensation
extern void Bed_Compensation_Set_Lines(int jint);
extern void Bed_Compensation_Redo_Lines(int jint);
extern int Bed_compensation_redo_offset;
extern int8_t Bed_Compensation_Lines_Selected[3];
extern uint8_t Bed_Compensation_state;// state 0: First Bed Calib, state 1: ZL Calib, state 2: Bed Compensation Back, state 3: Bed Compensation Front Left, state 4: Bed Compensation Front Right

// end Bed compensation

extern int redo_source;
extern bool check_regiter_num(unsigned int n);
extern void lcd_fsm_lcd_input_logic();
extern void lcd_fsm_output_logic();
extern long LCD_FSM_input_buton_flag;
extern int redo_source;

#endif

