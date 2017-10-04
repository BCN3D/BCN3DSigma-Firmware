#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "ConfigurationStore.h"

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
	do
	{
		eeprom_write_byte((unsigned char*)pos, *value);
		pos++;
		value++;
	}while(--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
	do
	{
		*value = eeprom_read_byte((unsigned char*)pos);
		pos++;
		value++;
	}while(--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))
//======================================================================================




#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.

#define EEPROM_VERSION "V12"
/*#ifdef DELTA
#undef EEPROM_VERSION
#define EEPROM_VERSION "V11"
#endif
#ifdef SCARA
#undef EEPROM_VERSION
#define EEPROM_VERSION "V12"
#endif*/

#ifdef EEPROM_SETTINGS
void Config_StoreSettings()
{
	char ver[4]= "000";
	int i=EEPROM_OFFSET;
	EEPROM_WRITE_VAR(i,ver); // invalidate data first
	EEPROM_WRITE_VAR(i,axis_steps_per_unit);
	EEPROM_WRITE_VAR(i,max_feedrate);
	EEPROM_WRITE_VAR(i,max_acceleration_units_per_sq_second);
	EEPROM_WRITE_VAR(i,acceleration);
	EEPROM_WRITE_VAR(i,retract_acceleration);
	EEPROM_WRITE_VAR(i,minimumfeedrate);
	EEPROM_WRITE_VAR(i,mintravelfeedrate);
	EEPROM_WRITE_VAR(i,minsegmenttime);
	EEPROM_WRITE_VAR(i,max_xy_jerk);
	EEPROM_WRITE_VAR(i,max_z_jerk);
	EEPROM_WRITE_VAR(i,max_e_jerk);
	EEPROM_WRITE_VAR(i,add_homing);
	#ifdef DELTA
	EEPROM_WRITE_VAR(i,endstop_adj);
	EEPROM_WRITE_VAR(i,delta_radius);
	EEPROM_WRITE_VAR(i,delta_diagonal_rod);
	EEPROM_WRITE_VAR(i,delta_segments_per_second);
	#endif
	#ifndef ULTIPANEL
	int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP, plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP, plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
	int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP, absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP, absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
	#endif
	EEPROM_WRITE_VAR(i,plaPreheatHotendTemp);
	EEPROM_WRITE_VAR(i,plaPreheatHPBTemp);
	EEPROM_WRITE_VAR(i,plaPreheatFanSpeed);
	EEPROM_WRITE_VAR(i,absPreheatHotendTemp);
	EEPROM_WRITE_VAR(i,absPreheatHPBTemp);
	EEPROM_WRITE_VAR(i,absPreheatFanSpeed);
	EEPROM_WRITE_VAR(i,zprobe_zoffset);
	
	//Extruder Offset
	EEPROM_WRITE_VAR(i,extruder_offset[X_AXIS][RIGHT_EXTRUDER]);
	EEPROM_WRITE_VAR(i,extruder_offset[Y_AXIS][RIGHT_EXTRUDER]);
	EEPROM_WRITE_VAR(i,extruder_offset[Z_AXIS][RIGHT_EXTRUDER]);
	
	//Quick Start Guide
	//EEPROM_WRITE_VAR(i,quick_guide);
	EEPROM_WRITE_VAR(i,print_temp_l);
	EEPROM_WRITE_VAR(i,load_temp_l);
	EEPROM_WRITE_VAR(i,unload_temp_l);
	EEPROM_WRITE_VAR(i,bed_temp_l);
	EEPROM_WRITE_VAR(i,old_print_temp_l);
	EEPROM_WRITE_VAR(i,old_load_temp_l);
	EEPROM_WRITE_VAR(i,old_unload_temp_l);
	EEPROM_WRITE_VAR(i,old_bed_temp_l);
	
	EEPROM_WRITE_VAR(i,print_temp_r);
	EEPROM_WRITE_VAR(i,load_temp_r);
	EEPROM_WRITE_VAR(i,unload_temp_r);
	EEPROM_WRITE_VAR(i,bed_temp_r);
	EEPROM_WRITE_VAR(i,old_print_temp_r);
	EEPROM_WRITE_VAR(i,old_load_temp_r);
	EEPROM_WRITE_VAR(i,old_unload_temp_r);
	EEPROM_WRITE_VAR(i,old_bed_temp_r);
	
	
	//Language
	//  EEPROM_WRITE_VAR(i,language);

	
	#ifdef PIDTEMP
	EEPROM_WRITE_VAR(i,Kp[0]);
	EEPROM_WRITE_VAR(i,Ki[0]);
	EEPROM_WRITE_VAR(i,Kd[0]);
	EEPROM_WRITE_VAR(i,Kp[1]);
	EEPROM_WRITE_VAR(i,Ki[1]);
	EEPROM_WRITE_VAR(i,Kd[1]);
	/*#else
	float dummy = 3000.0f;
	EEPROM_WRITE_VAR(i,dummy);
	dummy = 0.0f;
	EEPROM_WRITE_VAR(i,dummy);
	EEPROM_WRITE_VAR(i,dummy);*/
	#endif
	EEPROM_WRITE_VAR(i,log_prints);
	EEPROM_WRITE_VAR(i,log_prints_finished);
	EEPROM_WRITE_VAR(i,log_max_temp_l);
	EEPROM_WRITE_VAR(i,log_max_temp_r);
	EEPROM_WRITE_VAR(i,log_hours_print);
	EEPROM_WRITE_VAR(i,log_max_bed);
	
	#ifndef DOGLCD
	int lcd_contrast = 32;
	#endif
	EEPROM_WRITE_VAR(i,lcd_contrast);
	#ifdef SCARA
	EEPROM_WRITE_VAR(i,axis_scaling);
	#endif
	EEPROM_WRITE_VAR(i,UI_SerialID0);
	EEPROM_WRITE_VAR(i,UI_SerialID1);
	EEPROM_WRITE_VAR(i,UI_SerialID2);
	EEPROM_WRITE_VAR(i,log_minutes_lastprint);
	EEPROM_WRITE_VAR(i,log_hours_lastprint);
	EEPROM_WRITE_VAR(i,log_X0_mmdone);
	EEPROM_WRITE_VAR(i,log_X1_mmdone);
	EEPROM_WRITE_VAR(i,log_Y_mmdone);
	EEPROM_WRITE_VAR(i,log_E0_mmdone);
	EEPROM_WRITE_VAR(i,log_E1_mmdone);
	EEPROM_WRITE_VAR(i,FLAG_First_Start_Wizard);
	EEPROM_WRITE_VAR(i,saved_print_flag);
	EEPROM_WRITE_VAR(i,saved_x_position);
	EEPROM_WRITE_VAR(i,saved_y_position);
	EEPROM_WRITE_VAR(i,saved_z_position);
	EEPROM_WRITE_VAR(i,saved_tool_active);
	EEPROM_WRITE_VAR(i,saved_e_position);
	EEPROM_WRITE_VAR(i,saved_fileposition);
	EEPROM_WRITE_VAR(i,saved_temp1);
	EEPROM_WRITE_VAR(i,saved_temp0);
	EEPROM_WRITE_VAR(i,saved_tempbed);
	EEPROM_WRITE_VAR(i,saved_fanlayer);
	EEPROM_WRITE_VAR(i,saved_feedmulti);
	EEPROM_WRITE_VAR(i,saved_workDir_vector_lenght);
	EEPROM_WRITE_VAR(i,saved_workDir_vector[0]);
	EEPROM_WRITE_VAR(i,saved_workDir_vector[1]);
	EEPROM_WRITE_VAR(i,saved_workDir_vector[2]);
	EEPROM_WRITE_VAR(i,saved_workDir_vector[3]);
	EEPROM_WRITE_VAR(i,saved_workDir_vector[4]);
	EEPROM_WRITE_VAR(i,saved_workDir_vector[5]);
	EEPROM_WRITE_VAR(i,saved_workDir_vector[6]);
	EEPROM_WRITE_VAR(i,saved_workDir_vector[7]);
	EEPROM_WRITE_VAR(i,saved_workDir_vector[8]);
	EEPROM_WRITE_VAR(i,saved_workDir_vector[9]);
	EEPROM_WRITE_VAR(i,acceleration_old);
	EEPROM_WRITE_VAR(i,version_number);
	EEPROM_WRITE_VAR(i,bed_offset_left_screw);
	EEPROM_WRITE_VAR(i,bed_offset_right_screw);
	EEPROM_WRITE_VAR(i,manual_fine_calib_offset[0]);
	EEPROM_WRITE_VAR(i,manual_fine_calib_offset[1]);
	EEPROM_WRITE_VAR(i,manual_fine_calib_offset[2]);
	EEPROM_WRITE_VAR(i,manual_fine_calib_offset[3]);
	EEPROM_WRITE_VAR(i,saved_dual_x_carriage_mode);
	EEPROM_WRITE_VAR(i,saved_duplicate_extruder_x_offset);
	EEPROM_WRITE_VAR(i,Flag_fanSpeed_mirror);
	EEPROM_WRITE_VAR(i,bed_offset_version);
	EEPROM_WRITE_VAR(i,flag_utilities_calibration_zcomensationmode_gauges);
	char ver2[4]=EEPROM_VERSION;
	i=EEPROM_OFFSET;
	EEPROM_WRITE_VAR(i,ver2); // validate data
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Settings Stored");
}
#endif //EEPROM_SETTINGS


#ifndef DISABLE_M503
void Config_PrintSettings()
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Steps per unit:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M92 X",axis_steps_per_unit[0]);
	SERIAL_ECHOPAIR(" Y",axis_steps_per_unit[1]);
	SERIAL_ECHOPAIR(" Z",axis_steps_per_unit[2]);
	SERIAL_ECHOPAIR(" E",axis_steps_per_unit[3]);
	SERIAL_ECHOLN("");
	
	SERIAL_ECHO_START;
	#ifdef SCARA
	SERIAL_ECHOLNPGM("Scaling factors:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M365 X",axis_scaling[0]);
	SERIAL_ECHOPAIR(" Y",axis_scaling[1]);
	SERIAL_ECHOPAIR(" Z",axis_scaling[2]);
	SERIAL_ECHOLN("");
	
	SERIAL_ECHO_START;
	#endif
	SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M203 X",max_feedrate[0]);
	SERIAL_ECHOPAIR(" Y",max_feedrate[1] );
	SERIAL_ECHOPAIR(" Z", max_feedrate[2] );
	SERIAL_ECHOPAIR(" E", max_feedrate[3]);
	SERIAL_ECHOLN("");

	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M201 X" ,max_acceleration_units_per_sq_second[0] );
	SERIAL_ECHOPAIR(" Y" , max_acceleration_units_per_sq_second[1] );
	SERIAL_ECHOPAIR(" Z" ,max_acceleration_units_per_sq_second[2] );
	SERIAL_ECHOPAIR(" E" ,max_acceleration_units_per_sq_second[3]);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Acceleration: S=acceleration, T=retract acceleration");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M204 S",acceleration );
	SERIAL_ECHOPAIR(" T" ,retract_acceleration);
	SERIAL_ECHOLN("");

	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M205 S",minimumfeedrate );
	SERIAL_ECHOPAIR(" T" ,mintravelfeedrate );
	SERIAL_ECHOPAIR(" B" ,minsegmenttime );
	SERIAL_ECHOPAIR(" X" ,max_xy_jerk );
	SERIAL_ECHOPAIR(" Z" ,max_z_jerk);
	SERIAL_ECHOPAIR(" E" ,max_e_jerk);
	SERIAL_ECHOLN("");

	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Home offset (mm):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M206 X",add_homing[0] );
	SERIAL_ECHOPAIR(" Y" ,add_homing[1] );
	SERIAL_ECHOPAIR(" Z" ,add_homing[2] );
	SERIAL_ECHOLN("");

	#ifdef PIDTEMP
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("PID settings:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M301 LEFT P",Kp[0]);
	SERIAL_ECHOPAIR(" I" ,unscalePID_i(Ki[0]));
	SERIAL_ECHOPAIR(" D" ,unscalePID_d(Kd[0]));
	SERIAL_ECHOPAIR(" - M301 RIGHT P",Kp[1]);
	SERIAL_ECHOPAIR(" I" ,unscalePID_i(Ki[1]));
	SERIAL_ECHOPAIR(" D" ,unscalePID_d(Kd[1]));
	SERIAL_ECHOLN("");
	#endif

	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Offsets (mm):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M530 X ",extruder_offset[X_AXIS][1] );
	SERIAL_ECHOPAIR(" Y " ,extruder_offset[Y_AXIS][1]  );
	SERIAL_ECHOPAIR(" Z " ,extruder_offset[Z_AXIS][1]  );
	SERIAL_ECHOPAIR(" P(Z probe)" ,zprobe_zoffset  );
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Bed Offsets (mm):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M531 L(Bed left screw) ",bed_offset_left_screw);
	SERIAL_ECHOPAIR(" R(Bed right screw) " ,bed_offset_right_screw);
	SERIAL_ECHOPAIR(" V(Version) " ,(unsigned long)bed_offset_version);
	SERIAL_ECHOLN("");
	
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Temp (C):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  L_INSERT: ",(unsigned long)load_temp_l);
	SERIAL_ECHOPAIR(" C, L_REMOVE: " ,(unsigned long)unload_temp_l);
	SERIAL_ECHOPAIR(" C, L_BED: " ,(unsigned long)bed_temp_l);
	SERIAL_ECHOPAIR(" C, L_PRINT: " ,(unsigned long)print_temp_l);
	SERIAL_ECHOLN(" C");
	
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Temp (C):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  R_INSERT: ",(unsigned long)load_temp_r);
	SERIAL_ECHOPAIR(" C, R_REMOVE: " ,(unsigned long)unload_temp_r);
	SERIAL_ECHOPAIR(" C, R_BED: " ,(unsigned long)bed_temp_r);
	SERIAL_ECHOPAIR(" C, R_PRINT: " ,(unsigned long)print_temp_r);
	SERIAL_ECHOLN(" C");
	
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("STATS:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  prints: ",(unsigned long)log_prints);
	SERIAL_ECHOPAIR(", printing time: " ,(unsigned long)log_hours_print);
	SERIAL_ECHOPAIR("h, prints finished: " ,(unsigned long)log_prints_finished);
	SERIAL_ECHOPAIR(", max temp L: " ,(unsigned long)log_max_temp_l);
	SERIAL_ECHOPAIR(" C, max temp R: " ,(unsigned long)log_max_temp_r);
	SERIAL_ECHOPAIR(" C, max temp B: " ,(unsigned long)log_max_bed);
	SERIAL_ECHOPAIR(" C,\n X0 print distance: " ,(float)log_X0_mmdone/1000000);
	SERIAL_ECHOPAIR(" km, X1 print distance: " ,(float)log_X1_mmdone/1000000);
	SERIAL_ECHOPAIR(" km, Y print distance: " ,(float)log_Y_mmdone/1000000);
	SERIAL_ECHOPAIR(" km, E0 print distance: " ,(float)log_E0_mmdone/1000);
	SERIAL_ECHOPAIR(" m, E1 print distance: " ,(float)log_E1_mmdone/1000);
	SERIAL_ECHOLN(" m");
	
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("UI Information Serial Number:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  First ID number: ",(unsigned long)UI_SerialID0);
	SERIAL_ECHOPAIR(", Second ID number: " ,(unsigned long)UI_SerialID1);
	SERIAL_ECHOPAIR(", Third ID number " ,(unsigned long)UI_SerialID2);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Last print duration");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  Last print time-> Hours :",(unsigned long)log_hours_lastprint);
	SERIAL_ECHOPAIR(" Minutes :" ,(unsigned long)log_minutes_lastprint);
	SERIAL_ECHOLN("");
	
}
#endif
#ifndef DISABLE_M503
void Config_PrintSAVESettings()
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("SAVE PRINT LOG:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR(", saved_x_position: " ,(float)saved_x_position);
	SERIAL_ECHOPAIR(", saved_y_position: " ,(float)saved_y_position);
	SERIAL_ECHOPAIR(", saved_z_position: " ,(float)saved_z_position);
	SERIAL_ECHOPAIR(", saved_tool_active: " ,(unsigned long)saved_tool_active);
	SERIAL_ECHOPAIR(", saved_e_position: " ,(float)saved_e_position);
	SERIAL_ECHOPAIR(", saved_fileposition: " ,(unsigned long)saved_fileposition);
	SERIAL_ECHOPAIR(", saved_temp1: " ,(float)saved_temp1);
	SERIAL_ECHOPAIR(", saved_temp0: " ,(float)saved_temp0);
	SERIAL_ECHOPAIR(", saved_tempbed: " ,(float)saved_tempbed);
	SERIAL_ECHOPAIR(", saved_fanlayer: " ,(float)saved_fanlayer);
	SERIAL_ECHOPAIR(", saved_feedrate: " ,(float)saved_feedmulti);
	SERIAL_ECHOPAIR(", saved_Flag_fanlayer_mirror: " ,(float)saved_Flag_fanSpeed_mirror);
	SERIAL_ECHOLN("");
	
	
	
	
}
#endif

#ifdef EEPROM_SETTINGS
void Config_RetrieveSettings()
{
	int i=EEPROM_OFFSET;
	char stored_ver[4];
	char ver[4]=EEPROM_VERSION;
	EEPROM_READ_VAR(i,stored_ver); //read stored version
	
	//  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
	if (strncmp(ver,stored_ver,3) == 0)
	{
		// version number match
		EEPROM_READ_VAR(i,axis_steps_per_unit);
		EEPROM_READ_VAR(i,max_feedrate);
		EEPROM_READ_VAR(i,max_acceleration_units_per_sq_second);
		
		// steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		reset_acceleration_rates();
		
		EEPROM_READ_VAR(i,acceleration);
		EEPROM_READ_VAR(i,retract_acceleration);
		EEPROM_READ_VAR(i,minimumfeedrate);
		EEPROM_READ_VAR(i,mintravelfeedrate);
		EEPROM_READ_VAR(i,minsegmenttime);
		EEPROM_READ_VAR(i,max_xy_jerk);
		EEPROM_READ_VAR(i,max_z_jerk);
		EEPROM_READ_VAR(i,max_e_jerk);
		EEPROM_READ_VAR(i,add_homing);
		#ifdef DELTA
		EEPROM_READ_VAR(i,endstop_adj);
		EEPROM_READ_VAR(i,delta_radius);
		EEPROM_READ_VAR(i,delta_diagonal_rod);
		EEPROM_READ_VAR(i,delta_segments_per_second);
		#endif
		#ifndef ULTIPANEL
		int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
		int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
		#endif
		EEPROM_READ_VAR(i,plaPreheatHotendTemp);
		EEPROM_READ_VAR(i,plaPreheatHPBTemp);
		EEPROM_READ_VAR(i,plaPreheatFanSpeed);
		EEPROM_READ_VAR(i,absPreheatHotendTemp);
		EEPROM_READ_VAR(i,absPreheatHPBTemp);
		EEPROM_READ_VAR(i,absPreheatFanSpeed);
		
		EEPROM_READ_VAR(i,zprobe_zoffset);
		
		//Extruder Offset
		EEPROM_READ_VAR(i,extruder_offset[X_AXIS][RIGHT_EXTRUDER]);
		EEPROM_READ_VAR(i,extruder_offset[Y_AXIS][RIGHT_EXTRUDER]);
		EEPROM_READ_VAR(i,extruder_offset[Z_AXIS][RIGHT_EXTRUDER]);
		
		
		
		//Quick Start Guide
		//EEPROM_READ_VAR(i,quick_guide);
		EEPROM_READ_VAR(i,print_temp_l);
		EEPROM_READ_VAR(i,load_temp_l);
		EEPROM_READ_VAR(i,unload_temp_l);
		EEPROM_READ_VAR(i,bed_temp_l);
		EEPROM_READ_VAR(i,old_print_temp_l);
		EEPROM_READ_VAR(i,old_load_temp_l);
		EEPROM_READ_VAR(i,old_unload_temp_l);
		EEPROM_READ_VAR(i,old_bed_temp_l);
		
		EEPROM_READ_VAR(i,print_temp_r);
		EEPROM_READ_VAR(i,load_temp_r);
		EEPROM_READ_VAR(i,unload_temp_r);
		EEPROM_READ_VAR(i,bed_temp_r);
		EEPROM_READ_VAR(i,old_print_temp_r);
		EEPROM_READ_VAR(i,old_load_temp_r);
		EEPROM_READ_VAR(i,old_unload_temp_r);
		EEPROM_READ_VAR(i,old_bed_temp_r);
		
		//Language
		//		EEPROM_READ_VAR(i,language);
		
		#ifdef PIDTEMP
		// do not need to scale PID values as the values in EEPROM are already scaled
		EEPROM_READ_VAR(i,Kp[0]);
		EEPROM_READ_VAR(i,Ki[0]);
		EEPROM_READ_VAR(i,Kd[0]);
		EEPROM_READ_VAR(i,Kp[1]);
		EEPROM_READ_VAR(i,Ki[1]);
		EEPROM_READ_VAR(i,Kd[1]);
		#endif
		
		EEPROM_READ_VAR(i,log_prints);
		EEPROM_READ_VAR(i,log_prints_finished);
		EEPROM_READ_VAR(i,log_max_temp_l);
		EEPROM_READ_VAR(i,log_max_temp_r);
		EEPROM_READ_VAR(i,log_hours_print);
		EEPROM_READ_VAR(i,log_max_bed);
		
		#ifndef DOGLCD
		int lcd_contrast;
		#endif
		EEPROM_READ_VAR(i,lcd_contrast);
		#ifdef SCARA
		EEPROM_READ_VAR(i,axis_scaling);
		#endif
		EEPROM_READ_VAR(i,UI_SerialID0);
		EEPROM_READ_VAR(i,UI_SerialID1);
		EEPROM_READ_VAR(i,UI_SerialID2);
		EEPROM_READ_VAR(i,log_minutes_lastprint);
		EEPROM_READ_VAR(i,log_hours_lastprint);
		EEPROM_READ_VAR(i,log_X0_mmdone);
		EEPROM_READ_VAR(i,log_X1_mmdone);
		EEPROM_READ_VAR(i,log_Y_mmdone);
		EEPROM_READ_VAR(i,log_E0_mmdone);
		EEPROM_READ_VAR(i,log_E1_mmdone);
		EEPROM_READ_VAR(i,FLAG_First_Start_Wizard);
		EEPROM_READ_VAR(i,saved_print_flag);
		EEPROM_READ_VAR(i,saved_x_position);
		EEPROM_READ_VAR(i,saved_y_position);
		EEPROM_READ_VAR(i,saved_z_position);
		EEPROM_READ_VAR(i,saved_tool_active);
		EEPROM_READ_VAR(i,saved_e_position);
		EEPROM_READ_VAR(i,saved_fileposition);
		EEPROM_READ_VAR(i,saved_temp1);
		EEPROM_READ_VAR(i,saved_temp0);
		EEPROM_READ_VAR(i,saved_tempbed);
		EEPROM_READ_VAR(i,saved_fanlayer);
		EEPROM_READ_VAR(i,saved_feedmulti);
		EEPROM_READ_VAR(i,saved_workDir_vector_lenght);
		EEPROM_READ_VAR(i,saved_workDir_vector[0]);
		EEPROM_READ_VAR(i,saved_workDir_vector[1]);
		EEPROM_READ_VAR(i,saved_workDir_vector[2]);
		EEPROM_READ_VAR(i,saved_workDir_vector[3]);
		EEPROM_READ_VAR(i,saved_workDir_vector[4]);
		EEPROM_READ_VAR(i,saved_workDir_vector[5]);
		EEPROM_READ_VAR(i,saved_workDir_vector[6]);
		EEPROM_READ_VAR(i,saved_workDir_vector[7]);
		EEPROM_READ_VAR(i,saved_workDir_vector[8]);
		EEPROM_READ_VAR(i,saved_workDir_vector[9]);
		EEPROM_READ_VAR(i,acceleration_old);
		EEPROM_READ_VAR(i,version_number);
		EEPROM_READ_VAR(i,bed_offset_left_screw);
		EEPROM_READ_VAR(i,bed_offset_right_screw);
		EEPROM_READ_VAR(i,manual_fine_calib_offset[0]);
		EEPROM_READ_VAR(i,manual_fine_calib_offset[1]);
		EEPROM_READ_VAR(i,manual_fine_calib_offset[2]);
		EEPROM_READ_VAR(i,manual_fine_calib_offset[3]);
		EEPROM_READ_VAR(i,saved_dual_x_carriage_mode);
		EEPROM_READ_VAR(i,saved_duplicate_extruder_x_offset);
		EEPROM_READ_VAR(i,saved_Flag_fanSpeed_mirror);
		EEPROM_READ_VAR(i,bed_offset_version);
		EEPROM_READ_VAR(i,flag_utilities_calibration_zcomensationmode_gauges);
		// Call updatePID (similar to when we have processed M301)
		updatePID();
		SERIAL_ECHO_START;
		if (UI_SerialID0 <= 0 || UI_SerialID0 >= 123 || UI_SerialID2 >= 9999 || UI_SerialID1 >= 999999 || UI_SerialID2 <= 0 || UI_SerialID1 <= 0){
			UI_SerialID0 = 0;
			UI_SerialID1 = 0;
			UI_SerialID2 = 0;
		}
		if (abs(bed_offset_left_screw)>= 1.0 ||abs(bed_offset_right_screw) >= 1.0){
			bed_offset_left_screw = 0.0;
			bed_offset_right_screw = 0.0;
			}else if(isnan(bed_offset_left_screw) || isnan(bed_offset_right_screw)  ){
			bed_offset_left_screw = 0.0;
			bed_offset_right_screw = 0.0;
		}
		if(bed_offset_version > 1000 || isnan(bed_offset_version)){
			bed_offset_version = 0;
		}
		if(isnan(manual_fine_calib_offset[0]) || isnan(manual_fine_calib_offset[1])  || isnan(manual_fine_calib_offset [2]) || isnan(manual_fine_calib_offset[3]))
		{
			manual_fine_calib_offset[0] = 0.0;
			manual_fine_calib_offset[1] = 0.0;
			manual_fine_calib_offset[2] = 0.0;
			manual_fine_calib_offset[3] = 0.0;
			Config_StoreSettings();
		}
		SERIAL_ECHOLNPGM("Stored settings retrieved");
	}
	else
	{
		Config_ResetDefault();
		Config_Reset_Calib();
		Config_StoreSettings();
	}
	#ifdef EEPROM_CHITCHAT
	Config_PrintSettings();
	#endif
}
#endif

void Config_ResetDefault()
{
	float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
	float tmp2[]=DEFAULT_MAX_FEEDRATE;
	long tmp3[]=DEFAULT_MAX_ACCELERATION;
	for (short i=0;i<4;i++)
	{
		axis_steps_per_unit[i]=tmp1[i];
		max_feedrate[i]=tmp2[i];
		max_acceleration_units_per_sq_second[i]=tmp3[i];
		#ifdef SCARA
		axis_scaling[i]=1;
		#endif
	}
	
	// steps per sq second need to be updated to agree with the units per sq second
	reset_acceleration_rates();
	
	acceleration=DEFAULT_ACCELERATION;
	retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
	minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
	minsegmenttime=DEFAULT_MINSEGMENTTIME;
	mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
	max_xy_jerk=DEFAULT_XYJERK;
	max_z_jerk=DEFAULT_ZJERK;
	max_e_jerk=DEFAULT_EJERK;
	add_homing[0] = add_homing[1] = add_homing[2] = 0;
	//quick_guide = DEFAULT_QUICK_GUIDE;
	print_temp_l = DEFAULT_PRINT_TEMP;
	load_temp_l=DEFAULT_INSERT_TEMP;
	unload_temp_l=DEFAULT_REMOVE_TEMP;
	bed_temp_l = DEFAULT_BED_TEMP;
	old_print_temp_l=DEFAULT_OLD_PRINT_TEMP;
	old_load_temp_l=DEFAULT_OLD_INSERT_TEMP;
	old_unload_temp_l=DEFAULT_OLD_REMOVE_TEMP;
	old_bed_temp_l = DEFAULT_OLD_BED_TEMP;
	print_temp_r = DEFAULT_PRINT_TEMP;
	load_temp_r=DEFAULT_INSERT_TEMP;
	unload_temp_r=DEFAULT_REMOVE_TEMP;
	bed_temp_r = DEFAULT_BED_TEMP;
	old_print_temp_r=DEFAULT_OLD_PRINT_TEMP;
	old_load_temp_r=DEFAULT_OLD_INSERT_TEMP;
	old_unload_temp_r=DEFAULT_OLD_REMOVE_TEMP;
	old_bed_temp_r = DEFAULT_OLD_BED_TEMP;
	
	#ifdef DELTA
	endstop_adj[0] = endstop_adj[1] = endstop_adj[2] = 0;
	delta_radius= DELTA_RADIUS;
	delta_diagonal_rod= DELTA_DIAGONAL_ROD;
	delta_segments_per_second= DELTA_SEGMENTS_PER_SECOND;
	recalc_delta_settings(delta_radius, delta_diagonal_rod);
	#endif
	#ifdef ULTIPANEL
	plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
	plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
	plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
	absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
	absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
	absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
	#endif
	
	if (UI_SerialID0 <= 0 || UI_SerialID0 >= 123 || UI_SerialID2 >= 9999 || UI_SerialID1 >= 999999 || UI_SerialID2 <= 0 || UI_SerialID1 <= 0){
		UI_SerialID0 = 0;
		UI_SerialID1 = 0;
		UI_SerialID2 = 0;
	}
	log_minutes_lastprint = 0;
	log_hours_lastprint = 0;
	FLAG_First_Start_Wizard = 1888;
	
	/*
	//Extruder Offset
	//extruder_offset = {EXTRUDER_OFFSET_X,EXTRUDER_OFFSET_Y,EXTRUDER_OFFSET_Z};
	extruder_offset[X_AXIS][RIGHT_EXTRUDER] = X2_MAX_POS;
	extruder_offset[Y_AXIS][RIGHT_EXTRUDER] = -0.15;
	extruder_offset[Z_AXIS][RIGHT_EXTRUDER] = -0.1;

	#ifdef DOGLCD
	lcd_contrast = DEFAULT_LCD_CONTRAST;
	#endif
	#ifdef PIDTEMP
	Kp = DEFAULT_Kp;
	Ki = scalePID_i(DEFAULT_Ki);
	Kd = scalePID_d(DEFAULT_Kd);
	
	// call updatePID (similar to when we have processed M301)
	updatePID();
	
	#ifdef PID_ADD_EXTRUSION_RATE
	Kc = DEFAULT_Kc;
	#endif//PID_ADD_EXTRUSION_RATE
	#endif//PIDTEMP*/

	#ifdef DOGLCD
	lcd_contrast = DEFAULT_LCD_CONTRAST;
	#endif
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");

}

void Config_Reset_Calib(){
	//Extruder Offset
	//extruder_offset = {EXTRUDER_OFFSET_X,EXTRUDER_OFFSET_Y,EXTRUDER_OFFSET_Z};
	manual_fine_calib_offset[0] = 0.0;
	manual_fine_calib_offset[1] = 0.0;
	manual_fine_calib_offset[2] = 0.0;
	manual_fine_calib_offset[3] = 0.0;
	extruder_offset[X_AXIS][RIGHT_EXTRUDER] = X2_MAX_POS;
	extruder_offset[Y_AXIS][RIGHT_EXTRUDER] = 0.25;
	extruder_offset[Z_AXIS][RIGHT_EXTRUDER] = 0;
	#ifdef ENABLE_AUTO_BED_LEVELING
	zprobe_zoffset = -Z_PROBE_OFFSET_FROM_EXTRUDER;
	#endif
	#ifdef Z_SIGMA_HOME
	zprobe_zoffset = -Z_SIGMA_PROBE_OFFSET_FROM_EXTRUDER; //Overrides zprove_zoffset
	#endif
	
	#ifdef PIDTEMP
	Kp[0] = DEFAULT_Kp;
	Ki[0] = scalePID_i(DEFAULT_Ki);
	Kd[0] = scalePID_d(DEFAULT_Kd);
	Kp[1] = DEFAULT_Kp;
	Ki[1] = scalePID_i(DEFAULT_Ki);
	Kd[1] = scalePID_d(DEFAULT_Kd);
	
	// call updatePID (similar to when we have processed M301)
	//updatePID();
	
	#ifdef PID_ADD_EXTRUSION_RATE
	Kc[0] = DEFAULT_Kc;
	Kc[1] = DEFAULT_Kc;
	#endif//PID_ADD_EXTRUSION_RATE
	#endif//PIDTEMP
	
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Hardcoded Calib and PID Default Settings Loaded");
}

void Config_Reset_Statistics(int data){
	
	if (data == 1234){
		log_hours_print = 0;
		log_max_bed = 0;
		log_max_temp_l = 0;
		log_max_temp_r = 0;
		log_prints = 0;
		log_prints_finished = 0;
		log_X0_mmdone = 0;
		log_X1_mmdone = 0;
		log_Y_mmdone = 0;
		log_E0_mmdone = 0;
		log_E1_mmdone = 0;
		FLAG_First_Start_Wizard = 1888;
		SERIAL_PROTOCOLLNPGM("STATISTICS RESET");
	}
}
void Config_Set_UISerialNumber(int input0, long input1, int input2){
	if(input0 && input1 && input2 && !(input0 <= 0 || input0 >= 123 || input2 >= 9999 || input1 >= 999999 || input2 <= 0 || input1 <= 0)){
		UI_SerialID0 = input0;
		UI_SerialID1 = input1;
		UI_SerialID2 = input2;
		Serial.println(UI_SerialID0);
		Serial.println(UI_SerialID1);
		Serial.println(UI_SerialID2);
	}
}
void Change_ConfigTemp_LeftHotend(int i_temp_l, int r_temp_l, int p_temp_l, int b_temp_l){
	
	if (i_temp_l > HEATER_0_MAXTEMP ||  i_temp_l < EXTRUDE_MINTEMP){
		SERIAL_PROTOCOLLNPGM("Values out of range");
	}
	else if (r_temp_l > HEATER_0_MAXTEMP ||  r_temp_l < EXTRUDE_MINTEMP){
		SERIAL_PROTOCOLLNPGM("Values out of range");
	}
	else if (p_temp_l > HEATER_0_MAXTEMP ||  p_temp_l < EXTRUDE_MINTEMP){
		SERIAL_PROTOCOLLNPGM("Values out of range");
	}
	else if (b_temp_l > BED_MAXTEMP ||  b_temp_l < BED_MINTEMP){
		SERIAL_PROTOCOLLNPGM("Values out of range");
	}
	else{
		
		
		load_temp_l =i_temp_l;
		unload_temp_l = r_temp_l;
		print_temp_l = p_temp_l;
		bed_temp_l = b_temp_l;
		
		SERIAL_PROTOCOLLNPGM("SUCCESS");
		
	}
	
	
	
}
void Change_ConfigTemp_RightHotend(int i_temp_r, int r_temp_r, int p_temp_r, int b_temp_r){
	
	if (i_temp_r > HEATER_0_MAXTEMP ||  i_temp_r < EXTRUDE_MINTEMP){
		SERIAL_PROTOCOLLNPGM("Values out of range");
	}
	else if (r_temp_r > HEATER_0_MAXTEMP ||  r_temp_r < EXTRUDE_MINTEMP){
		SERIAL_PROTOCOLLNPGM("Values out of range");
	}
	else if (p_temp_r > HEATER_0_MAXTEMP ||  p_temp_r < EXTRUDE_MINTEMP){
		SERIAL_PROTOCOLLNPGM("Values out of range");
	}
	else if (b_temp_r > BED_MAXTEMP ||  b_temp_r < BED_MINTEMP){
		SERIAL_PROTOCOLLNPGM("Values out of range");
	}
	else{
		
		
		load_temp_r =i_temp_r;
		unload_temp_r = r_temp_r;
		print_temp_r = p_temp_r;
		bed_temp_r = b_temp_r;
		SERIAL_PROTOCOLLNPGM("SUCCESS");
		
		
	}
	
	
}
void Change_ConfigCalibration(float Xcalib, float Ycalib, float Zcalib, float Zprobecalib){
	
	
	extruder_offset[X_AXIS][1]= Xcalib;
	extruder_offset[Y_AXIS][1]= Ycalib;
	extruder_offset[Z_AXIS][1]= Zcalib;
	zprobe_zoffset= Zprobecalib;
	
}
void Change_ConfigBed_offset(float bed_left, float bed_right, unsigned int version){
	
	if (abs(bed_left >= 1.0) || abs(bed_right >= 1.0)){
		bed_offset_left_screw = bed_offset_left_screw;
		bed_offset_right_screw = bed_offset_right_screw;
		bed_offset_version = bed_offset_version;
		}else{
		bed_offset_left_screw = bed_left;
		bed_offset_right_screw = bed_right;
		bed_offset_version = version;
	}
	
	
}