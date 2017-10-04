// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// License: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>


#include "fastio.h"
#include "Configuration.h"
#include "cardreader.h"
#include "pins.h"

#include "genieArduino.h"

//#ifndef AT90USB
//#define  HardwareSerial_h // trick to disable the standard HWserial
//#endif

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
//Arduino < 1.0.0 does not define this, so we need to do it ourselves
# define analogInputToDigitalPin(p) ((p) + A0)
#endif

//#ifdef AT90USB
#include "HardwareSerial.h"
//#endif

//#include "MarlinSerial.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"
//
//#ifdef AT90USB
//#ifdef BTENABLED
//#define MYSERIAL bt
//#else
#define MYSERIAL Serial
//#define MYSERIAL_SCREEN Serial2
//#define MYSERIAL_SCREEN Serial3
//#endif // BTENABLED
//#else
//#define MYSERIAL MSerial
//#endif
#define screen_printing_pause_form0		'A'
#define screen_printing_pause_form1		'B'
#define screen_printing_pause_form2		'C'

#define SERIAL_PROTOCOL(x) (MYSERIAL.print(x))
#define SERIAL_PROTOCOL_F(x,y) (MYSERIAL.print(x,y))
#define SERIAL_PROTOCOLPGM(x) (serialprintPGM(PSTR(x)))
#define SERIAL_PROTOCOLLN(x) (MYSERIAL.print(x),MYSERIAL.write('\n'))
#define SERIAL_PROTOCOLLNPGM(x) (serialprintPGM(PSTR(x)),MYSERIAL.write('\n'))


const char errormagic[] PROGMEM ="Error:";
const char echomagic[] PROGMEM ="echo:";
#define SERIAL_ERROR_START (serialprintPGM(errormagic))
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START (serialprintPGM(echomagic))
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, unsigned long v);


//Things to write to serial from Program memory. Saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str)
{
	char ch=pgm_read_byte(str);
	while(ch)
	{
		MYSERIAL.write(ch);
		ch=pgm_read_byte(++str);
	}
}


void get_command();
void process_commands();

void manage_inactivity();

#if defined(DUAL_X_CARRIAGE) && defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1 \
&& defined(X2_ENABLE_PIN) && X2_ENABLE_PIN > -1
#define  enable_x() do { WRITE(X_ENABLE_PIN, X_ENABLE_ON); WRITE(X2_ENABLE_PIN, X_ENABLE_ON);}  while (0)
#define disable_x() do { WRITE(X_ENABLE_PIN,!X_ENABLE_ON); WRITE(X2_ENABLE_PIN,!X_ENABLE_ON); axis_known_position[X_AXIS] = false; } while (0)
#elif defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
#define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
#define disable_x() { WRITE(X_ENABLE_PIN,!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }
#else
#define enable_x() ;
#define disable_x() ;
#endif

#if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
#ifdef Y_DUAL_STEPPER_DRIVERS
#define  enable_y() { WRITE(Y_ENABLE_PIN, Y_ENABLE_ON); WRITE(Y2_ENABLE_PIN,  Y_ENABLE_ON); }
#define disable_y() { WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON); WRITE(Y2_ENABLE_PIN, !Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
#else
#define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
#define disable_y() { WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
#endif
#else
#define enable_y() ;
#define disable_y() ;
#endif

#if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
#ifdef Z_DUAL_STEPPER_DRIVERS
#define  enable_z() { WRITE(Z_ENABLE_PIN, Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); }
#define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
#else
#define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
#define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
#endif
#else
#define enable_z() ;
#define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
#define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
#define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
#define enable_e0()  /* nothing */
#define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
#define enable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
#define disable_e1() WRITE(E1_ENABLE_PIN,!E_ENABLE_ON)
#else
#define enable_e1()  /* nothing */
#define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
#define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
#define disable_e2() WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
#else
#define enable_e2()  /* nothing */
#define disable_e2() /* nothing */
#endif


enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};


void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
#ifdef DELTA
void calculate_delta(float cartesian[3]);
extern float delta[3];
#endif
#ifdef SCARA
void calculate_delta(float cartesian[3]);
void calculate_SCARA_forward_Transform(float f_scara[3]);
#endif
void prepare_move();
void kill();
void Stop();
bool IsStopped();

void enquecommand(const char *cmd); //put an ASCII command at the end of the current buffer.
void enquecommand_P(const char *cmd); //put an ASCII command at the end of the current buffer, read from flash
void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

void refresh_cmd_timeout(void);

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
#define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START
extern bool home_made;
extern bool home_made_Z;
extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent) for all extruders
extern int extruder_multiply[EXTRUDERS]; // sets extrude multiply factor (in percent) for each extruder individually
extern float volumetric_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern float current_position[NUM_AXIS] ;
extern float add_homing[3];
#ifdef DELTA
extern float endstop_adj[3];
extern float delta_radius;
extern float delta_diagonal_rod;
extern float delta_segments_per_second;
void recalc_delta_settings(float radius, float diagonal_rod);
#endif
#ifdef SCARA
extern float axis_scaling[3];  // Build size scaling
#endif
extern float min_pos[3];
extern float max_pos[3];
extern bool axis_known_position[3];
extern float zprobe_zoffset;
extern float bed_offset_left_screw;
extern float bed_offset_right_screw;
extern unsigned int bed_offset_version;
extern int flag_utilities_calibration_zcomensationmode_gauges;
extern int fanSpeed;
extern int Flag_fanSpeed_mirror;
extern int sd_printing_temp_setting_offset_bed;
extern int sd_printing_temp_setting_offset_hotent0;
extern int sd_printing_temp_setting_offset_hotent1;
#ifdef BARICUDA
extern int ValvePressure;
extern int EtoPPressure;
#endif

#ifdef FAN_SOFT_PWM
extern unsigned char fanSpeedSoftPwm;
#endif

#ifdef FILAMENT_SENSOR
extern float filament_width_nominal;  //holds the theoretical filament diameter ie., 3.00 or 1.75
extern bool filament_sensor;  //indicates that filament sensor readings should control extrusion
extern float filament_width_meas; //holds the filament diameter as accurately measured
extern signed char measurement_delay[];  //ring buffer to delay measurement
extern int delay_index1, delay_index2;  //index into ring buffer
extern float delay_dist; //delay distance counter
extern int meas_delay_cm; //delay distance
#endif

#ifdef FWRETRACT
extern bool autoretract_enabled;
extern bool retracted[EXTRUDERS];
extern float retract_length, retract_length_swap, retract_feedrate, retract_zlift;
extern float retract_recover_length, retract_recover_length_swap, retract_recover_feedrate;
#endif

extern unsigned long starttime;
extern unsigned long stoptime;

// Handling multiple extruders pins
extern uint8_t active_extruder;

#ifdef DIGIPOT_I2C
extern void digipot_i2c_set_current( int channel, float current );
extern void digipot_i2c_init();
#endif

//Rapduch
extern Genie genie;
void touchscreen_update();
extern uint16_t filepointer;
extern int8_t saved_active_extruder;
extern String screen_status;
extern bool waiting_temps;
extern int UI_SerialID0;
extern long UI_SerialID1;
extern int UI_SerialID2;
extern bool surfing_utilities;
extern bool screen_sdcard;
extern bool surfing_temps;
extern bool is_on_printing_screen;
extern uint8_t which_extruder;
extern char filament_mode;
extern bool is_changing_filament;
extern bool is_purging_filament;
extern int getBuflen();
extern void HeaterCooldownInactivity(bool switchOnOff);
extern void homeFromMain();
extern void changeTool(int ntool);
extern void changeToolSigma(int tool);
extern int vuitens1;
extern int vuitens2;
extern int vuitens3;
extern int sentit1;
extern int sentit2;
extern int sentit3;
extern float extruder_offset[3][EXTRUDERS];
extern void sleep_RELAY();
extern void wake_RELAY();
extern void home_axis_from_code(bool x_c, bool y_c, bool z_c);
extern float saved_position[NUM_AXIS];
extern bool flag_continue_calib;
extern int purge_extruder_selected;
extern float manual_fine_calib_offset[4];
//extern bool quick_guide;
//extern int language;
//extern const char idiom[50][50];
//extern void updateLanguage();
extern float x_home_pos(int extruder);
extern int bed_calibration_times;
extern char printer_state;
/////// Processing Gifs	/////////
extern char gif_processing_state;
extern uint8_t processing_z_set;
extern bool printing_error_temps;
extern int FLAG_First_Start_Wizard;
extern bool FLAG_thermal_runaway;
extern bool FLAG_thermal_runaway_screen;
extern int ID_thermal_runaway;
extern bool Step_First_Start_Wizard;
/////// end Processing Gifs	/////////

extern bool back_home;
extern bool cancel_heatup;
//extern int quick_guide_step;
//extern unsigned long previous_millis_cmd;
extern void z_test_print_code(int tool, float x_offset);
extern void bed_test_print_code(float x_offset, float y_offset, int zline);
extern bool heatting;
extern char namefilegcode[24];

////// Temperatures of current material for two extruders //////

extern int load_temp_l;
extern int unload_temp_l;
extern int print_temp_l;
extern int bed_temp_l;
extern int load_temp_r;
extern int unload_temp_r;
extern int print_temp_r;
extern int bed_temp_r;
extern int old_load_temp_l;
extern int old_unload_temp_l;
extern int old_print_temp_l;
extern int old_bed_temp_l;
extern int old_load_temp_r;
extern int old_unload_temp_r;
extern int old_print_temp_r;
extern int old_bed_temp_r;

////// end Temperatures of current material for two extruders //////
extern int version_number;
/////// Print Recovery	/////////

extern float saved_x_position;
extern float saved_y_position;
extern float saved_z_position;
extern int saved_tool_active;
extern float saved_e_position;
extern long saved_fileposition;
extern int saved_temp1;
extern int saved_temp0;
extern int saved_tempbed;
extern int saved_feedmulti;
extern int saved_fanlayer;
extern int saved_print_flag;
extern bool saved_print_smartpurge_flag;
extern int workDir_vector[MAX_DIR_DEPTH];
extern uint8_t workDir_vector_lenght;
extern int saved_workDir_vector[MAX_DIR_DEPTH];
extern int saved_Flag_fanSpeed_mirror;
extern uint8_t saved_workDir_vector_lenght;
extern int saved_dual_x_carriage_mode;
extern float saved_duplicate_extruder_x_offset;

/////// end Print Recovery	/////////

extern void PID_autotune_Save(float temp, int extruder, int ncycles, float K_p);
extern void Config_Reset_Calib();
extern void Config_StoreSettings();
extern bool doblocking;
extern int preheat_E0_value;
extern int preheat_E1_value;
extern int preheat_B_value;

//////// PRINT STATS ////////

extern int log_prints;
extern int log_hours_print;
extern long log_min_print;
extern int log_minutes_lastprint;
extern int log_hours_lastprint;
extern int log_prints_finished;
extern int log_max_temp_l;
extern int log_max_temp_r;
extern int log_max_bed;
extern int long long log_X0_mmdone;
extern int long long log_X1_mmdone;
extern int long long log_Y_mmdone;
extern int long long log_E0_mmdone;
extern int long long log_E1_mmdone;

//////// end PRINT STATS ////////

extern char screen_printing_pause_form;
extern bool screen_change_nozz1up;
extern bool screen_change_nozz2up;
extern bool screen_change_bedup;
extern bool screen_change_speedup;
extern bool screen_change_nozz1down;
extern bool screen_change_nozz2down;
extern bool screen_change_beddown;
extern bool screen_change_speeddown;
extern void set_dual_x_carriage_mode(int mode);
extern int get_dual_x_carriage_mode();
extern void set_duplicate_extruder_x_offset(int offset);
extern float get_duplicate_extruder_x_offset();
extern void checkMaxTemps();
extern void gcode_T0_T1_auto(int code);
extern void Config_Reset_Statistics(int data);
extern void Config_Set_UISerialNumber(int input0, long input1, int input2);
extern void Change_ConfigTemp_LeftHotend(int i_temp_l, int remove_temp_l, int p_temp_r, int b_temp_l);
extern void Change_ConfigTemp_RightHotend(int i_temp_r, int r_temp_r, int p_temp_r, int b_temp_r);
extern void Change_ConfigCalibration(float Xcalib, float Ycalib, float Zcalib, float Zprobecalib);
extern void Change_ConfigBed_offset(float bed_left, float bed_right, unsigned int version);
extern void thermal_error_screen_on();
#endif
