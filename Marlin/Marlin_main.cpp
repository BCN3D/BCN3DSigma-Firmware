/* -*- c++ -*- */

/*
Reprap firmware based on Sprinter and grbl.
Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
This firmware is a mashup between Sprinter and grbl.
(https://github.com/kliment/Sprinter)
(https://github.com/simen/grbl/tree)

It has preliminary support for Matthew Roberts advance algorithm
http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
*/

#include "Marlin.h"

#ifdef ENABLE_AUTO_BED_LEVELING
#include "vector_3.h"
#ifdef AUTO_BED_LEVELING_GRID
#include "qr_solve.h"
#endif
#endif // ENABLE_AUTO_BED_LEVELING

#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"


//Rapduch
#include "genieArduino.h"
#include "Touch_Screen_Definitions.h"
#include "LCD_Handler.h"

//static Genie genie;
Genie genie;
//void myGenieEventHandler();
//-------------------------

#ifdef BLINKM
#include "BlinkM.h"
#include "Wire.h"
#endif

#if NUM_SERVOS > 0
#include "Servo.h"
#endif

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif

/*GCODES

 look here for descriptions of G-codes: http://linuxcnc.org/handbook/gcode/g-code.html
 http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

Implemented Codes
-------------------
 G0  -> G1
 G1  - Coordinated Movement X Y Z E
 G2  - CW ARC
 G3  - CCW ARC
 G4  - Dwell S<seconds> or P<milliseconds>
 G10 - retract filament according to settings of M207
 G11 - retract recover filament according to settings of M208
 G28 - Home all Axis
 G29 - Detailed Z-Probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
 G30 - Single Z Probe, probes bed at current XY location.
 G31 - Dock sled (Z_PROBE_SLED only)
 G32 - Undock sled (Z_PROBE_SLED only)

Rapduch
 G33 - Autolevel Bed
 G34 - Autocal Bed
 G40 - Wizard X
 G41 - Square

 G90 - Use Absolute Coordinates
 G91 - Use Relative Coordinates
 G92 - Set current position to coordinates given

 M Codes
 M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
 M1   - Same as M0
 M17  - Enable/Power all stepper motors
 M18  - Disable all stepper motors; same as M84
 M20  - List SD card
 M21  - Init SD card
 M22  - Release SD card
 M23  - Select SD file (M23 filename.g)
 M24  - Start/resume SD print
 M25  - Pause SD print
 M26  - Set SD position in bytes (M26 S12345)
 M27  - Report SD print status
 M28  - Start SD write (M28 filename.g)
 M29  - Stop SD write
 M30  - Delete file from SD (M30 filename.g)
 M31  - Output time since last M109 or SD card start to serial
 M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
        syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
        Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
        The '#' is necessary when calling from within sd files, as it stops buffer prereading
 M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
 M80  - Turn on Power Supply
 M81  - Turn off Power Supply
 M82  - Set E codes absolute (default)
 M83  - Set E codes relative while in Absolute Coordinates (G90) mode
 M84  - Disable steppers until next move,
        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
 M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 M92  - Set axis_steps_per_unit - same syntax as G92
 M104 - Set extruder target temp
 M105 - Read current temp
 M106 - Fan on
 M107 - Fan off
 M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
        IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
 M112 - Emergency stop
 M114 - Output current position to serial port
 M115 - Capabilities string
 M117 - display message
 M119 - Output Endstop status to serial port
 M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
 M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
 M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
 M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
 M140 - Set bed target temp
 M150 - Set BlinkM Color Output R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
 M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
 M200 D<millimeters>- set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
 M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
 M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
 M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
 M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
 M206 - set additional homing offset
 M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
 M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
 M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
 M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
 M220 S<factor in percent>- set speed factor override percentage
 M221 S<factor in percent>- set extrude factor override percentage
 M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
 M240 - Trigger a camera to take a photograph
 M250 - Set LCD contrast C<contrast value> (value 0..63)
 M280 - set servo position absolute. P: servo index, S: angle or microseconds
 M300 - Play beep sound S<frequency Hz> P<duration ms>
 M301 - Set PID parameters P I and D
 M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
 M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
 M304 - Set bed PID parameters P I and D
 M350 - Set microstepping mode.
 M351 - Toggle MS1 MS2 pins directly.
 M352 - Reset quick wizard guide
 M400 - Finish all moves
 M401 - Lower z-probe if present
 M402 - Raise z-probe if present
 M404 - N<dia in mm> Enter the nominal filament width (3mm, 1.75mm ) or will display nominal filament width without parameters
 M405 - Turn on Filament Sensor extrusion control.  Optional D<delay in cm> to set delay in centimeters between sensor and extruder
 M406 - Turn off Filament Sensor extrusion control
 M407 - Displays measured filament diameter
 M500 - stores parameters in EEPROM
 M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
 M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
 M503 - print the current settings (from memory not from EEPROM)
 M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
 M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
 M665 - set delta configurations
 M666 - set delta endstop adjustment
 M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
 M907 - Set digital trimpot motor current using axis codes.
 M908 - Control digital trimpot directly. 

 ************ SCARA Specific - This can change to suit future G-code regulations
 M360 - SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 M361 - SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 M362 - SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 M363 - SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 M364 - SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
 M365 - SCARA calibration: Scaling factor, X, Y, Z axis
************* SCARA End ***************

 M928 - Start SD logging (M928 filename.g) - ended by M29
 M999 - Restart after being stopped by error
*/

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;

#endif



//Rapduch
#ifdef SIGMA_TOUCH_SCREEN
bool surfing_utilities = false;
bool is_on_printing_screen = false;
uint16_t filepointer = 0;
String screen_status = "Printing...";
uint8_t which_extruder=0;
char filament_mode='O';
bool is_changing_filament=false;
String currentSDFileName;
int vuitens1=0;
int vuitens2=0;
int vuitens3=0;
int sentit1=0;
int sentit2=0;
int sentit3=0;
#endif

#ifndef SIGMA_TOUCH_SCREEN
bool firstime = true;
void SD_firstPrint();
#endif


bool quick_guide = false;
bool blocking_x = false;
bool blocking_y = false;
int quick_guide_step = 0;
bool z_print_test = false;

int insert_temp_l;
int remove_temp_l;
int print_temp_l;
int bed_temp_l;
int insert_temp_r;
int remove_temp_r;
int print_temp_r;
int bed_temp_r;

int old_insert_temp_l;
int old_remove_temp_l;
int old_print_temp_l;
int old_bed_temp_l;
int old_insert_temp_r;
int old_remove_temp_r;
int old_print_temp_r;
int old_bed_temp_r;

bool home_made = false;
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
int extruder_multiply[EXTRUDERS] = {100
	#if EXTRUDERS > 1
	, 100
	#if EXTRUDERS > 2
	, 100
	#endif
	#endif
};
float volumetric_multiplier[EXTRUDERS] = {1.0
	#if EXTRUDERS > 1
	, 1.0
	#if EXTRUDERS > 2
	, 1.0
	#endif
	#endif
};
float	current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float	saved_position[NUM_AXIS] = {0.0,0.0,0.0,0.0}; //Xavi -> array to save the current position to make a pause

float add_homing[3]={0,0,0};
#ifdef DELTA
float endstop_adj[3]={0,0,0};
#endif

float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
bool axis_known_position[3] = {false, false, false};
float zprobe_zoffset;

bool processing = false;
bool heatting = false;


int bed_calibration_times = 0;

// Extruder offset
#if EXTRUDERS > 1
#ifndef DUAL_X_CARRIAGE
#define NUM_EXTRUDER_OFFSETS 2 // only in XY plane
#else
#define NUM_EXTRUDER_OFFSETS 3 // supports offsets in XYZ plane
#endif
float extruder_offset[NUM_EXTRUDER_OFFSETS][EXTRUDERS] = {
	#if defined(EXTRUDER_OFFSET_X)
	EXTRUDER_OFFSET_X
	#else
	0
	#endif
	,
	#if defined(EXTRUDER_OFFSET_Y)
	EXTRUDER_OFFSET_Y
	#else
	0
	#endif
	,
	//Rapduch
	#if defined(EXTRUDER_OFFSET_Z)
	EXTRUDER_OFFSET_Z
	#else
	0
	#endif
};
#endif
uint8_t active_extruder = 0;
int fanSpeed=0;
#ifdef SERVO_ENDSTOPS
int servo_endstops[] = SERVO_ENDSTOPS;
int servo_endstop_angles[] = SERVO_ENDSTOP_ANGLES;
#endif
#ifdef BARICUDA
int ValvePressure=0;
int EtoPPressure=0;
#endif

#ifdef FWRETRACT
bool autoretract_enabled=false;
bool retracted[EXTRUDERS]={false
	#if EXTRUDERS > 1
	, false
	#if EXTRUDERS > 2
	, false
	#endif
	#endif
};
bool retracted_swap[EXTRUDERS]={false
	#if EXTRUDERS > 1
	, false
	#if EXTRUDERS > 2
	, false
	#endif
	#endif
};

float retract_length = RETRACT_LENGTH;
float retract_length_swap = RETRACT_LENGTH_SWAP;
float retract_feedrate = RETRACT_FEEDRATE;
float retract_zlift = RETRACT_ZLIFT;
float retract_recover_length = RETRACT_RECOVER_LENGTH;
float retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
float retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE;
#endif

#ifdef ULTIPANEL
#ifdef PS_DEFAULT_OFF
bool powersupply = false;
#else
bool powersupply = true;
#endif
#endif

#ifdef DELTA
float delta[3] = {0.0, 0.0, 0.0};
#define SIN_60 0.8660254037844386
#define COS_60 0.5
// these are the default values, can be overriden with M665
float delta_radius= DELTA_RADIUS;
float delta_tower1_x= -SIN_60*delta_radius; // front left tower
float delta_tower1_y= -COS_60*delta_radius;
float delta_tower2_x=  SIN_60*delta_radius; // front right tower
float delta_tower2_y= -COS_60*delta_radius;
float delta_tower3_x= 0.0;                  // back middle tower
float delta_tower3_y= delta_radius;
float delta_diagonal_rod= DELTA_DIAGONAL_ROD;
float delta_diagonal_rod_2= sq(delta_diagonal_rod);
float delta_segments_per_second= DELTA_SEGMENTS_PER_SECOND;
#endif

#ifdef SCARA                              // Build size scaling
float axis_scaling[3]={1,1,1};  // Build size scaling, default to 1
#endif

bool cancel_heatup = false ;

#ifdef FILAMENT_SENSOR
//Variables for Filament Sensor input
float filament_width_nominal=DEFAULT_NOMINAL_FILAMENT_DIA;  //Set nominal filament width, can be changed with M404
bool filament_sensor=false;  //M405 turns on filament_sensor control, M406 turns it off
float filament_width_meas=DEFAULT_MEASURED_FILAMENT_DIA; //Stores the measured filament diameter
signed char measurement_delay[MAX_MEASUREMENT_DELAY+1];  //ring buffer to delay measurement  store extruder factor after subtracting 100
int delay_index1=0;  //index into ring buffer
int delay_index2=-1;  //index into ring buffer - set to -1 on startup to indicate ring buffer needs to be initialized
float delay_dist=0; //delay distance counter
int meas_delay_cm = MEASUREMENT_DELAY_CM;  //distance delay setting
#endif

//===========================================================================
//=============================Private Variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};

#ifndef DELTA
static float delta[3] = {0.0, 0.0, 0.0};
#endif

//Rapduch
float z_restaurada;

bool flag_continue_calib = false;
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the command string like X, Y, Z, E, etc

const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;

static uint8_t tmp_extruder;


bool Stopped=false;

#if NUM_SERVOS > 0
Servo servos[NUM_SERVOS];
#endif

bool CooldownNoWait = true;
bool target_direction;

//Insert variables if CHDK is defined
#ifdef CHDK
unsigned long chdkHigh = 0;
boolean chdkActive = false;
#endif

//===========================================================================
//=============================Routines======================================
//===========================================================================

void get_arc_coordinates();
bool setTargetedHotend(int code);

void serial_echopair_P(const char *s_P, float v)
{ serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
{ serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
{ serialprintPGM(s_P); SERIAL_ECHO(v); }

extern "C"{
	extern unsigned int __bss_end;
	extern unsigned int __heap_start;
	extern void *__brkval;

	int freeMemory() {
		int free_memory;

		if((int)__brkval == 0)
		free_memory = ((int)&free_memory) - ((int)&__bss_end);
		else
		free_memory = ((int)&free_memory) - ((int)__brkval);

		return free_memory;
	}
}

//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
	if(buflen < BUFSIZE)
	{
		//this is dangerous if a mixing of serial and this happens
		strcpy(&(cmdbuffer[bufindw][0]),cmd);
		SERIAL_ECHO_START;
		SERIAL_ECHOPGM("enqueing \"");
		SERIAL_ECHO(cmdbuffer[bufindw]);
		SERIAL_ECHOLNPGM("\"");
		bufindw= (bufindw + 1)%BUFSIZE;
		buflen += 1;
	}
}

void enquecommand_P(const char *cmd)
{
	if(buflen < BUFSIZE)
	{
		//this is dangerous if a mixing of serial and this happens
		strcpy_P(&(cmdbuffer[bufindw][0]),cmd);
		SERIAL_ECHO_START;
		SERIAL_ECHOPGM("enqueing \"");
		SERIAL_ECHO(cmdbuffer[bufindw]);
		SERIAL_ECHOLNPGM("\"");
		bufindw= (bufindw + 1)%BUFSIZE;
		buflen += 1;
	}
}

void setup_killpin()
{
	#if defined(KILL_PIN) && KILL_PIN > -1
	pinMode(KILL_PIN,INPUT);
	WRITE(KILL_PIN,HIGH);
	#endif
}

void setup_photpin()
{
	#if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
	SET_OUTPUT(PHOTOGRAPH_PIN);
	WRITE(PHOTOGRAPH_PIN, LOW);
	#endif
}

void setup_powerhold()
{
	#if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
	SET_OUTPUT(SUICIDE_PIN);
	WRITE(SUICIDE_PIN, HIGH);
	#endif
	#if defined(PS_ON_PIN) && PS_ON_PIN > -1
	SET_OUTPUT(PS_ON_PIN);
	#if defined(PS_DEFAULT_OFF)
	WRITE(PS_ON_PIN, PS_ON_ASLEEP);
	#else
	WRITE(PS_ON_PIN, PS_ON_AWAKE);
	#endif
	#endif
}

void suicide()
{
	#if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
	SET_OUTPUT(SUICIDE_PIN);
	WRITE(SUICIDE_PIN, LOW);
	#endif
}

void servo_init()
{
	#if (NUM_SERVOS >= 1) && defined(SERVO0_PIN) && (SERVO0_PIN > -1)
	servos[0].attach(SERVO0_PIN);
	#endif
	#if (NUM_SERVOS >= 2) && defined(SERVO1_PIN) && (SERVO1_PIN > -1)
	servos[1].attach(SERVO1_PIN);
	#endif
	#if (NUM_SERVOS >= 3) && defined(SERVO2_PIN) && (SERVO2_PIN > -1)
	servos[2].attach(SERVO2_PIN);
	#endif
	#if (NUM_SERVOS >= 4) && defined(SERVO3_PIN) && (SERVO3_PIN > -1)
	servos[3].attach(SERVO3_PIN);
	#endif
	#if (NUM_SERVOS >= 5)
	#error "TODO: enter initalisation code for more servos"
	#endif

	// Set position of Servo Endstops that are defined
	#ifdef SERVO_ENDSTOPS
		for(int8_t i = 0; i < 3; i++)
		{
			if(servo_endstops[i] > -1) {
				servos[servo_endstops[i]].write(servo_endstop_angles[i * 2 + 1]);
			}
		}
	#endif

	#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
		delay(PROBE_SERVO_DEACTIVATION_DELAY);
		servos[servo_endstops[Z_AXIS]].detach();
	#endif
}


void setup()
{
	
	setup_killpin();
	setup_powerhold();
	
	MYSERIAL.begin(BAUDRATE);

	SERIAL_PROTOCOLLNPGM(VERSION_STRING);
	SERIAL_ECHO_START;
	SERIAL_PROTOCOLLNPGM("start");
	
	
	Serial.println("BCN3D Sigma");
	
	//LCD START routine
	#ifdef SIGMA_TOUCH_SCREEN

			
		#if MOTHERBOARD == BCN3D_BOARD
			MYSERIAL_SCREEN.begin(200000);			
	
			genie.Begin(MYSERIAL_SCREEN);   // Use Serial3  for talking to the Genie Library, and to the 4D Systems display
			genie.AttachEventHandler(myGenieEventHandler); // Attach the user function Event Handler for processing events
			// Reset the Display
			// THIS IS IMPORTANT AND CAN PREVENT OUT OF SYNC ISSUES, SLOW SPEED RESPONSE ETC
	
			pinMode(RESETLINE, OUTPUT);  // Set Output (4D Arduino Adaptor V2 - Display Reset)
			digitalWrite(RESETLINE, 0);  // Reset the Display
			delay(100);
			digitalWrite(RESETLINE, 1);  // unReset the Display
	
			delay(4500); //showing the splash screen			
	
			// loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
			//Config_RetrieveSettings();
	
			/*if (quick_guide) {
				genie.WriteStr(3,VERSION_STRING);						
				delay(2500);
				Serial.println("Welcome by first time to SIGMA");
				
				Config_StoreSettings(); //Store changes			
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WELCOME,0);
				surfing_utilities=true;				
				
			} else {*/
				genie.WriteStr(3,VERSION_STRING);
				delay(2500);
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
				// loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
					Config_RetrieveSettings();
			//}
			//Turn the Display on (Contrast) - (Not needed but illustrates how)
			/*for(int i = 0;i<16;i++){				
				genie.WriteContrast(i);	
				delay(1500);
			}*/
			
		#endif	


	#endif

	
	
	/*//Enabling RELE ( Stepper Drivers Power )
	#if MOTHERBOARD==BCN3D_BOARD //BCNElectronics v1
	//enable 24V
	pinMode(RELAY, OUTPUT);
	digitalWrite(RELAY, LOW);
	//delay(50);
	digitalWrite(RELAY, HIGH);
	//delay(50);
	
	//pinMode(RED,OUTPUT);
	//pinMode(GREEN,OUTPUT);
	//pinMode(BLUE,OUTPUT);

	//analogWrite(RED,177);
	//analogWrite(GREEN,177);
	//analogWrite(BLUE,127);//Turn printer Blue rgb
	
	analogWrite(RED,255);
	analogWrite(GREEN,255);
	analogWrite(BLUE,255);
	#endif*/
	

	tp_init();    // Initialize temperature loop
	plan_init();  // Initialize planner;
	watchdog_init();
	st_init();    // Initialize stepper, this enables interrupts!
	setup_photpin();
	servo_init();
	
	
	//lcd_init();
	//Enabling RELAY ( Stepper Drivers Power )
	#if MOTHERBOARD == BCN3D_BOARD	
		pinMode(RED,OUTPUT);
		pinMode(GREEN,OUTPUT);
		pinMode(BLUE,OUTPUT);
		//Setting the LEDs at full power -> WHITE
		digitalWrite(RED,HIGH);
		digitalWrite(GREEN,HIGH);
		digitalWrite(BLUE,HIGH);
		
		//enable 24V
		pinMode(RELAY, OUTPUT);
		digitalWrite(RELAY, LOW);
		delay(1);
		digitalWrite(RELAY, HIGH); //Relay On
	#endif

	#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
	SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
	#endif

	#ifdef DIGIPOT_I2C
	digipot_i2c_init();
	#endif
	#ifdef Z_PROBE_SLED
	pinMode(SERVO0_PIN, OUTPUT);
	digitalWrite(SERVO0_PIN, LOW); // turn it off
	#endif // Z_PROBE_SLED


	#ifndef SIGMA_TOUCH_SCREEN  //Print First Gcode
	//char cmd[30];
	//char* c;
	//card.getfilename(0); //First gcode
	//sprintf_P(cmd, PSTR("M23 %s"), card.filename);
	//for(c = &cmd[4]; *c; c++)
	//{
	//*c = tolower(*c);
	//}
	//enquecommand(cmd);
	//enquecommand_P(PSTR("M24"));
	#endif
}


void loop()
{
	if(buflen < (BUFSIZE-1))
	get_command();
	#ifdef SDSUPPORT
	card.checkautostart(false);
	#endif
	if(buflen)
	{
		#ifdef SDSUPPORT
		if(card.saving)
		{
			if(strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL)
			{
				card.write_command(cmdbuffer[bufindr]);
				if(card.logging)
				{
					process_commands();
				}
				else
				{
					SERIAL_PROTOCOLLNPGM(MSG_OK);
				}
			}
			else
			{
				card.closefile();
				SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
			}
		}
		else
		{
			process_commands();
		}
		#else
		process_commands();
		#endif //SDSUPPORT
		buflen = (buflen-1);
		bufindr = (bufindr + 1)%BUFSIZE;
	}
	//check heater every n milliseconds
	manage_heater();
	manage_inactivity();
	checkHitEndstops();
	
	//lcd_update();
	#ifdef SIGMA_TOUCH_SCREEN
		touchscreen_update();
	#endif
	//if (processing) updateprocessing();
	//#ifndef SIGMA_TOUCH_SCREEN //JUST PRINT FIRST GCODE ON SD
	//if (firstime)
	//{
	//SD_firstPrint();
	//firstime=false;
	//}
	//#endif
	
}

void SD_firstPrint (){
	char cmd[30];
	char* c;
	card.getfilename(0);
	sprintf_P(cmd, PSTR("M23 %s"), card.filename);
	for(c = &cmd[4]; *c; c++)
	{
		*c = tolower(*c);
	}
	enquecommand(cmd);
	enquecommand_P(PSTR("M24"));
}

int getBuflen ()
{
	return buflen;
}


#ifdef SIGMA_TOUCH_SCREEN
//Rapduch
void touchscreen_update() //Updates the Serial Communications with the screen
{
	//static keyword specifies that the variable retains its state between calls to the function
	static uint32_t waitPeriod = millis();

	//if(card.sdprinting && is_on_printing_screen)
	if(card.sdprinting)
	{
		if (millis() >= waitPeriod)
		{
			int tHotend=int(degHotend(0));
			int tHotend1=int(degHotend(1));
			int tBed=int(degBed() + 0.5);
			
			//Rapduch
			//Edit for final TouchScreen
			char buffer[256];
			sprintf(buffer, "%3d %cC",tHotend,0x00B0);
			//Serial.println(buffer);
			genie.WriteStr(STRING_PRINTING_NOZZ1,buffer);
			
			sprintf(buffer, "%3d %cC",tHotend1,0x00B0);
			//Serial.println(buffer);
			genie.WriteStr(STRING_PRINTING_NOZZ2,buffer);
			
			sprintf(buffer, "%2d %cC",tBed,0x00B0);
			//Serial.println(buffer);
			genie.WriteStr(STRING_PRINTING_BED,buffer);
			
			sprintf(buffer, "% 3d %%",card.percentDone());
			//Serial.println(buffer);
			genie.WriteStr(STRING_PRINTING_PERCENT,buffer);
			
			sprintf(buffer, "% 3d %%",feedmultiply);
			//Serial.println(buffer);
			genie.WriteStr(STRINGS_PRINTING_FEED,buffer);
			
			waitPeriod=5000+millis();	//Every 5s
		}		 
	}
	else if (processing){
		if (millis() >= waitPeriod){			
			static int processing_state = 0;
			genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_PROCESSING,processing_state);
			if (processing_state == 0) processing_state = (processing_state+1)%3;
			else processing_state = (processing_state+1)%3;
			waitPeriod=180+millis();
		}
	}
	/*else if (heatting){
		if (millis() >= waitPeriod){			
			static int heatting_state = 0;
			genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_HEATTING,heatting);
			if (heatting_state == 0) heatting_state = (heatting_state+1)%6;
			else heatting_state = (heatting_state+1)%6;
			waitPeriod=16+millis();
		}
	}*/
	else if (surfing_utilities)
	{
		if (millis() >= waitPeriod)
		{
			int tHotend=int(degHotend(0));
			int tHotend1=int(degHotend(1));
			int tBed=int(degBed() + 0.5);
			char buffer[256];
			
			//Rapduch
			//Edit for final TouchScreen
			
			sprintf(buffer, "%3d %cC",tHotend,0x00B0);
			//Serial.println(buffer);
			genie.WriteStr(STRING_TEMP_NOZZ1,buffer);
			
			sprintf(buffer, "%3d %cC",tHotend1,0x00B0);
			//Serial.println(buffer);
			genie.WriteStr(STRING_TEMP_NOZZ2,buffer);
			
			sprintf(buffer, "%2d %cC",tBed,0x00B0);
			//Serial.println(buffer);
			genie.WriteStr(STRING_TEMP_BED,buffer);
			
			genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, current_position[X_AXIS]);
			genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, current_position[Y_AXIS]);
			genie.WriteObject(GENIE_OBJ_LED_DIGITS, 2, current_position[Z_AXIS]);
			
			sprintf(buffer, "%3d %cC",tHotend,0x00B0);
			//Serial.println(buffer);
			genie.WriteStr(STRING_PURGE_LEFT_TEMP,buffer);
			
			sprintf(buffer, "%3d %cC",tHotend1,0x00B0);
			//Serial.println(buffer);
			genie.WriteStr(STRING_PURGE_RIGHT_TEMP,buffer);
			
			#if EXTRUDERS > 1
			// Check if preheat for insert_FIL is done ////////////////////////////////////////////////////////////////////
			if ((degHotend(0) >= (degTargetHotend0()-5)) && (degHotend(1) >= (degTargetHotend1()-5)) && is_changing_filament){
			// if we want to add user setting temp, we should control if is heating			
				Serial.println("temp ok");				
				
				Serial.println("Ready to Insert/Remove");
				//We have preheated correctly
				if (filament_mode =='I'){		
					heatting = false;								
					genie.WriteStr(STRING_FILAMENT,"Press GO and keep pushing the filament \n until starts being pulled");
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INSERT_FIL,0);
					genie.WriteStr(STRING_FILAMENT,"Press GO and keep pushing the filament \n until starts being pulled");
				}
				else if (filament_mode =='R')
				{
					heatting = false;	
					genie.WriteStr(STRING_FILAMENT,"Press GO to Remove Filament,\n move the spool to save the filament");
					genie.WriteObject(GENIE_OBJ_FORM,FORM_REMOVE_FIL,0);
					genie.WriteStr(STRING_FILAMENT,"Press GO to Remove Filament,\n move the spool to save the filament");
					
				}
				else
				{
					heatting = false;	
					genie.WriteStr(STRING_FILAMENT,"Press GO to Purge Filament"); 
					genie.WriteObject(GENIE_OBJ_FORM,FORM_PURGE_FIL,0);
				}
				is_changing_filament=false; //Reset changing filament control
			}
			#endif //Extruders > 1
			waitPeriod=1000+millis(); // Every Second
		}
	}	
	else
	{
		//Serial.println("NOTHING DONE");//Do always...
	}
	
	//waitPeriod=250+millis();
	genie.DoEvents(); //Processes the TouchScreen Queued Events. Calls LCD_Handler.h ->myGenieEventHandler()
}
#endif //SIGMA TOUCHSCREEN


void get_command()
{
	while( MYSERIAL.available() > 0  && buflen < BUFSIZE) {
		serial_char = MYSERIAL.read();
		if(serial_char == '\n' ||
		serial_char == '\r' ||
		(serial_char == ':' && comment_mode == false) ||
		serial_count >= (MAX_CMD_SIZE - 1) )
		{
			if(!serial_count) { //if empty line
				comment_mode = false; //for new command
				return;
			}
			cmdbuffer[bufindw][serial_count] = 0; //terminate string
			if(!comment_mode){
				comment_mode = false; //for new command
				fromsd[bufindw] = false;
				if(strchr(cmdbuffer[bufindw], 'N') != NULL)
				{
					strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
					gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
					if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {
						SERIAL_ERROR_START;
						SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
						SERIAL_ERRORLN(gcode_LastN);
						//Serial.println(gcode_N);
						FlushSerialRequestResend();
						serial_count = 0;
						return;
					}

					if(strchr(cmdbuffer[bufindw], '*') != NULL)
					{
						byte checksum = 0;
						byte count = 0;
						while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
						strchr_pointer = strchr(cmdbuffer[bufindw], '*');

						if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
							SERIAL_ERROR_START;
							SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
							SERIAL_ERRORLN(gcode_LastN);
							FlushSerialRequestResend();
							serial_count = 0;
							return;
						}
						//if no errors, continue parsing
					}
					else
					{
						SERIAL_ERROR_START;
						SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
						SERIAL_ERRORLN(gcode_LastN);
						FlushSerialRequestResend();
						serial_count = 0;
						return;
					}

					gcode_LastN = gcode_N;
					//if no errors, continue parsing
				}
				else  // if we don't receive 'N' but still see '*'
				{
					if((strchr(cmdbuffer[bufindw], '*') != NULL))
					{
						SERIAL_ERROR_START;
						SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
						SERIAL_ERRORLN(gcode_LastN);
						serial_count = 0;
						return;
					}
				}
				if((strchr(cmdbuffer[bufindw], 'G') != NULL)){
					strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
					switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
						case 0:
						case 1:
						case 2:
						case 3:
						if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
							#ifdef SDSUPPORT
							if(card.saving)
							break;
							#endif //SDSUPPORT
							SERIAL_PROTOCOLLNPGM(MSG_OK);
						}
						else {
							SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
							LCD_MESSAGEPGM(MSG_STOPPED);
						}
						break;
						default:
						break;
					}

				}

				//If command was e-stop process now
				if(strcmp(cmdbuffer[bufindw], "M112") == 0)
				kill();
				
				bufindw = (bufindw + 1)%BUFSIZE;
				buflen += 1;
			}
			serial_count = 0; //clear buffer
		}
		else
		{
			if(serial_char == ';') comment_mode = true;
			if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
		}
	}
	#ifdef SDSUPPORT
	if(!card.sdprinting || serial_count!=0){ //Detects if printer is paused
		#ifdef SIGMA_TOUCH_SCREEN
		//Rapduch
		static long waitperiod=millis();
		/*if (millis()>=waitperiod){
		//screen_status="Paused...";
		//genie.WriteStr(6,"Paused..."); //Print Paused on screen Status
		
		waitperiod=millis()+500;
		////
		}*/
		
		//*********PAUSE POSITION AND RESUME POSITION IN PROBES
		if (flag_pause){
			enquecommand_P(((PSTR("G69"))));
			flag_pause = false;
			Serial.println("pause detected");
		}
		
		//****************************************************/
		
		#endif
		return;
	}

	//'#' stops reading from SD to the buffer prematurely, so procedural macro calls are possible
	// if it occurs, stop_buffering is triggered and the buffer is ran dry.
	// this character _can_ occur in serial com, due to checksums. however, no checksums are used in SD printing

	static bool stop_buffering=false;
	if(buflen==0) stop_buffering=false;

	while( !card.eof()  && buflen < BUFSIZE && !stop_buffering) {
		int16_t n=card.get();
		serial_char = (char)n;
		if(serial_char == '\n' ||
		serial_char == '\r' ||
		(serial_char == '#' && comment_mode == false) ||
		(serial_char == ':' && comment_mode == false) ||
		serial_count >= (MAX_CMD_SIZE - 1)||n==-1)
		{
			if(card.eof()){
				SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
				stoptime=millis();
				char time[30];
				unsigned long t=(stoptime-starttime)/1000;
				int hours, minutes;
				minutes=(t/60)%60;
				hours=t/60/60;
				sprintf_P(time, PSTR("%i hours %i minutes"),hours, minutes);
				SERIAL_ECHO_START;
				SERIAL_ECHOLN(time);
				lcd_setstatus(time);
				card.printingHasFinished();
				card.checkautostart(true);

			}
			if(serial_char=='#')
			stop_buffering=true;

			if(!serial_count)
			{
				comment_mode = false; //for new command
				return; //if empty line
			}
			cmdbuffer[bufindw][serial_count] = 0; //terminate string
			//      if(!comment_mode){
			fromsd[bufindw] = true;
			buflen += 1;
			bufindw = (bufindw + 1)%BUFSIZE;
			//      }
			comment_mode = false; //for new command
			serial_count = 0; //clear buffer
		}
		else
		{
			if(serial_char == ';') comment_mode = true;
			if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
		}
	}
	#endif //SDSUPPORT

}


float code_value()
{
	return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()
{
	return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
	strchr_pointer = strchr(cmdbuffer[bufindr], code);
	return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
static inline type pgm_read_any(const type *p)  \
{ return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
{ X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
{ return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

#ifdef DUAL_X_CARRIAGE
#if EXTRUDERS == 1 || defined(COREXY) \
|| !defined(X2_ENABLE_PIN) || !defined(X2_STEP_PIN) || !defined(X2_DIR_PIN) \
|| !defined(X2_HOME_POS) || !defined(X2_MIN_POS) || !defined(X2_MAX_POS) \
|| !defined(X_MAX_PIN) || X_MAX_PIN < 0
#error "Missing or invalid definitions for DUAL_X_CARRIAGE mode."
#endif
#if X_HOME_DIR != -1 || X2_HOME_DIR != 1
#error "Please use canonical x-carriage assignment" // the x-carriages are defined by their homing directions
#endif

#define DXC_FULL_CONTROL_MODE 0
#define DXC_AUTO_PARK_MODE    1
#define DXC_DUPLICATION_MODE  2
static int dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;

float x_home_pos(int extruder) {
	if (extruder == 0)
	return base_home_pos(X_AXIS) + add_homing[X_AXIS];
	else
	// In dual carriage mode the extruder offset provides an override of the
	// second X-carriage offset when homed - otherwise X2_HOME_POS is used.
	// This allow soft recalibration of the second extruder offset position without firmware reflash
	// (through the M218 command).
	return (extruder_offset[X_AXIS][1] > 0) ? extruder_offset[X_AXIS][1] : X2_HOME_POS;
}

static int x_home_dir(int extruder) {
	return (extruder == 0) ? X_HOME_DIR : X2_HOME_DIR;
}

static float inactive_extruder_x_pos = X2_MAX_POS; // used in mode 0 & 1
static bool active_extruder_parked = false; // used in mode 1 & 2
static float raised_parked_position[NUM_AXIS]; // used in mode 1
static unsigned long delayed_move_time = 0; // used in mode 1
static float duplicate_extruder_x_offset = DEFAULT_DUPLICATION_X_OFFSET; // used in mode 2
static float duplicate_extruder_temp_offset = 0; // used in mode 2
bool extruder_duplication_enabled = false; // used in mode 2
#endif //DUAL_X_CARRIAGE

static void axis_is_at_home(int axis) {
	#ifdef DUAL_X_CARRIAGE
	if (axis == X_AXIS) {
		if (active_extruder != 0) {
			current_position[X_AXIS] = x_home_pos(active_extruder);
			min_pos[X_AXIS] =          X2_MIN_POS;
			//Rapduch : Changing to only allow the Extruder offset set the max position for right extruder
			max_pos[X_AXIS] =          extruder_offset[X_AXIS][1];
			//max_pos[X_AXIS] =          max(extruder_offset[X_AXIS][1], X2_MAX_POS);
			return;
		}
		else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0) {
			current_position[X_AXIS] = base_home_pos(X_AXIS) + add_homing[X_AXIS];
			min_pos[X_AXIS] =          base_min_pos(X_AXIS) + add_homing[X_AXIS];
			max_pos[X_AXIS] =          min(base_max_pos(X_AXIS) + add_homing[X_AXIS],
			max(extruder_offset[X_AXIS][1], X2_MAX_POS) - duplicate_extruder_x_offset);
			return;
		}
	}
	#endif
	current_position[axis] = base_home_pos(axis) + add_homing[axis];
	min_pos[axis] =          base_min_pos(axis) + add_homing[axis];
	max_pos[axis] =          base_max_pos(axis) + add_homing[axis];

}

#ifdef ENABLE_AUTO_BED_LEVELING
#ifdef AUTO_BED_LEVELING_GRID
static void set_bed_level_equation_lsq(double *plane_equation_coefficients)
{
	vector_3 planeNormal = vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1);
	planeNormal.debug("planeNormal");
	plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
	//bedLevel.debug("bedLevel");

	//plan_bed_level_matrix.debug("bed level before");
	//vector_3 uncorrected_position = plan_get_position_mm();
	//uncorrected_position.debug("position before");

	vector_3 corrected_position = plan_get_position();
	//    corrected_position.debug("position after");
	current_position[X_AXIS] = corrected_position.x;
	current_position[Y_AXIS] = corrected_position.y;
	current_position[Z_AXIS] = corrected_position.z;

	// put the bed at 0 so we don't go below it.
	current_position[Z_AXIS] = zprobe_zoffset; // in the lsq we reach here after raising the extruder due to the loop structure

	plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

#else // not AUTO_BED_LEVELING_GRID

static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3) {

	plan_bed_level_matrix.set_to_identity();

	#ifdef Z_SIGMA_AUTOLEVEL
	float probe_x1 = (X_SIGMA_PROBE_1_LEFT_EXTR+((X_SIGMA_PROBE_1_RIGHT_EXTR-X_SIGMA_PROBE_1_LEFT_EXTR)/2));
	float probe_y1 = Y_SIGMA_PROBE_1_RIGHT_EXTR;
	vector_3 pt1 = vector_3(probe_x1, probe_y1, z_at_pt_1);
	//Rapduch
	Serial.print("Setting Vector1 X:");
	Serial.print(probe_x1);
	Serial.print(", Y:");
	Serial.print(probe_y1);
	Serial.print(", Z:");
	Serial.println(z_at_pt_1);
	vector_3 pt2 = vector_3(X_SIGMA_PROBE_2_LEFT_EXTR, Y_SIGMA_PROBE_2_LEFT_EXTR, z_at_pt_2);
	//Rapduch
	Serial.print("Setting Vector2 X:");
	Serial.print(X_SIGMA_PROBE_2_LEFT_EXTR);
	Serial.print(", Y:");
	Serial.print(Y_SIGMA_PROBE_2_LEFT_EXTR);
	Serial.print(", Z:");
	Serial.println(z_at_pt_2);
	vector_3 pt3 = vector_3(X_SIGMA_PROBE_3_LEFT_EXTR, Y_SIGMA_PROBE_3_LEFT_EXTR, z_at_pt_3);
	//Rapduch
	Serial.print("Setting Vector3 X:");
	Serial.print(X_SIGMA_PROBE_3_LEFT_EXTR);
	Serial.print(", Y:");
	Serial.print(Y_SIGMA_PROBE_3_LEFT_EXTR);
	Serial.print(", Z:");
	Serial.println(z_at_pt_3);
	#else
	vector_3 pt1 = vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, z_at_pt_1);
	vector_3 pt2 = vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, z_at_pt_2);
	vector_3 pt3 = vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, z_at_pt_3);
	#endif
	vector_3 from_2_to_1 = (pt1 - pt2).get_normal();
	vector_3 from_2_to_3 = (pt3 - pt2).get_normal();
	vector_3 planeNormal = vector_3::cross(from_2_to_1, from_2_to_3).get_normal();
	planeNormal = vector_3(planeNormal.x, planeNormal.y, abs(planeNormal.z));

	plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

	vector_3 corrected_position = plan_get_position();
	current_position[X_AXIS] = corrected_position.x;
	current_position[Y_AXIS] = corrected_position.y;
	current_position[Z_AXIS] = corrected_position.z;

	// put the bed at 0 so we don't go below it.
	current_position[Z_AXIS] = zprobe_zoffset;

	plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

int sentit (float dz)
{
	int sent;
	if (dz > 0)
	{
		sent = 1;//Horari
	}
	else
	{
		sent = -1;//Antihorari
	}
	return sent;
}

float voltes (float dz)
{
	float volt = dz/PAS_M5;
	return volt;
}

int aprox (float voltes)
{
	float res =0.125; //resolució d' 1/8 de volta
	float aprox_raw = voltes/res;
	int aprox = round(aprox_raw-0.05); // Arrodoniment de les voltes
	return aprox;
}

#endif // AUTO_BED_LEVELING_GRID


static void run_z_probe() {
	plan_bed_level_matrix.set_to_identity();
	feedrate = homing_feedrate[Z_AXIS];

	// move up until you find the bed
	float zPosition = -10;
	plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
	st_synchronize();

	// we have to let the planner know where we are right now as it is not where we said to go.
	zPosition = st_get_position_mm(Z_AXIS);
	plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

	// move down the retract distance
	zPosition += (home_retract_mm(Z_AXIS)/2);
	plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], (feedrate*1.5)/60, active_extruder);
	st_synchronize();

	// move back up slowly to find bed
	feedrate = homing_feedrate[Z_AXIS]/4;
	zPosition -= home_retract_mm(Z_AXIS) * 2;
	plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], (feedrate*1.5)/60, active_extruder);
	st_synchronize();

	current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
	// make sure the planner knows where we are as it may be a bit different than we last said to move to
	plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

static void do_blocking_move_to(float x, float y, float z) {
	float oldFeedRate = feedrate;

	feedrate = XY_TRAVEL_SPEED*1.5;

	current_position[X_AXIS] = x;
	current_position[Y_AXIS] = y;
	current_position[Z_AXIS] = z;
	plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
	st_synchronize();

	feedrate = oldFeedRate;
}

static void do_blocking_move_relative(float offset_x, float offset_y, float offset_z) {
	do_blocking_move_to(current_position[X_AXIS] + offset_x, current_position[Y_AXIS] + offset_y, current_position[Z_AXIS] + offset_z);
}

static void setup_for_endstop_move() {
	saved_feedrate = feedrate;
	saved_feedmultiply = feedmultiply;
	feedmultiply = 100;
	previous_millis_cmd = millis();

	enable_endstops(true);
}

static void clean_up_after_endstop_move() {
	#ifdef ENDSTOPS_ONLY_FOR_HOMING
	enable_endstops(false);
	#endif

	feedrate = saved_feedrate;
	feedmultiply = saved_feedmultiply;
	previous_millis_cmd = millis();
}

static void engage_z_probe() {
	// Engage Z Servo endstop if enabled
	#ifdef SERVO_ENDSTOPS
	if (servo_endstops[Z_AXIS] > -1) {
		#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
		servos[servo_endstops[Z_AXIS]].attach(0);
		#endif
		servos[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2]);
		#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
		delay(PROBE_SERVO_DEACTIVATION_DELAY);
		servos[servo_endstops[Z_AXIS]].detach();
		#endif
	}
	#endif
}

static void retract_z_probe() {
	// Retract Z Servo endstop if enabled
	#ifdef SERVO_ENDSTOPS
	if (servo_endstops[Z_AXIS] > -1) {
		#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
		servos[servo_endstops[Z_AXIS]].attach(0);
		#endif
		servos[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2 + 1]);
		#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
		delay(PROBE_SERVO_DEACTIVATION_DELAY);
		servos[servo_endstops[Z_AXIS]].detach();
		#endif
	}
	#endif
}

/// Probe bed height at position (x,y), returns the measured z value
static float probe_pt(float x, float y, float z_before) {
	// move to right place
	do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before);
	#ifdef Z_SIGMA_AUTOLEVEL
		if (active_extruder == LEFT_EXTRUDER) //DEFAULT ACTIVE EXTRUDER (left)
		{
			do_blocking_move_to(x - X_SIGMA_PROBE_OFFSET_FROM_EXTRUDER, y - Y_SIGMA_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);
		}
		else
		{
			do_blocking_move_to(x - X_SIGMA_SECOND_PROBE_OFFSET_FROM_EXTRUDER, y - Y_SIGMA_SECOND_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);
		}
	#else
		do_blocking_move_to(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);
	#endif
	
	

	#ifndef Z_PROBE_SLED
		engage_z_probe();   // Engage Z Servo endstop if available
	#endif // Z_PROBE_SLED
		run_z_probe();
	float measured_z = current_position[Z_AXIS];
	#ifndef Z_PROBE_SLED
		retract_z_probe();
	#endif // Z_PROBE_SLED

	SERIAL_PROTOCOLPGM(MSG_BED);
	SERIAL_PROTOCOLPGM(" x: ");
	SERIAL_PROTOCOL(x);
	SERIAL_PROTOCOLPGM(" y: ");
	SERIAL_PROTOCOL(y);
	SERIAL_PROTOCOLPGM(" z: ");
	SERIAL_PROTOCOL(measured_z);
	SERIAL_PROTOCOLPGM("\n");
	return measured_z;
}

#endif // #ifdef ENABLE_AUTO_BED_LEVELING

#ifdef Z_SIGMA_HOME

//static void run_z_probe() {
//plan_bed_level_matrix.set_to_identity();
//feedrate = homing_feedrate[Z_AXIS];
//
//// move down until you find the bed
//float zPosition = -10;
//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
//st_synchronize();
//
//// we have to let the planner know where we are right now as it is not where we said to go.
//zPosition = st_get_position_mm(Z_AXIS);
//plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);
//
//// move up the retract distance
//zPosition += home_retract_mm(Z_AXIS);
//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
//st_synchronize();
//
//// move back down slowly to find bed
//feedrate = homing_feedrate[Z_AXIS]/4;
//zPosition -= home_retract_mm(Z_AXIS) * 2;
//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
//st_synchronize();
//
//current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
//// make sure the planner knows where we are as it may be a bit different than we last said to move to
//plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//}
//
//static void do_blocking_move_to(float x, float y, float z) {
//float oldFeedRate = feedrate;
//
//feedrate = XY_TRAVEL_SPEED;
//
//current_position[X_AXIS] = x;
//current_position[Y_AXIS] = y;
//current_position[Z_AXIS] = z;
//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
//st_synchronize();
//
//feedrate = oldFeedRate;
//}
//
//static void do_blocking_move_relative(float offset_x, float offset_y, float offset_z) {
//do_blocking_move_to(current_position[X_AXIS] + offset_x, current_position[Y_AXIS] + offset_y, current_position[Z_AXIS] + offset_z);
//}
//
//static void setup_for_endstop_move() {
//saved_feedrate = feedrate;
//saved_feedmultiply = feedmultiply;
//feedmultiply = 100;
//previous_millis_cmd = millis();
//
//enable_endstops(true);
//}
//
//static void clean_up_after_endstop_move() {
//#ifdef ENDSTOPS_ONLY_FOR_HOMING
//enable_endstops(false);
//#endif
//
//feedrate = saved_feedrate;
//feedmultiply = saved_feedmultiply;
//previous_millis_cmd = millis();
//}
//
//
///// Probe bed height at position (x,y), returns the measured z value
//static float probe_pt(float x, float y, float z_before) {
//// move to right place
//do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before);
//do_blocking_move_to(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);
//
//#ifndef Z_PROBE_SLED
//engage_z_probe();   // Engage Z Servo endstop if available
//#endif // Z_PROBE_SLED
//run_z_probe();
//float measured_z = current_position[Z_AXIS];
//#ifndef Z_PROBE_SLED
//retract_z_probe();
//#endif // Z_PROBE_SLED
//
//SERIAL_PROTOCOLPGM(MSG_BED);
//SERIAL_PROTOCOLPGM(" x: ");
//SERIAL_PROTOCOL(x);
//SERIAL_PROTOCOLPGM(" y: ");
//SERIAL_PROTOCOL(y);
//SERIAL_PROTOCOLPGM(" z: ");
//SERIAL_PROTOCOL(measured_z);
//SERIAL_PROTOCOLPGM("\n");
//return measured_z;
//}


#endif


static void homeaxis(int axis) {
	#define HOMEAXIS_DO(LETTER) \
	((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

	if (axis==X_AXIS ? HOMEAXIS_DO(X) :	axis==Y_AXIS ? HOMEAXIS_DO(Y) :	axis==Z_AXIS ? HOMEAXIS_DO(Z) :	0) {
		int axis_home_dir = home_dir(axis);		
		#ifdef DUAL_X_CARRIAGE
			if (axis == X_AXIS) axis_home_dir = x_home_dir(active_extruder);
		#endif

		current_position[axis] = 0;
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

		destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
		Serial.print("Move to: "); Serial.print(axis); Serial.print(" "); Serial.println(destination[axis]);
		feedrate = homing_feedrate[axis];
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
		st_synchronize();

		current_position[axis] = 0;
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		
		destination[axis] = -home_retract_mm(axis) * axis_home_dir;  //Actual retract
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
		st_synchronize();

		destination[axis] = 2*home_retract_mm(axis) * axis_home_dir;

		feedrate = homing_feedrate[axis]/2;
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder); //Slow
		st_synchronize();

		axis_is_at_home(axis);
		destination[axis] = current_position[axis];
		feedrate = 0.0;
		endstops_hit_on_purpose();
		axis_known_position[axis] = true;

	}
}


#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)


//Rapduch
void homeFromMain()
{
	homeaxis(X_AXIS);
	homeaxis(Y_AXIS);
	homeaxis(Z_AXIS);
}


void refresh_cmd_timeout(void)
{
	previous_millis_cmd = millis();
}

#ifdef FWRETRACT
void retract(bool retracting, bool swapretract = false) {
	if(retracting && !retracted[active_extruder]) {
		destination[X_AXIS]=current_position[X_AXIS];
		destination[Y_AXIS]=current_position[Y_AXIS];
		destination[Z_AXIS]=current_position[Z_AXIS];
		destination[E_AXIS]=current_position[E_AXIS];
		if (swapretract) {
			current_position[E_AXIS]+=retract_length_swap/volumetric_multiplier[active_extruder];
			} else {
			current_position[E_AXIS]+=retract_length/volumetric_multiplier[active_extruder];
		}
		plan_set_e_position(current_position[E_AXIS]);
		float oldFeedrate = feedrate;
		feedrate=retract_feedrate*60;
		retracted[active_extruder]=true;
		prepare_move();
		current_position[Z_AXIS]-=retract_zlift;
		#ifdef DELTA
		calculate_delta(current_position); // change cartesian kinematic to  delta kinematic;
		plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
		#else
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		#endif
		prepare_move();
		feedrate = oldFeedrate;
		} else if(!retracting && retracted[active_extruder]) {
		destination[X_AXIS]=current_position[X_AXIS];
		destination[Y_AXIS]=current_position[Y_AXIS];
		destination[Z_AXIS]=current_position[Z_AXIS];
		destination[E_AXIS]=current_position[E_AXIS];
		current_position[Z_AXIS]+=retract_zlift;
		#ifdef DELTA
		calculate_delta(current_position); // change cartesian kinematic  to  delta kinematic;
		plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
		#else
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		#endif
		//prepare_move();
		if (swapretract) {
			current_position[E_AXIS]-=(retract_length_swap+retract_recover_length_swap)/volumetric_multiplier[active_extruder];
			} else {
			current_position[E_AXIS]-=(retract_length+retract_recover_length)/volumetric_multiplier[active_extruder];
		}
		plan_set_e_position(current_position[E_AXIS]);
		float oldFeedrate = feedrate;
		feedrate=retract_recover_feedrate*60;
		retracted[active_extruder]=false;
		prepare_move();
		feedrate = oldFeedrate;
	}
} //retract
#endif //FWRETRACT

#ifdef Z_PROBE_SLED
//
// Method to dock/undock a sled designed by Charles Bell.
//
// dock[in]     If true, move to MAX_X and engage the electromagnet
// offset[in]   The additional distance to move to adjust docking location
//
static void dock_sled(bool dock, int offset=0) {
	int z_loc;
	
	if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
		LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
		return;
	}

	if (dock) {
		do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset,
		current_position[Y_AXIS],
		current_position[Z_AXIS]);
		// turn off magnet
		digitalWrite(SERVO0_PIN, LOW);
		} else {
		if (current_position[Z_AXIS] < (Z_RAISE_BEFORE_PROBING + 5))
		z_loc = Z_RAISE_BEFORE_PROBING;
		else
		z_loc = current_position[Z_AXIS];
		do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset,
		Y_PROBE_OFFSET_FROM_EXTRUDER, z_loc);
		// turn on magnet
		digitalWrite(SERVO0_PIN, HIGH);
	}
}
#endif

void process_commands()
{
	unsigned long codenum; //throw away variable
	char *starpos = NULL;
	#ifdef ENABLE_AUTO_BED_LEVELING
	float x_tmp, y_tmp, z_tmp, real_z;
	#endif
	if(code_seen('G'))
	{
		switch((int)code_value())
		{
			case 0: // G0 -> G1
			case 1: // G1
			if(Stopped == false) {
				get_coordinates(); // For X Y Z E F
				#ifdef FWRETRACT
				if(autoretract_enabled)
				if( !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
					float echange=destination[E_AXIS]-current_position[E_AXIS];
					if((echange<-MIN_RETRACT && !retracted) || (echange>MIN_RETRACT && retracted)) { //move appears to be an attempt to retract or recover
						current_position[E_AXIS] = destination[E_AXIS]; //hide the slicer-generated retract/recover from calculations
						plan_set_e_position(current_position[E_AXIS]); //AND from the planner
						retract(!retracted);
						return;
					}
				}
				#endif //FWRETRACT
				prepare_move();
				//ClearToSend();
				return;
			}
			break;
			
			#ifndef SCARA //disable arc support
			case 2: // G2  - CW ARC
			if(Stopped == false) {
				get_arc_coordinates();
				prepare_arc_move(true);
				return;
			}
			break;
			case 3: // G3  - CCW ARC
			if(Stopped == false) {
				get_arc_coordinates();
				prepare_arc_move(false);
				return;
			}
			break;
			#endif
			case 4: // G4 dwell
			LCD_MESSAGEPGM(MSG_DWELL);
			codenum = 0;
			if(code_seen('P')) codenum = code_value(); // milliseconds to wait
			if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

			st_synchronize();
			codenum += millis();  // keep track of when we started waiting
			previous_millis_cmd = millis();
			while(millis() < codenum) {
				manage_heater();
				manage_inactivity();
				//lcd_update();
				#ifdef SIGMA_TOUCH_SCREEN
				touchscreen_update();
				#endif
				
			}
			break;
			#ifdef FWRETRACT
			case 10: // G10 retract
			#if EXTRUDERS > 1
			retracted_swap[active_extruder]=(code_seen('S') && code_value_long() == 1); // checks for swap retract argument
			retract(true,retracted_swap[active_extruder]);
			#else
			retract(true);
			#endif
			break;
			case 11: // G11 retract_recover
			#if EXTRUDERS > 1
			retract(false,retracted_swap[active_extruder]);
			#else
			retract(false);
			#endif
			break;
			#endif //FWRETRACT
			
			
			case 28: { //G28 Home all Axis one at a time
				#ifdef ENABLE_AUTO_BED_LEVELING
				plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)
				#endif //ENABLE_AUTO_BED_LEVELING

				#ifdef Z_SIGMA_HOME
				int saved_active_extruder = active_extruder;
				Serial.print("Extruder active: ");
				Serial.println(saved_active_extruder);
				#endif
				
				saved_feedrate = feedrate;
				saved_feedmultiply = feedmultiply;
				feedmultiply = 100;
				previous_millis_cmd = millis();

				enable_endstops(true); //Activate endstops

				for(int8_t i=0; i < NUM_AXIS; i++) {
					destination[i] = current_position[i];
				}
				feedrate = 0.0;

				home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])));

				#if Z_HOME_DIR > 0                      // If homing away from BED do Z first. WE ARE NOT USING THIS IN SIGMA
				if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
					HOMEAXIS(Z);
				}
				#endif

				#ifdef QUICK_HOME
				if((home_all_axis)||( code_seen(axis_codes[X_AXIS]) && code_seen(axis_codes[Y_AXIS])) )  //first diagonal move
				{
					current_position[X_AXIS] = 0;current_position[Y_AXIS] = 0;

					#ifndef DUAL_X_CARRIAGE
					int x_axis_home_dir = home_dir(X_AXIS); //Set X directions
					#else
					int x_axis_home_dir = x_home_dir(active_extruder);
					extruder_duplication_enabled = false;
					#endif

					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					
					destination[X_AXIS] = 1.5 * max_length(X_AXIS) * x_axis_home_dir;
					destination[Y_AXIS] = 1.5 * max_length(Y_AXIS) * home_dir(Y_AXIS);
					
					feedrate = homing_feedrate[X_AXIS];
					
					if(homing_feedrate[Y_AXIS]<feedrate)
					feedrate = homing_feedrate[Y_AXIS];
					
					if (max_length(X_AXIS) > max_length(Y_AXIS)) {
						feedrate *= sqrt(pow(max_length(Y_AXIS) / max_length(X_AXIS), 2) + 1);
						} else {
						feedrate *= sqrt(pow(max_length(X_AXIS) / max_length(Y_AXIS), 2) + 1);
					}
					
					plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
					st_synchronize();

					axis_is_at_home(X_AXIS);
					axis_is_at_home(Y_AXIS);
					
					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					destination[X_AXIS] = current_position[X_AXIS];
					destination[Y_AXIS] = current_position[Y_AXIS];
					plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
					feedrate = 0.0;
					st_synchronize();
					endstops_hit_on_purpose();

					current_position[X_AXIS] = destination[X_AXIS];
					current_position[Y_AXIS] = destination[Y_AXIS];
					current_position[Z_AXIS] = destination[Z_AXIS];
				}
				#endif

				if((home_all_axis) || (code_seen(axis_codes[X_AXIS]))) // First do X
				{
					#ifdef DUAL_X_CARRIAGE
						#ifdef Z_SIGMA_HOME
							if(saved_active_extruder == RIGHT_EXTRUDER)
							{
								//active_extruder=LEFT_EXTRUDER; //Always use the same routine (left extruder probes)
							}
						#endif
						int tmp_extruder = active_extruder;
						extruder_duplication_enabled = false;
						active_extruder = !active_extruder;
						HOMEAXIS(X);
						inactive_extruder_x_pos = current_position[X_AXIS];
						active_extruder = tmp_extruder;
						HOMEAXIS(X);
						// reset state used by the different modes
						memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
						delayed_move_time = 0;
						active_extruder_parked = true;
					#else
						HOMEAXIS(X);
					#endif
				}

				if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
					HOMEAXIS(Y);
				}

				if(code_seen(axis_codes[X_AXIS]))
				{
					if(code_value_long() != 0) {
						current_position[X_AXIS]=code_value()+add_homing[0];
					}
				}

				if(code_seen(axis_codes[Y_AXIS])) {
					if(code_value_long() != 0) {
						current_position[Y_AXIS]=code_value()+add_homing[1];
					}
				}

				#if Z_HOME_DIR < 0   // If homing towards BED do Z last
				//Rapduch
				#ifdef Z_SIGMA_HOME
				if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
					
					
					if (saved_active_extruder == RIGHT_EXTRUDER){
						active_extruder=LEFT_EXTRUDER;
						axis_is_at_home(X_AXIS);
						plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
					}
					
					destination[X_AXIS] = round(Z_SIGMA_HOME_X_POINT-X_SIGMA_PROBE_OFFSET_FROM_EXTRUDER);
					destination[Y_AXIS] = round(Z_SIGMA_HOME_Y_POINT-Y_SIGMA_PROBE_OFFSET_FROM_EXTRUDER);
					destination[Z_AXIS] = Z_SIGMA_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
					feedrate = SIGMA_Z_HOME_TRAVEL_SPEED;
					current_position[Z_AXIS] = 0;
					
					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);//Left Extruder
					st_synchronize();
					current_position[X_AXIS] = destination[X_AXIS];
					current_position[Y_AXIS] = destination[Y_AXIS];
					HOMEAXIS(Z);
					
					Serial.println("Z SIGMA Homed");
					//At this point our probe is homed, no offset is added.

					//Now lets home with the second extruder:
					//st_synchronize();
					//
					//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+10, current_position[E_AXIS], feedrate, 1);
					////plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS]+10, current_position[E_AXIS], feedrate, 1);
					//destination[X_AXIS] = round(Z_SIGMA_HOME_SECOND_X_POINT);
					//destination[Y_AXIS] = round(Z_SIGMA_HOME_SECOND_Y_POINT);
					//destination[Z_AXIS] = Z_SIGMA_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
					//feedrate = XY_SIGMA_TRAVEL_SPEED;
					//current_position[Z_AXIS] = 0;
					//SERIAL_ECHO("Z SIGMA Homed");
					//
					//plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					//plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, tmp_extruder); //Second extruder(temporal)
					//current_position[X_AXIS] = destination[X_AXIS];
					//current_position[Y_AXIS] = destination[Y_AXIS];
					//HOMEAXIS(Z);
					
				}
				#else
				
				#ifndef Z_SAFE_HOMING
				if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
					#if defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
					destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
					feedrate = max_feedrate[Z_AXIS];
					plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
					st_synchronize();
					#endif
					HOMEAXIS(Z);
				}
				#else                   // Z Safe mode activated.
				if(home_all_axis) {
					destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
					destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
					destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
					feedrate = XY_TRAVEL_SPEED;
					current_position[Z_AXIS] = 0;

					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
					st_synchronize();
					current_position[X_AXIS] = destination[X_AXIS];
					current_position[Y_AXIS] = destination[Y_AXIS];

					HOMEAXIS(Z);
				}
				
				//Let's see if X and Y are homed and probe is inside bed area.
				if(code_seen(axis_codes[Z_AXIS])) {
					if ( (axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]) \
					&& (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER >= X_MIN_POS) \
					&& (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER <= X_MAX_POS) \
					&& (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER >= Y_MIN_POS) \
					&& (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER <= Y_MAX_POS)){
						
						current_position[Z_AXIS] = 0;
						plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
						destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
						feedrate = max_feedrate[Z_AXIS];
						plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
						st_synchronize();

						HOMEAXIS(Z);
						} else if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
						LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
						SERIAL_ECHO_START;
						SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
						} else {
						LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
						SERIAL_ECHO_START;
						SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
					}
				}
				#endif
				
				#endif
				
				#endif

				if(code_seen(axis_codes[Z_AXIS])) { //Sets Z position
					if(code_value_long() != 0) {
						current_position[Z_AXIS]=code_value()+add_homing[2];
					}
				}
				
				//Rapduch
				#ifdef Z_SIGMA_HOME
				if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
					current_position[Z_AXIS] += zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)
				}
				#else
				#ifdef ENABLE_AUTO_BED_LEVELING  //ADDING the Z offset
				//if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
				//current_position[Z_AXIS] += zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)
				//}
				#endif
				#endif

				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

				#ifdef ENDSTOPS_ONLY_FOR_HOMING
				enable_endstops(false);
				#endif

				feedrate = saved_feedrate;
				feedmultiply = saved_feedmultiply;
				previous_millis_cmd = millis();
				endstops_hit_on_purpose();

				//Rapduch
				#ifdef Z_SIGMA_HOME  //This to return the left extruder at Xhome position
				if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
					
					feedrate = homing_feedrate[Z_AXIS];
					current_position[Z_AXIS]+=Z_SIGMA_RAISE_AFTER_HOMING;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					current_position[X_AXIS] = x_home_pos(active_extruder);
					feedrate = homing_feedrate[X_AXIS];
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					current_position[Z_AXIS] = 0;
					feedrate = homing_feedrate[Z_AXIS];
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					st_synchronize();
					
					if(saved_active_extruder==RIGHT_EXTRUDER)
					{
						changeToolSigma(RIGHT_EXTRUDER); //Get again the same tool
						Serial.print("Extruder released active: ");
						Serial.println(saved_active_extruder);
					}
					////Activate Probing sequence
					//setup_for_endstop_move();
					//feedrate = homing_feedrate[Z_AXIS];
					//active_extruder=1; //extruder1(right) instead of extruder0(left)
					////tmp_extruder=active_extruder;
					//probe_pt(Z_SIGMA_HOME_SECOND_X_POINT,Z_SIGMA_HOME_SECOND_Y_POINT, Z_SIGMA_RAISE_BEFORE_HOMING);
					//clean_up_after_endstop_move();
				}
				#endif
				home_made = true;
			}break;


			//Rapduch for RepRapBCN Sigma
			#ifdef EXTRUDER_CALIBRATION_WIZARD
			case 40://G40 --> X Extruder calibration
			{
				Serial.println("Starting X Calibration Wizard");
				//1) Set temps and wait
				setTargetHotend0(PLA_PREHEAT_HOTEND_TEMP-5);
				setTargetHotend1(PLA_PREHEAT_HOTEND_TEMP-5);
				setTargetBed(max(bed_temp_l,bed_temp_r)-5);
				
				while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-10) && degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-10)&& degBed()<(PLA_PREHEAT_HPB_TEMP-10)){ //Waiting to heat the extruder
					
					manage_heater();
				}
				 
				delay(5000);
				//Raise for a layer of Z=0.2
				current_position[Z_AXIS]=0.65; //0.5 + 0.15 per ajustar una bona alçada
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60 , active_extruder);
				
				
				//2)Extruder one prints
				//Purge & up
				current_position[E_AXIS]+=15;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60 , active_extruder);
				//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+5, current_position[E_AXIS], 500/60 , active_extruder);
				//current_position[E_AXIS]-=4;
				current_position[E_AXIS]-=4;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, active_extruder);//Retrack
				
				st_synchronize();
				
				float mm_second_extruder[NUM_LINES];
				float i =  -0.5;
				mm_second_extruder[0] = i;
				for (int count = 1; count <= NUM_LINES; count++){
					
					mm_second_extruder[count] =  mm_second_extruder[count-1] + 0.1;
				}
				//float mm_second_extruder[9] = {19.6, 19.7, 19.8, 19.9, 20 ,20.1 ,20.2, 20.3, 20.4};

				float mm_each_extrusion = 10;
				float mm_left_offset = X_CALIB_STARTING_X;
				
				//int times = 9;
				for (int i=0;(i<(NUM_LINES));i++) //4 times
				{ 
					if (i == 0){
						
						//draw borders
						plan_buffer_line(current_position[X_AXIS],250+11+10, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						
						plan_buffer_line(mm_left_offset-10,250+11+10, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						st_synchronize();
						
						current_position[Z_AXIS]=0.2; //Move X and Z
						plan_buffer_line(mm_left_offset-10,250+11+10, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS]/2, active_extruder);//Retrack
						st_synchronize();
						
						current_position[E_AXIS]+=4; 
						plan_buffer_line(mm_left_offset-10,250+11+10, current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60, active_extruder);//Retrack
						st_synchronize();
						
						current_position[E_AXIS]+=((110)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*10),250+11+10, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						current_position[E_AXIS]+=((271-139)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*10),149-10, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						current_position[E_AXIS]+=((110)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						plan_buffer_line(mm_left_offset-10,149-10, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						current_position[E_AXIS]+=((271-139)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						plan_buffer_line(mm_left_offset-10,250+11+10, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						current_position[E_AXIS]-=2;
						plan_buffer_line(mm_left_offset-10,250+11+10, current_position[Z_AXIS],current_position[E_AXIS], INSERT_FAST_SPEED/60 , active_extruder);
						st_synchronize();
						current_position[Z_AXIS]=0.2;
						plan_buffer_line(mm_left_offset-10,250+11+10, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS]/2 , active_extruder);//Move X and Z
						st_synchronize();
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*i), 211, current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/ , active_extruder);						
						st_synchronize();
						//draw a one
						
						/*plan_buffer_line(mm_left_offset+(mm_each_extrusion*i),250+11, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*i),250+5, current_position[Z_AXIS], current_position[E_AXIS]+=1, max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*i),250+5, current_position[Z_AXIS], current_position[E_AXIS]-=3, max_feedrate[X_AXIS]/2 , active_extruder);
						st_synchronize();*/
					}
					/*if (i ==9){
						//draw a one
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*i)-1, 250+11,current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*i)-1, 250+5, current_position[Z_AXIS], current_position[E_AXIS]+=1, max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*i)-1, 250+5, current_position[Z_AXIS], current_position[E_AXIS]-=3, max_feedrate[X_AXIS]/2 , active_extruder);
						st_synchronize();
						//draw a zero
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*i)+4,250+11, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*i)+2,250+11,current_position[Z_AXIS], current_position[E_AXIS]+=1, max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*i)+2,250+5, current_position[Z_AXIS], current_position[E_AXIS]+=1, max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*i)+4,250+5, current_position[Z_AXIS], current_position[E_AXIS]+=1, max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*i)+4,250+11, current_position[Z_AXIS], current_position[E_AXIS]+=1, max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(mm_left_offset+(mm_each_extrusion*i)+4,250+11, current_position[Z_AXIS], current_position[E_AXIS]-=3, max_feedrate[X_AXIS]/2 , active_extruder);
						st_synchronize();
					}*/
					current_position[E_AXIS]+=2; //Move the retracked space
					plan_buffer_line(mm_left_offset+(mm_each_extrusion*i), 211, current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/ , active_extruder);					
					st_synchronize();
					
					current_position[E_AXIS]+=((250-200)*0.33*current_position[Z_AXIS]*10/55.119)*1.75;
					plan_buffer_line(mm_left_offset+(mm_each_extrusion*i), 261, current_position[Z_AXIS], current_position[E_AXIS], 1500/60, active_extruder);//Move Y and extrude
					
					current_position[E_AXIS]-=2;
					plan_buffer_line(mm_left_offset+(mm_each_extrusion*i), 261, current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, active_extruder);//Move Y and extrude
					
					if (i != 9) plan_buffer_line(mm_left_offset+(mm_each_extrusion*(i+1)), 211, current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//Retrack
					else plan_buffer_line(mm_left_offset+(mm_each_extrusion*i), 261, current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//Retrack
					st_synchronize();
				}
				
				current_position[X_AXIS]=mm_left_offset+(NUM_LINES-1)*mm_each_extrusion;
				current_position[Y_AXIS]=261;				
				current_position[E_AXIS]-=2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 50, active_extruder);//MORE Retrack
				
				current_position[Z_AXIS]+=0.2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 50, active_extruder);//MORE Retrack
				
				
				
				//2)Extruder 2 prints with corrections

				
				current_position[X_AXIS]= x_home_pos(LEFT_EXTRUDER);				
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//MORE Retrack
				current_position[Z_AXIS]-= 0.2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//MORE Retrack

				
				changeTool(1);

				
				current_position[Z_AXIS]=0.5;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 1000/60 , active_extruder); //Raise for a layer of Z=0.2
				//Purge & up
				current_position[E_AXIS]+=15;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60 , active_extruder);
				//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+5, current_position[E_AXIS], 500/60 , active_extruder);
				//current_position[E_AXIS]-=4;
				current_position[E_AXIS]-=4;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, active_extruder);//Retrack
				st_synchronize();				

				plan_buffer_line(current_position[X_AXIS], 149-5, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder); //Move Y and Z
				st_synchronize();
				
				//Second Extruder (correcting)
				for (int i=0; i<(NUM_LINES);i++) //N times
				{
					if (i == 0) {
						
						plan_buffer_line(mm_left_offset-5,149-5, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						st_synchronize();
						
						current_position[Z_AXIS]=0.2;
						plan_buffer_line(mm_left_offset-5,149-5, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						st_synchronize();
						
						current_position[E_AXIS]+=4;
						plan_buffer_line(mm_left_offset-5,149-5, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();		
										
						current_position[E_AXIS]+=((100)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						plan_buffer_line(mm_left_offset-5+(mm_each_extrusion*10),149-5, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						current_position[E_AXIS]+=((122)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						plan_buffer_line(mm_left_offset-5+(mm_each_extrusion*10),250+11+5, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						current_position[E_AXIS]+=((100)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						plan_buffer_line(mm_left_offset-5,250+11+5, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						current_position[E_AXIS]+=((122)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						plan_buffer_line(mm_left_offset-5,149-5, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						current_position[E_AXIS]-=2;
						plan_buffer_line(mm_left_offset-5,149-5, current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60 , active_extruder); //Move X
						st_synchronize();
						current_position[Z_AXIS]=0.2;
						plan_buffer_line(mm_left_offset-5,149-5, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS]/2 , active_extruder); //Move X
						st_synchronize();
						
						plan_buffer_line(mm_left_offset+(mm_second_extruder[i]+(mm_each_extrusion*(i))), 210, current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/ , active_extruder); //Move the retracked space						
						st_synchronize();
					}
					current_position[E_AXIS]+=2;
					plan_buffer_line(mm_left_offset+(mm_second_extruder[i]+(mm_each_extrusion*(i))), 210, current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/ , active_extruder); //Move the retracked space
					st_synchronize(); 
									
					current_position[E_AXIS]+=((199-149)*0.33*current_position[Z_AXIS]*10/55.119)*1.75;
					plan_buffer_line(mm_left_offset+(mm_second_extruder[i]+(mm_each_extrusion*(i))), 149, current_position[Z_AXIS], current_position[E_AXIS], 1500/60, active_extruder);//Move Y and extrude
					
					current_position[E_AXIS]-=2;
					plan_buffer_line(mm_left_offset+(mm_second_extruder[i]+(mm_each_extrusion*(i))), 149, current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, active_extruder);//Move Y and extrude
					
					if (i != 9) plan_buffer_line(mm_left_offset+(mm_second_extruder[i+1]+(mm_each_extrusion*(i+1))), 210, current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//Retrack
					else plan_buffer_line(mm_left_offset+(mm_second_extruder[i]+(mm_each_extrusion*(i))), 149, current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);
					//plan_buffer_line(mm_left_offset+(mm_each_extrusion*i), 150, current_position[Z_AXIS]+5, current_position[E_AXIS], 1500/60, active_extruder);//Lift Z
					st_synchronize();	
				}
				current_position[X_AXIS]=mm_left_offset+(mm_second_extruder[NUM_LINES-1]+(mm_each_extrusion*(NUM_LINES-1)));
				current_position[Y_AXIS]=149;
				current_position[E_AXIS]-=2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);
				current_position[Z_AXIS]+=0.2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);
				
				changeTool(0);
				
				current_position[Z_AXIS]-=0.2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);	
				current_position[Y_AXIS] = Y_MAX_POS;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], XY_TRAVEL_SPEED, active_extruder);
				//Go to Calibration select screen	
				genie.WriteObject(GENIE_OBJ_FORM,FORM_X_CALIB_SELECT,0);
				
				break;
			}
			#endif //EXTRUDER_CALIBRATION_WIZARD


			//Rapduch for RepRapBCN Sigma
			#ifdef EXTRUDER_CALIBRATION_WIZARD
			case 41://G41 --> Y Extruder calibration
			{
				Serial.println("Starting Y Calibration Wizard");
				//1) Set temps and wait
				setTargetHotend0(PLA_PREHEAT_HOTEND_TEMP-5);
				setTargetHotend1(PLA_PREHEAT_HOTEND_TEMP-5);
				setTargetBed(max(bed_temp_l,bed_temp_r)-5);
				
				while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-10) || degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-10) || degBed()<(PLA_PREHEAT_HPB_TEMP-10)){ //Waiting to heat the extruder
					
					manage_heater();
				}
				delay(5000);
				
				//Raise for a layer of Z=0.2
				current_position[Z_AXIS]=0.5;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 1000/60 , active_extruder);


				//2)Extruder one prints
				//Purge & up
				current_position[E_AXIS]+=15;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60 , active_extruder);				
				current_position[E_AXIS]-=4;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, active_extruder);//Retrack
				st_synchronize();
				
				
				float mm_second_extruder[NUM_LINES];
				float i = -0.5;
				mm_second_extruder[0] = i;
				for (int count = 1; count <= NUM_LINES; count++){
					
					mm_second_extruder[count] =  mm_second_extruder[count-1] + 0.1;
				}
				float mm_each_extrusion = 10;
				float mm_left_offset = 120;				
				float mm_y_offset = X_CALIB_STARTING_Y;				
				
				for (int i=0; i<(NUM_LINES);i++) //4 times
				{
					if (i == 0){
						plan_buffer_line(current_position[X_AXIS],mm_left_offset+15, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						st_synchronize();
						
						 plan_buffer_line(85,mm_left_offset+15, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						 st_synchronize();
						 
						 current_position[Z_AXIS]=0.2; //Move X and Z	
						 plan_buffer_line(85,mm_left_offset+15, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS]/2, active_extruder);//Retrack
						 	
						 
						 current_position[E_AXIS]+=4;
						 plan_buffer_line(85,mm_left_offset+15, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						 st_synchronize();
						 
						 current_position[E_AXIS]+=((115)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						 plan_buffer_line(85,mm_left_offset-(mm_each_extrusion*10), current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						 st_synchronize();
						 current_position[E_AXIS]+=((215-85)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						 plan_buffer_line(215,mm_left_offset-(mm_each_extrusion*10), current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						 st_synchronize();
						 current_position[E_AXIS]+=((115)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						 plan_buffer_line(215,mm_left_offset+15, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						 st_synchronize();
						 current_position[E_AXIS]+=((215-85)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						 plan_buffer_line(85,mm_left_offset+15, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						 st_synchronize();
						 current_position[E_AXIS]-=2;
						 plan_buffer_line(85,mm_left_offset+15, current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60 , active_extruder);
						 st_synchronize();
						 current_position[Z_AXIS]=0.2; //Move X and Z
						 plan_buffer_line(85,mm_left_offset+15, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						 st_synchronize();
						 	
						 plan_buffer_line(150, mm_left_offset-(mm_each_extrusion*i), current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);//Move X and Z
						 st_synchronize();			
						  //draw a one						 
						 /*plan_buffer_line(100-6, mm_left_offset-(mm_each_extrusion*i)-2, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						 plan_buffer_line(100-6, mm_left_offset-(mm_each_extrusion*i)+4, current_position[Z_AXIS], current_position[E_AXIS]+=1, max_feedrate[X_AXIS]/2 , active_extruder);
						 plan_buffer_line(100-6, mm_left_offset-(mm_each_extrusion*i)+4, current_position[Z_AXIS], current_position[E_AXIS]-=2, max_feedrate[X_AXIS]/2 , active_extruder);
						 st_synchronize();*/
					}
					/*if (i ==9){
						//draw a one						
						plan_buffer_line(100-13, mm_left_offset-(mm_each_extrusion*i)-2, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(100-13, mm_left_offset-(mm_each_extrusion*i)+4, current_position[Z_AXIS], current_position[E_AXIS]+=1, max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(100-13, mm_left_offset-(mm_each_extrusion*i)+4, current_position[Z_AXIS], current_position[E_AXIS]-=2, max_feedrate[X_AXIS]/2 , active_extruder);
						st_synchronize();
						//draw a zero
						plan_buffer_line(100-10, mm_left_offset-(mm_each_extrusion*i)-2, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(100-8, mm_left_offset-(mm_each_extrusion*i)-2, current_position[Z_AXIS], current_position[E_AXIS]+=1, max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(100-8, mm_left_offset-(mm_each_extrusion*i)+4, current_position[Z_AXIS], current_position[E_AXIS]+=1, max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(100-10, mm_left_offset-(mm_each_extrusion*i)+4, current_position[Z_AXIS], current_position[E_AXIS]+=1, max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(100-10, mm_left_offset-(mm_each_extrusion*i)-2, current_position[Z_AXIS], current_position[E_AXIS]+=1, max_feedrate[X_AXIS]/2 , active_extruder);
						plan_buffer_line(100-10, mm_left_offset-(mm_each_extrusion*i)-2, current_position[Z_AXIS], current_position[E_AXIS]-=2, max_feedrate[X_AXIS]/2 , active_extruder);
						st_synchronize();
					}*/
					
					current_position[E_AXIS]+=2; //Move the retracked space
					plan_buffer_line(150, mm_left_offset-(mm_each_extrusion*i), current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/ , active_extruder);
					
					st_synchronize();
									
					current_position[E_AXIS]+=((50)*0.33*current_position[Z_AXIS]*10/55.119)*1.75;
					plan_buffer_line(100, mm_left_offset-(mm_each_extrusion*i), current_position[Z_AXIS], current_position[E_AXIS], 1500/60, active_extruder);//Move Y and extrude
					
					current_position[E_AXIS]-=2;
					plan_buffer_line(100, mm_left_offset-(mm_each_extrusion*i), current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, active_extruder);//Move Y and extrude
					
					if(i != 9) plan_buffer_line(150, mm_left_offset-(mm_each_extrusion*(i+1)), current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//Retrack
					else plan_buffer_line(100, mm_left_offset-(mm_each_extrusion*i), current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//Retrack
					//plan_buffer_line(mm_left_offset+(mm_each_extrusion*i), 150, current_position[Z_AXIS]+5, current_position[E_AXIS], 1500/60, active_extruder);//Lift Z
					st_synchronize();
				}
				current_position[Y_AXIS]=mm_left_offset-(NUM_LINES-1)*mm_each_extrusion;
				current_position[X_AXIS]=100;				
				current_position[E_AXIS]-=2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//MORE Retrack
				current_position[Z_AXIS]+=0.2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//MORE Retrack
				
				//2)Extruder 2 prints with corrections
				current_position[X_AXIS]= x_home_pos(LEFT_EXTRUDER);
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//MORE Retrack			

				
				current_position[Z_AXIS]-=0.2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//MORE Retrack
				//2)Extruder 2 prints with corrections

				changeTool(1);
				
				//Raise for a layer of Z=0.2
				current_position[Z_AXIS]=0.5;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 1000/60 , active_extruder);

				//Purge & up
				current_position[E_AXIS]+=15;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60 , active_extruder);
				
				current_position[E_AXIS]-=4;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, active_extruder);//Retrack
				st_synchronize();
				
				plan_buffer_line(current_position[X_AXIS], mm_left_offset+10, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);//Move Y and Z
				st_synchronize();
				
				for (int i=0; i<(NUM_LINES);i++) //4 times
				{
					if (i == 0) {
						
						plan_buffer_line(current_position[X_AXIS],mm_left_offset+10, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						st_synchronize();
						
						plan_buffer_line(210,mm_left_offset+10, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						st_synchronize();
						
						current_position[Z_AXIS]=0.2;
						plan_buffer_line(210,mm_left_offset+10, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);
						st_synchronize();
						
						current_position[E_AXIS]+=4;
						plan_buffer_line(210,mm_left_offset+10, current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, active_extruder);//Retrack
						st_synchronize();
						
						current_position[E_AXIS]+=((210-90)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						plan_buffer_line(90,mm_left_offset+10, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						current_position[E_AXIS]+=((105)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						plan_buffer_line(90,mm_left_offset-(mm_each_extrusion*10)+5, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						current_position[E_AXIS]+=((210-90)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						plan_buffer_line(210,mm_left_offset-(mm_each_extrusion*10)+5, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						current_position[E_AXIS]+=((105)*0.33*current_position[Z_AXIS]*10/55.119)*2;
						plan_buffer_line(210,mm_left_offset+10, current_position[Z_AXIS], current_position[E_AXIS], 1500/60 , active_extruder);
						st_synchronize();
						current_position[E_AXIS]-=2;
						plan_buffer_line(210,mm_left_offset+10, current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60 , active_extruder);
						st_synchronize();
						current_position[Z_AXIS]=0.2;
						plan_buffer_line(210,mm_left_offset+10, current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS]/2 , active_extruder);//Move X
						st_synchronize();
						plan_buffer_line(151, mm_left_offset-(mm_second_extruder[i]+(mm_each_extrusion*(i))), current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/ , active_extruder);//Move the retracked space
						st_synchronize();						
						
					}	
					
					current_position[E_AXIS]+=2;
					plan_buffer_line(151, mm_left_offset-(mm_second_extruder[i]+(mm_each_extrusion*(i))), current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/ , active_extruder);//Move the retracked space
					Serial.println("We are restoring retrack");
					st_synchronize();									
					
					//((50)*0.33*current_position[Z_AXIS]*10/55.119)*1.75;
					current_position[E_AXIS]+=((50)*0.33*current_position[Z_AXIS]*10/55.119)*1.75;
					plan_buffer_line(200, mm_left_offset-(mm_second_extruder[i]+(mm_each_extrusion*(i))), current_position[Z_AXIS], current_position[E_AXIS], 1500/60, active_extruder);//Move Y and extrude
					
					current_position[E_AXIS]-=2;
					plan_buffer_line(200, mm_left_offset-(mm_second_extruder[i]+(mm_each_extrusion*(i))), current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, active_extruder);//Move Y and extrude
					
					if (i != 9) plan_buffer_line(151, mm_left_offset-(mm_second_extruder[i+1]+(mm_each_extrusion*(i+1))), current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//Retrack
					else plan_buffer_line(200, mm_left_offset-(mm_second_extruder[i]+(mm_each_extrusion*(i))), current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//Retrack
					st_synchronize();
				}
				current_position[Y_AXIS]=mm_left_offset-(mm_second_extruder[NUM_LINES-1]+(mm_each_extrusion*(NUM_LINES-1)));
				current_position[X_AXIS]=200;
				current_position[E_AXIS]-=2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//MORE Retrack
				current_position[Z_AXIS]+=2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//MORE Retrack
							

				changeTool(0);	
				
				current_position[Z_AXIS]-=2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 100/*50*/, active_extruder);//MORE Retrack
				enquecommand_P((PSTR("G28 X0 Y0")));
				st_synchronize();
				
				//Go to Calibration select screen
				genie.WriteObject(GENIE_OBJ_FORM,FORM_Y_CALIB_SELECT,0);
				
				break;
			}
			#endif //EXTRUDER_CALIBRATION_WIZARD


			//Rapduch for RepRapBCN Sigma
			#ifdef EXTRUDER_CALIBRATION_WIZARD
			case 43://G43 --> Z LEFT Extruder calibration
			{
				Serial.println("Starting Z Calibration Wizard");
				//Raise to correct
				feedrate = homing_feedrate[Z_AXIS];
				current_position[Z_AXIS]=0;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60 , active_extruder);				
				st_synchronize();
				
				feedrate = homing_feedrate[Z_AXIS];
				current_position[Z_AXIS]=1;
				
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60 , active_extruder);
				feedrate = homing_feedrate[X_AXIS];
				current_position[X_AXIS]=X_MAX_POS/2;
				current_position[Y_AXIS]=Y_MAX_POS/2;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60 , active_extruder);
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
				
				//Now we are in position to do the calibration MANUALLY with the TOUCHSCREEN
				st_synchronize();
				
				//Go to Z Calibration select screen if first time!
				if (active_extruder==LEFT_EXTRUDER) {
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_Z_EXTRUDER1,0);
					}else{
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_Z_EXTRUDER2,0);
				}	
				break;
			}
			#endif //EXTRUDER_CALIBRATION_WIZARD
			
			
			#ifdef ENABLE_AUTO_BED_LEVELING

			case 33: // G33 Calibration Wizard by Eric Pallarés & Jordi Calduch for RepRapBCN
			{
				//WARNING: T0 (LEFT_EXTRUDER) MUST BE SELECTED!
				if (active_extruder==RIGHT_EXTRUDER){
					Serial.println("Error: Left Extruder MUST BE ACTIVE");
					break; //An error message should show up on screen
				}
				
				//Rapduch
				#ifdef Z_SIGMA_AUTOLEVEL
				
				#if Z_MIN_PIN == -1
				#error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature!!! Z_MIN_PIN must point to a valid hardware pin."
				#endif

				// Prevent user from running a G29 without first homing in X and Y
				if (! (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) )
				{
					LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
					SERIAL_ECHO_START;
					SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
					//break; // abort G29, since we don't know where we are
				}
				
				//We have to save the active extruder.
				int saved_active_extruder = active_extruder;
				
				//Starting Calibration WIZARD
				plan_bed_level_matrix.set_to_identity();
				vector_3 uncorrected_position = plan_get_position();
				//uncorrected_position.debug("position durring G29");
				current_position[X_AXIS] = uncorrected_position.x;
				current_position[Y_AXIS] = uncorrected_position.y;
				current_position[Z_AXIS] = uncorrected_position.z;
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
				
				//MOVING THE EXTRUDERS TO AVOID HITTING THE CASE WHEN PROBING-------------------------
				current_position[X_AXIS]+=15;
				feedrate=homing_feedrate[X_AXIS];
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, LEFT_EXTRUDER);
				st_synchronize();
				//current_position[X_AXIS] = x_home_pos(RIGHT_EXTRUDER);
				
				active_extruder=RIGHT_EXTRUDER;
				axis_is_at_home(X_AXIS); //Redoes the Max Min calculus for the right extruder
				Serial.println(current_position[X_AXIS]);
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
				current_position[X_AXIS]-=15;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
				
				//Now we can proceed to probe the first 3 points with the left extruder
				active_extruder=LEFT_EXTRUDER;
				axis_is_at_home(X_AXIS);
				current_position[X_AXIS]+=15;
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]); // We are now at position
				st_synchronize();
				
				//STARTING THE ACTUAL PROBE
				setup_for_endstop_move();
				
				feedrate = homing_feedrate[Z_AXIS];
				
				// Probe at 3 arbitrary points
				// probe left extruder
				
				Serial.print("Zvalue after home:");
				Serial.println(current_position[Z_AXIS]);
				
				float z_at_pt_1 = probe_pt(X_SIGMA_PROBE_1_LEFT_EXTR,Y_SIGMA_PROBE_1_LEFT_EXTR, Z_RAISE_BEFORE_PROBING);
				float z_at_pt_2 = probe_pt(X_SIGMA_PROBE_2_LEFT_EXTR,Y_SIGMA_PROBE_2_LEFT_EXTR, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
				float z_at_pt_3 = probe_pt(X_SIGMA_PROBE_3_LEFT_EXTR,Y_SIGMA_PROBE_3_LEFT_EXTR, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
				
				feedrate=homing_feedrate[Z_AXIS];
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+5, current_position[E_AXIS], feedrate/60, LEFT_EXTRUDER);
				feedrate = XY_TRAVEL_SPEED;
				current_position[X_AXIS]=x_home_pos(active_extruder)+15;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+5, current_position[E_AXIS], feedrate/60, LEFT_EXTRUDER);
				feedrate=homing_feedrate[Z_AXIS];
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, LEFT_EXTRUDER);
				st_synchronize();
				
				
				//Now the right extruder joins the party!
				active_extruder=RIGHT_EXTRUDER;
				Serial.print("Zvalue at start:");
				Serial.println(current_position[Z_AXIS]);
				axis_is_at_home(X_AXIS); //Redoes the Max Min calculus for the right extruder
				current_position[X_AXIS]-=15;
				Serial.print("Zvalue before:");
				Serial.println(current_position[Z_AXIS]);				
				Serial.print("Zvalue after:");
				Serial.println(current_position[Z_AXIS]);
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
				

				//Probe at 3 arbitrary points
				//probe left extruder
				float z2_at_pt_1 = probe_pt(X_SIGMA_PROBE_1_RIGHT_EXTR,Y_SIGMA_PROBE_1_RIGHT_EXTR, Z_RAISE_BEFORE_PROBING);
				float z2_at_pt_2 = probe_pt(X_SIGMA_PROBE_2_RIGHT_EXTR,Y_SIGMA_PROBE_2_RIGHT_EXTR, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
				float z2_at_pt_3 = probe_pt(X_SIGMA_PROBE_3_RIGHT_EXTR,Y_SIGMA_PROBE_3_RIGHT_EXTR, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
				
				
				feedrate=homing_feedrate[Z_AXIS];
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+5, current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
				feedrate = XY_TRAVEL_SPEED;
				current_position[X_AXIS]=x_home_pos(active_extruder)-15;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+5, current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
				feedrate=homing_feedrate[Z_AXIS];
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
				current_position[Y_AXIS]=Y_MAX_POS/2;
				feedrate = XY_TRAVEL_SPEED;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
				st_synchronize();
				
				
				clean_up_after_endstop_move();
				
				
				//Update zOffset. We have to take into account the 2 different probe offsets
				//NOT NEEDED
				//Calculate medians
				
				float z_final_probe_1 = (z_at_pt_1+z2_at_pt_1)/2; //Upper left, upper right
				float z_final_probe_2 = (z_at_pt_2+z2_at_pt_3)/2; //Lower left, lower right
				float z_final_probe_3 = (z_at_pt_3+z2_at_pt_2)/2; //lower right, lower left
				
				Serial.print("Probe 1: ");
				Serial.println(z_final_probe_1);
				Serial.print("Probe 2: ");
				Serial.println(z_final_probe_2);
				Serial.print("Probe 3: ");
				Serial.println(z_final_probe_3);
				
				//Rapduch: We negated the Z points passed on this functions because the actual correction was inverted
				//set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);
				set_bed_level_equation_3pts(-z_final_probe_1, -z_final_probe_2, -z_final_probe_3);
				//This puts the current_posZ at z probe offset!!!
				
				// The following code correct the Z height difference from z-probe position and hotend tip position.
				// The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
				// When the bed is uneven, this height must be corrected.
				real_z = float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)
				Serial.print("Real_Z: ");
				Serial.println(real_z);
				
				x_tmp = current_position[X_AXIS] + X_SIGMA_SECOND_PROBE_OFFSET_FROM_EXTRUDER;
				y_tmp = current_position[Y_AXIS] + Y_SIGMA_SECOND_PROBE_OFFSET_FROM_EXTRUDER;
				z_tmp = current_position[Z_AXIS];

				apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);         //Apply the correction, we sent the z_offset as current position
				current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
				Serial.print("Current Z after setting: ");
				Serial.println(current_position[Z_AXIS]);
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

				//enquecommand_P(PSTR("G35")); //Home again without wiping autolevel data!!!
				
				//Put both extruders at position
				active_extruder=RIGHT_EXTRUDER;
				current_position[X_AXIS]=x_home_pos(active_extruder)-15;
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
				feedrate = homing_feedrate[X_AXIS];
				current_position[X_AXIS]=x_home_pos(RIGHT_EXTRUDER);
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS], feedrate/60, active_extruder);
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
				axis_is_at_home(X_AXIS); //Redoes the Max Min calculus for the extruder
				
				//changeToolSigma(LEFT_EXTRUDER);
				active_extruder=LEFT_EXTRUDER;
				feedrate = homing_feedrate[X_AXIS];
				current_position[X_AXIS]=x_home_pos(LEFT_EXTRUDER)+15;
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
				current_position[X_AXIS]=x_home_pos(LEFT_EXTRUDER);
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				
				//Home Z
				feedrate = homing_feedrate[Z_AXIS];
				current_position[Z_AXIS]=0;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				
				//current_position[Z_AXIS] += zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)

				axis_is_at_home(X_AXIS); //Redoes the Max Min calculus for the extruder
				
				//Restore the previous active extruder:
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
				
				
				if(saved_active_extruder==LEFT_EXTRUDER)
				{
					//changeToolSigma(LEFT_EXTRUDER); //Get again the same tool
					Serial.print("Extruder active: ");
					Serial.println(saved_active_extruder);
					}else{
					
				}
				
				
				#else
				
				//First do AUTOHOME
				//enquecommand_P((PSTR("G28"))); //This is not working!!!!!!!!!!! cant enqueue and plan buffer!
				
				///////////////////////////
				//Autohome done - Now calibration calculus
				///////////////////////////

				st_synchronize();
				// make sure the bed_level_rotation_matrix is identity or the planner will get it incorrectly
				//vector_3 corrected_position = plan_get_position_mm();
				//corrected_position.debug("position before G29");
				plan_bed_level_matrix.set_to_identity();
				vector_3 uncorrected_position = plan_get_position();
				//uncorrected_position.debug("position durring G29");
				current_position[X_AXIS] = uncorrected_position.x;
				current_position[Y_AXIS] = uncorrected_position.y;
				current_position[Z_AXIS] = uncorrected_position.z;
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
				setup_for_endstop_move();
				
				feedrate = homing_feedrate[Z_AXIS];
				
				// Probe at 3 arbitrary points
				// probe 1
				float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING);
				// probe 2
				float z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
				// probe 3
				float z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

				clean_up_after_endstop_move();

				
				plan_bed_level_matrix.set_to_identity();

				vector_3 pt1 = vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, z_at_pt_1);
				vector_3 pt2 = vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, z_at_pt_2);
				vector_3 pt3 = vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, z_at_pt_3);

				vector_3 from_2_to_1 = (pt1 - pt2);
				vector_3 from_2_to_3 = (pt3 - pt2);
				vector_3 planeNormal = vector_3::cross(from_2_to_1, from_2_to_3);
				planeNormal = vector_3(planeNormal.x, planeNormal.y, abs(planeNormal.z));
				
				//Es calcula el pla a partir del pt2 i per tant és l'origen (0,0,0)
				
				//Posicions relatives dels cargols de regulació respecte l'origen
				float cargol_1_x = CARGOL_1_X;
				float cargol_1_y = CARGOL_1_Y;
				
				float cargol_2_x = CARGOL_2_X;
				float cargol_2_y = CARGOL_2_Y;
				
				float cargol_3_x = CARGOL_3_X;
				float cargol_3_y = CARGOL_3_Y;
				
				//Càlcul dels vectors normalitzats de l'origen fins al cargol de regulació

				vector_3 d1 = vector_3 (cargol_1_x, cargol_1_y, 0);
				vector_3 d2 = vector_3(cargol_2_x, cargol_2_y, 0);
				vector_3 d3 = vector_3(cargol_3_x, cargol_3_y, 0);
				
				//Càlcul de l'alçada Z dels cargols de regulació
				float z1=(-planeNormal.x*d1.x-planeNormal.y*d1.y)/planeNormal.z;
				float z2=(-planeNormal.x*d2.x-planeNormal.y*d2.y)/planeNormal.z;
				float z3=(-planeNormal.x*d3.x-planeNormal.y*d3.y)/planeNormal.z;

				//Posició relativa del centre de la plataforma respecte l'origen
				float centre_x = (X_MAX_POS/2)-ABL_PROBE_PT_2_X;
				float centre_y = (Y_MAX_POS/2)-ABL_PROBE_PT_2_Y;
				vector_3 centre = vector_3 (centre_x, centre_y, 0);
				
				//Càlcul de l'alçada Z del centre de la plataforma
				float zc=(-planeNormal.x*centre.x-planeNormal.y*centre.y)/planeNormal.z;
				
				//Càlcul alçades relatives cargols regulació respecte al centre (objectiu de regulació)
				float dz1 = zc-z1;
				float dz2 = zc-z2;
				float dz3 = zc-z3;
				
				//Voltes cargols
				
				float pas_M5 = PAS_M5;
				
				int sentit1 = sentit (dz1);
				int sentit2 = sentit (dz2);
				int sentit3 = sentit (dz3);
				
				float voltes1= voltes (dz1);
				float voltes2= voltes (dz2);
				float voltes3= voltes (dz3);
				
				//Aproximació a 1/8 de volta
				int aprox1 = aprox (voltes1);
				int numvoltes1 = aprox1/8;   // Voltes completes
				int vuitens1 = aprox1 % 8;  // Vuitens
				
				int aprox2 = aprox (voltes2);
				int numvoltes2 = aprox2/8;   // Voltes completes
				int vuitens2 = aprox2 % 8;  // Vuitens
				
				int aprox3 = aprox (voltes3);
				int numvoltes3 = aprox3/8;   // Voltes completes
				int vuitens3 = aprox3 % 8;  // Vuitens
				
				if (aprox1==0 && aprox2==0 && aprox3==0)
				{
					#ifdef SIGMA_TOUCH_SCREEN
					
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
					#endif
					
					SERIAL_PROTOCOL(" Platform is Calibrated! ");
					}else{
					
					#ifdef SIGMA_TOUCH_SCREEN
					char buffer[256];
					sprintf(buffer, " %d / 8",vuitens1); //Printing how to calibrate on screen
					//genie.WriteStr(STRING_BED_SCREW1,buffer);
					
					if (sentit1>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,3,1);} //The direction is inverted in Sigma's bed screws
					else{genie.WriteObject(GENIE_OBJ_USERIMAGES,3,0);}
					
					//sprintf(buffer, " %d / 8",vuitens2);
					//genie.WriteStr(STRING_SCREW2,buffer);
					
					//sprintf(buffer, " %d / 8",vuitens3);
					//genie.WriteStr(STRING_SCREW3,buffer);
					
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_BAD,1);
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW1,0);
					
					
					#endif
					
					SERIAL_PROTOCOLPGM(" zc: ");
					SERIAL_PROTOCOL(zc);
					SERIAL_PROTOCOLPGM("\n");
					SERIAL_PROTOCOLPGM(" dz1: ");
					SERIAL_PROTOCOL(dz1);
					SERIAL_PROTOCOLPGM(" dz2: ");
					SERIAL_PROTOCOL(dz2);
					SERIAL_PROTOCOLPGM(" dz3: ");
					SERIAL_PROTOCOL(dz3);
					SERIAL_PROTOCOLPGM("\n");

					SERIAL_PROTOCOLPGM(" Voltes cargol 1: ");
					if (numvoltes1 > 0)
					{
						SERIAL_PROTOCOL(numvoltes1);
						SERIAL_PROTOCOLPGM(" ");
					}
					SERIAL_PROTOCOL(vuitens1);
					SERIAL_PROTOCOLPGM("/8");
					if (sentit1 > 0)
					{
						SERIAL_PROTOCOLPGM(" horari\n ");
					}
					else
					{
						SERIAL_PROTOCOLPGM(" antihorari\n: ");
					}

					SERIAL_PROTOCOLPGM(" Voltes cargol 2: ");
					if (numvoltes2 > 0)
					{
						SERIAL_PROTOCOL(numvoltes2);
						SERIAL_PROTOCOLPGM(" ");
					}
					SERIAL_PROTOCOL(vuitens2);
					SERIAL_PROTOCOLPGM("/8");
					if (sentit2 > 0)
					{
						SERIAL_PROTOCOLPGM(" horari\n ");
					}
					else
					{
						SERIAL_PROTOCOLPGM(" antihorari\n: ");
					}
					
					SERIAL_PROTOCOLPGM(" Voltes cargol 3: ");
					if (numvoltes3 > 0)
					{
						SERIAL_PROTOCOL(numvoltes3);
						SERIAL_PROTOCOLPGM(" ");
					}
					SERIAL_PROTOCOL(vuitens3);
					SERIAL_PROTOCOLPGM("/8");
					if (sentit3 > 0)
					{
						SERIAL_PROTOCOLPGM(" horari\n ");
					}
					else
					{
						SERIAL_PROTOCOLPGM(" antihorari\n: ");
					}
					SERIAL_PROTOCOLPGM("\n");
				}
				
				#endif //Z_SIGMA_AUTOLEVEL
				
				break; //G33 ends
			}
			
			//Rapduch
			#ifdef SIGMA_BED_AUTOCALIB
			case 34: // G34 Performs a BED calibration Wizard with TouchScreen Integration
			{
				
				
				//WARNING: T0 (LEFT_EXTRUDER) MUST BE SELECTED!
				if (active_extruder==RIGHT_EXTRUDER){
					Serial.println("Error: Left Extruder MUST BE ACTIVE");
					break; //An error message should show up on screen
				}
				
				if (flag_full_calib){
					setTargetHotend0(EXTRUDER_LEFT_CLEAN_TEMP);
					setTargetHotend1(EXTRUDER_LEFT_CLEAN_TEMP);
					setTargetBed(max(bed_temp_l,bed_temp_r));
					Serial.print("Temperatura del llit: ");Serial.println(max(bed_temp_l,bed_temp_r));
				}
				
				#if Z_MIN_PIN == -1
				#error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature!!! Z_MIN_PIN must point to a valid hardware pin."
				#endif

				// Prevent user from running a G29 without first homing in X and Y
				if (! (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) )
				{
					LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
					SERIAL_ECHO_START;
					SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
					//break; // abort G29, since we don't know where we are
				}
				
				//We have to save the active extruder.
				int saved_active_extruder = active_extruder;
				
				//Starting Calibration WIZARD
				plan_bed_level_matrix.set_to_identity();
				vector_3 uncorrected_position = plan_get_position();
				//uncorrected_position.debug("position durring G29");
				current_position[X_AXIS] = uncorrected_position.x;
				current_position[Y_AXIS] = uncorrected_position.y;
				current_position[Z_AXIS] = uncorrected_position.z;
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
				
				//MOVING THE EXTRUDERS TO AVOID HITTING THE CASE WHEN PROBING-------------------------
				current_position[X_AXIS] += 15;//x_home_pos(LEFT_EXTRUDER)+15;
				feedrate=homing_feedrate[X_AXIS];
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, LEFT_EXTRUDER);
				///////st_synchronize();
				//current_position[X_AXIS] = x_home_pos(RIGHT_EXTRUDER);			
				
				active_extruder=RIGHT_EXTRUDER;
				axis_is_at_home(X_AXIS); //Redoes the Max Min calculus for the right extruder
				Serial.println(current_position[X_AXIS]);
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
				current_position[X_AXIS]-=15;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
				
				//*********************************************************************		
				//Now we can proceed to probe the first 3 points with the left extruder
				active_extruder=LEFT_EXTRUDER;
				axis_is_at_home(X_AXIS);
				current_position[X_AXIS]+=15;
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]); // We are now at position
				st_synchronize();
				
				
				//STARTING THE ACTUAL PROBE
				setup_for_endstop_move();
				
				feedrate = homing_feedrate[Z_AXIS];
				
				// Probe at 3 arbitrary points
				// probe left extruder
				
				Serial.print("Zvalue after home:");
				Serial.println(current_position[Z_AXIS]);
				
				float z_at_pt_1 = probe_pt(X_SIGMA_PROBE_1_LEFT_EXTR,Y_SIGMA_PROBE_1_LEFT_EXTR, Z_RAISE_BEFORE_PROBING);
				float z_at_pt_2 = probe_pt(X_SIGMA_PROBE_2_LEFT_EXTR,Y_SIGMA_PROBE_2_LEFT_EXTR, current_position[Z_AXIS] + (Z_RAISE_BETWEEN_PROBINGS/2));
				float z_at_pt_3 = probe_pt(X_SIGMA_PROBE_3_LEFT_EXTR,Y_SIGMA_PROBE_3_LEFT_EXTR, current_position[Z_AXIS] + (Z_RAISE_BETWEEN_PROBINGS/2));
				
				//feedrate=homing_feedrate[X_AXIS];
				feedrate = XY_TRAVEL_SPEED;
				current_position[X_AXIS]=x_home_pos(active_extruder)+15; current_position[Z_AXIS]+= 3;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, LEFT_EXTRUDER);
				/*feedrate=homing_feedrate[Z_AXIS];
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, LEFT_EXTRUDER);*/
				st_synchronize();
			
				
				//Now the right extruder joins the party!
				
				active_extruder=RIGHT_EXTRUDER;				
				axis_is_at_home(X_AXIS); //Redoes the Max Min calculus for the right extruder
				current_position[X_AXIS]-=15;				
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
				
				
				//Probe at 3 arbitrary points
				//probe left extruder
				float z2_at_pt_3 = probe_pt(X_SIGMA_PROBE_3_RIGHT_EXTR,Y_SIGMA_PROBE_3_RIGHT_EXTR, Z_RAISE_BEFORE_PROBING);
				float z2_at_pt_2 = probe_pt(X_SIGMA_PROBE_2_RIGHT_EXTR,Y_SIGMA_PROBE_2_RIGHT_EXTR, current_position[Z_AXIS] + (Z_RAISE_BETWEEN_PROBINGS/2));
				float z2_at_pt_1 = probe_pt(X_SIGMA_PROBE_1_RIGHT_EXTR,Y_SIGMA_PROBE_1_RIGHT_EXTR, current_position[Z_AXIS] + (Z_RAISE_BETWEEN_PROBINGS/2));
				
				
				
				feedrate=homing_feedrate[Z_AXIS];
				//current_position[Z_AXIS] += 5;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+5, current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
				//feedrate=homing_feedrate[X_AXIS];
				feedrate = XY_TRAVEL_SPEED;
				current_position[X_AXIS]=x_home_pos(active_extruder)-15;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+5, current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
				feedrate=homing_feedrate[Z_AXIS];
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
				current_position[Y_AXIS]=Y_MAX_POS/2;
				feedrate = XY_TRAVEL_SPEED;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, RIGHT_EXTRUDER);
				st_synchronize();
				
				clean_up_after_endstop_move();
				
				
				//Update zOffset. We have to take into account the 2 different probe offsets
				//NOT NEEDED because we have to check the bed from the same position. Theorically the offsets between probes is inexistent
				//Calculate medians
				float z_final_probe_1 = (z_at_pt_1+(z2_at_pt_1-(((z2_at_pt_2-z_at_pt_3)+(z2_at_pt_3-z_at_pt_2))/2)))/2; //Upper left, upper right
				float z_final_probe_2 = z_at_pt_2;//(z_at_pt_2+z2_at_pt_3)/2; //Lower left, lower left
				float z_final_probe_3 = z_at_pt_3;//(z_at_pt_3+z2_at_pt_2)/2; //lower right, lower right
				
				Serial.print("Probe 1: ");
				Serial.println(z_final_probe_1);
				Serial.print("Probe 2: ");
				Serial.println(z_final_probe_2);
				Serial.print("Probe 3: ");
				Serial.println(z_final_probe_3);
				
				//Eric's Part!!!
				//plan_bed_level_matrix.set_to_identity();
				float probe_x1 = (X_SIGMA_PROBE_1_LEFT_EXTR+((X_SIGMA_PROBE_1_RIGHT_EXTR-X_SIGMA_PROBE_1_LEFT_EXTR)/2));
				Serial.print("The p1 is on X:");
				Serial.println(probe_x1);
				float probe_y1 = Y_SIGMA_PROBE_1_LEFT_EXTR;
				vector_3 pt1 = vector_3(probe_x1, probe_y1, z_final_probe_1);

				vector_3 pt2 = vector_3(X_SIGMA_PROBE_2_LEFT_EXTR, Y_SIGMA_PROBE_2_LEFT_EXTR, z_final_probe_2);
				vector_3 pt3 = vector_3(X_SIGMA_PROBE_3_LEFT_EXTR, Y_SIGMA_PROBE_3_LEFT_EXTR, z_final_probe_3);
				
				vector_3 from_2_to_1 = (pt1 - pt2);
				vector_3 from_2_to_3 = (pt3 - pt2);
				vector_3 planeNormal = vector_3::cross(from_2_to_1, from_2_to_3);
				planeNormal = vector_3(planeNormal.x, planeNormal.y, abs(planeNormal.z));
				
				//Es calcula el pla a partir del pt2 i per tant és l'origen (0,0,0)
				
				//Posicions relatives dels cargols de regulació respecte l'origen
				float cargol_1_x = CARGOL_1_X-X_SIGMA_PROBE_2_LEFT_EXTR;
				float cargol_1_y = CARGOL_1_Y-Y_SIGMA_PROBE_2_LEFT_EXTR;
				
				float cargol_2_x = CARGOL_2_X-X_SIGMA_PROBE_2_LEFT_EXTR;
				float cargol_2_y = CARGOL_2_Y-Y_SIGMA_PROBE_2_LEFT_EXTR;
				
				float cargol_3_x = CARGOL_3_X-X_SIGMA_PROBE_2_LEFT_EXTR;
				float cargol_3_y = CARGOL_3_Y-Y_SIGMA_PROBE_2_LEFT_EXTR;
				
				//Càlcul dels vectors normalitzats de l'origen fins al cargol de regulació

				vector_3 d1 = vector_3 (cargol_1_x, cargol_1_y, 0);
				vector_3 d2 = vector_3(cargol_2_x, cargol_2_y, 0);
				vector_3 d3 = vector_3(cargol_3_x, cargol_3_y, 0);
				
				//Càlcul de l'alçada Z dels cargols de regulació
				float z1=(-planeNormal.x*d1.x-planeNormal.y*d1.y)/planeNormal.z;
				float z2=(-planeNormal.x*d2.x-planeNormal.y*d2.y)/planeNormal.z;
				float z3=(-planeNormal.x*d3.x-planeNormal.y*d3.y)/planeNormal.z;

				//Posició relativa del centre de la plataforma respecte l'origen
				float centre_x = cargol_1_x;//(X_MAX_POS/2)-X_SIGMA_PROBE_2_LEFT_EXTR;
				float centre_y = cargol_1_y;//(Y_MAX_POS/2)-Y_SIGMA_PROBE_2_LEFT_EXTR;
				vector_3 centre = vector_3 (centre_x, centre_y, 0);
				
				//Càlcul de l'alçada Z del centre de la plataforma
				float zc=(-planeNormal.x*centre.x-planeNormal.y*centre.y)/planeNormal.z;
				
				//Càlcul alçades relatives cargols regulació respecte al centre (objectiu de regulació)
				float dz1 = zc-z1;
				float dz2 = zc-z2;
				float dz3 = zc-z3;
				
				//Voltes cargols
				
				float pas_M5 = PAS_M5;
				
				sentit1 = sentit (dz1);
				sentit2 = sentit (dz2);
				sentit3 = sentit (dz3);
				
				float voltes1= voltes (dz1);
				float voltes2= voltes (dz2);
				float voltes3= voltes (dz3);
				
				//Aproximació a 1/8 de volta
				int aprox1 = aprox (voltes1);
				int numvoltes1 = aprox1/8;   // Voltes completes
				vuitens1 = aprox1 % 8;  // Vuitens
				
				int aprox2 = aprox (voltes2);
				int numvoltes2 = aprox2/8;   // Voltes completes
				vuitens2 = aprox2 % 8;  // Vuitens
				
				int aprox3 = aprox (voltes3);
				int numvoltes3 = aprox3/8;   // Voltes completes
				vuitens3 = aprox3 % 8;  // Vuitens
				
				
				if (numvoltes1!=0){
					vuitens1+=(numvoltes1*8);
				}
				
				if (numvoltes2!=0){
					vuitens2+=(numvoltes2*8);
				}
				
				if (numvoltes3!=0){
					vuitens3+=(numvoltes3*8);
				}
				
				if (vuitens1<0) vuitens1=vuitens1*(-1);
				if (vuitens2<0) vuitens2=vuitens2*(-1);
				if (vuitens3<0) vuitens3=vuitens3*(-1);
				
				//limit the maxim turns to 1
				if (vuitens1>8) vuitens1=8;
				if (vuitens2>8) vuitens2=8;
				if (vuitens3>8) vuitens3=8;
				
				Serial.print("Voltes1:  ");
				Serial.println(voltes1);
				Serial.print("Aprox1:  ");
				Serial.println(aprox1);
				Serial.print("Vuitens1:  ");
				Serial.println(vuitens1);
				Serial.println("");
				
				Serial.print("Voltes2:  ");
				Serial.println(voltes2);
				Serial.print("Aprox2:  ");
				Serial.println(aprox2);
				Serial.print("Vuitens2:  ");
				Serial.println(vuitens2);
				Serial.println("");
				
				Serial.print("Voltes3:  ");
				Serial.println(voltes3);
				Serial.print("Aprox3:  ");
				Serial.println(aprox3);
				Serial.print("Vuitens3:  ");
				Serial.println(vuitens3);
				Serial.println("");
				
				if (aprox1==0 && aprox2==0 && aprox3==0) //If the calibration it's ok
				{
					processing = false;
					if (flag_full_calib){						
						
						
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
						
						home_axis_from_code();
						//changeTool(LEFT_EXTRUDER);
						
						while (degHotend(LEFT_EXTRUDER)<(degTargetHotend(LEFT_EXTRUDER)-5) && degHotend(RIGHT_EXTRUDER)<(degTargetHotend(RIGHT_EXTRUDER)-5)){ //Waiting to heat the extruder
							
							manage_heater();
						}
						
						//MOVE EXTRUDERS
						current_position[Z_AXIS] = 60;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS]*2/60, LEFT_EXTRUDER);//move bed
						st_synchronize();
						current_position[X_AXIS] = 155; current_position[Y_AXIS] = 0;
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS]/3, LEFT_EXTRUDER);//move first extruder
						
						blocking_x = true; blocking_y = true;
						
						genie.WriteObject(GENIE_OBJ_USERBUTTON,USERBUTTON_CLEAN_DONE,1);
						genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_THERMOMETHER,1);
						genie.WriteStr(STRING_CLEAN_INSTRUCTIONS,"Clean the left nozzle \n and press GO to move on to \n the next EXTRUDER");
						flag_continue_calib = true;							
					}
					else{
						
						#ifdef SIGMA_TOUCH_SCREEN
							genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_GOOD,0);
						#endif
						SERIAL_PROTOCOL(" Platform is Calibrated! ");
						current_position[Z_AXIS] += 35;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
					}
				}else{	
					processing = false;
					bed_calibration_times++;
								
					#ifdef SIGMA_TOUCH_SCREEN	
					if (bed_calibration_times >= 2) genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SKIP_BED,1);
					else genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_SKIP_BED,0);		
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INFO_TURN_SCREWS,0);					
					/*		 //char buffer[256];
					//
					//
					//if (vuitens1!= 0){
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW1,0);
					//sprintf(buffer, " %d / 8",vuitens1); //Printing how to calibrate on screen
					//genie.WriteStr(STRING_BED_SCREW1,buffer);
					//if (vuitens2==0) genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW2,2);
					//if (sentit1>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW1,1);} //The direction is inverted in Sigma's bed screws
					//else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW1,0);}
					//}
					//
					//else if (vuitens2!= 0){
					//Serial.println("Jump over screw1");
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW2,0);
					//sprintf(buffer, " %d / 8",vuitens2); //Printing how to calibrate on screen
					//genie.WriteStr(STRING_BED_SCREW2,buffer);
					//if (vuitens3==0) genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_BED_CALIB_SW3,2);
					//if (sentit1>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,1);} //The direction is inverted in Sigma's bed screws
					//else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,0);}
					//}
					//
					//else if (vuitens3!= 0){
					//Serial.println("Jump over screw1 and screw2");
					//
					//sprintf(buffer, " %d / 8",vuitens3); //Printing how to calibrate on screen
					//genie.WriteStr(STRING_BED_SCREW3,buffer);
					//if (sentit1>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,1);} //The direction is inverted in Sigma's bed screws
					//else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,0);}
					//
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW3,0);
					//}
					
					//sprintf(buffer, " %d / 8",vuitens2);
					//genie.WriteStr(STRING_SCREW2,buffer);
					
					//sprintf(buffer, " %d / 8",vuitens3);
					//genie.WriteStr(STRING_SCREW3,buffer);
					
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_DONE_BAD,1);*/
					
					
					#endif
					
					SERIAL_PROTOCOLPGM(" zc: ");
					SERIAL_PROTOCOL(zc);
					SERIAL_PROTOCOLPGM("\n");
					SERIAL_PROTOCOLPGM(" dz1: ");
					SERIAL_PROTOCOL(dz1);
					SERIAL_PROTOCOLPGM(" dz2: ");
					SERIAL_PROTOCOL(dz2);
					SERIAL_PROTOCOLPGM(" dz3: ");
					SERIAL_PROTOCOL(dz3);
					SERIAL_PROTOCOLPGM("\n");

					SERIAL_PROTOCOLPGM(" Voltes cargol 1: ");
					if (numvoltes1 != 0)
					{
					SERIAL_PROTOCOL(numvoltes1);
					SERIAL_PROTOCOLPGM("voltes ");
					}
					SERIAL_PROTOCOL(vuitens1);
					SERIAL_PROTOCOLPGM("/8");
					if (sentit1 > 0)
					{
					SERIAL_PROTOCOLPGM(" horari\n ");
					}
					else
					{
					SERIAL_PROTOCOLPGM(" antihorari\n: ");
					}

					SERIAL_PROTOCOLPGM(" Voltes cargol 2: ");
					if (numvoltes2 != 0)
					{
					SERIAL_PROTOCOL(numvoltes2);
					SERIAL_PROTOCOLPGM("voltes ");
					}
					SERIAL_PROTOCOL(vuitens2);
					SERIAL_PROTOCOLPGM("/8");
					if (sentit2 > 0)
					{
					SERIAL_PROTOCOLPGM(" horari\n ");
					}
					else
					{
					SERIAL_PROTOCOLPGM(" antihorari\n: ");
					}
					
					SERIAL_PROTOCOLPGM(" Voltes cargol 3: ");
					if (numvoltes3 != 0)
					{
					SERIAL_PROTOCOL(numvoltes3);
					SERIAL_PROTOCOLPGM("voltes ");
					}
					SERIAL_PROTOCOL(vuitens3);
					SERIAL_PROTOCOLPGM("/8");
					if (sentit3 > 0)
					{
					SERIAL_PROTOCOLPGM(" horari\n ");
					}
					else
					{
					SERIAL_PROTOCOLPGM(" antihorari\n: ");
					}
					SERIAL_PROTOCOLPGM("\n");
					}
					
					break;
					}
					#endif //SIGMA_BED_AUTOCALIB
					
					case 69: //G69 pause
					{
					Serial.println("G69 ACTIVATED");
					////*******SAVE ACTUIAL POSITION
					saved_position[X_AXIS] = current_position[X_AXIS];
					saved_position[Y_AXIS] = current_position[Y_AXIS];
					saved_position[Z_AXIS] = current_position[Z_AXIS];
					
					//*********************************//
					
					//********RETRACK
					current_position[E_AXIS]-=2;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 50, active_extruder);//Retrack
					st_synchronize();
					//*********************************//
					
					//********MOVE TO PAUSE POSITION
					
					if(current_position[Z_AXIS]>=180) current_position[Z_AXIS] += 2;								//
					else if(current_position[Z_AXIS]>=205) {}														//Move the bed, more or less in function of current_position
					else current_position[Z_AXIS] += 20;															//
					int feedrate=homing_feedrate[Z_AXIS];
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],  current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					st_synchronize();
					
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],  current_position[Z_AXIS], current_position[E_AXIS] -= 4, 400, active_extruder);	//Retrack
					st_synchronize();
					
					feedrate=homing_feedrate[X_AXIS];
					if (active_extruder == LEFT_EXTRUDER){															//Move X axis, controlling the current_extruder
					current_position[X_AXIS] = 0;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],  current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					}else{
					current_position[X_AXIS] = extruder_offset[X_AXIS][1];
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],  current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					}
					st_synchronize();
					//*********************************//
					flag_pause = false;
					break;
					}
					
					case 70: //G70 resume
					Serial.println("G70 ACTIVATED");
					////*******LOAD ACTUIAL POSITION
					current_position[Y_AXIS] = saved_position[Y_AXIS];
					//Serial.println(current_position[Z_AXIS]);
					//*********************************//
					
					//********MOVE TO ORIGINAL POSITION Z
					//if(current_position[Z_AXIS]>=extruder_offset[Z_AXIS]) += 20;
					current_position[Z_AXIS] = saved_position[Z_AXIS];
					feedrate=homing_feedrate[Z_AXIS];
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],  current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					st_synchronize();
					//*********************************//
					
					
					////******PURGE   -->> Purge to clean the extruder, retrack to avoid the trickle
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],  current_position[Z_AXIS], current_position[E_AXIS] += 1.2, 400, active_extruder);
					st_synchronize();
					delay(300);
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],  current_position[Z_AXIS], current_position[E_AXIS] -= 4, 2400, active_extruder);
					st_synchronize();
					//*********************************//
					
					//********MOVE TO ORIGINAL POSITION X
					current_position[X_AXIS] = saved_position[X_AXIS];
					feedrate=homing_feedrate[X_AXIS];
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],  current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
					st_synchronize();
					//*********************************//
					
					//********EXTRACK to keep ready to the new instruction
					current_position[E_AXIS]+=0; //2
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 50, active_extruder);
					st_synchronize();
					//*********************************//
					flag_resume = false;
					break;

					case 29: // G29 Detailed Z-Probe, probes the bed at 3 or more points.
					{
					#if Z_MIN_PIN == -1
					#error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature!!! Z_MIN_PIN must point to a valid hardware pin."
					#endif

					// Prevent user from running a G29 without first homing in X and Y
					if (! (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) )
					{
					LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
					SERIAL_ECHO_START;
					SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
					break; // abort G29, since we don't know where we are
					}

					#ifdef Z_PROBE_SLED
					dock_sled(false);
					#endif // Z_PROBE_SLED
					st_synchronize();
					// make sure the bed_level_rotation_matrix is identity or the planner will get it incorectly
					//vector_3 corrected_position = plan_get_position_mm();
					//corrected_position.debug("position before G29");
					plan_bed_level_matrix.set_to_identity();
					vector_3 uncorrected_position = plan_get_position();
					//uncorrected_position.debug("position durring G29");
					current_position[X_AXIS] = uncorrected_position.x;
					current_position[Y_AXIS] = uncorrected_position.y;
					current_position[Z_AXIS] = uncorrected_position.z;
					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					setup_for_endstop_move();

					feedrate = homing_feedrate[Z_AXIS];
					
					
					#ifdef Z_SIGMA_AUTOLEVEL
					
					#else
					
					#ifdef AUTO_BED_LEVELING_GRID
					// probe at the points of a lattice grid

					int xGridSpacing = (RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);
					int yGridSpacing = (BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);


					// solve the plane equation ax + by + d = z
					// A is the matrix with rows [x y 1] for all the probed points
					// B is the vector of the Z positions
					// the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
					// so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z

					// "A" matrix of the linear system of equations
					double eqnAMatrix[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS*3];
					// "B" vector of Z points
					double eqnBVector[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS];


					int probePointCounter = 0;
					bool zig = true;

					for (int yProbe=FRONT_PROBE_BED_POSITION; yProbe <= BACK_PROBE_BED_POSITION; yProbe += yGridSpacing)
					{
					int xProbe, xInc;
					if (zig)
					{
					xProbe = LEFT_PROBE_BED_POSITION;
					//xEnd = RIGHT_PROBE_BED_POSITION;
					xInc = xGridSpacing;
					zig = false;
					} else // zag
					{
					xProbe = RIGHT_PROBE_BED_POSITION;
					//xEnd = LEFT_PROBE_BED_POSITION;
					xInc = -xGridSpacing;
					zig = true;
					}

					for (int xCount=0; xCount < AUTO_BED_LEVELING_GRID_POINTS; xCount++)
					{
					float z_before;
					if (probePointCounter == 0)
					{
					// raise before probing
					z_before = Z_RAISE_BEFORE_PROBING;
					} else
					{
					// raise extruder
					z_before = current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS;
					}

					float measured_z = probe_pt(xProbe, yProbe, z_before);

					eqnBVector[probePointCounter] = measured_z;

					eqnAMatrix[probePointCounter + 0*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = xProbe;
					eqnAMatrix[probePointCounter + 1*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = yProbe;
					eqnAMatrix[probePointCounter + 2*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = 1;
					probePointCounter++;
					xProbe += xInc;
					}
					}
					clean_up_after_endstop_move();

					// solve lsq problem
					double *plane_equation_coefficients = qr_solve(AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS, 3, eqnAMatrix, eqnBVector);

					SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
					SERIAL_PROTOCOL(plane_equation_coefficients[0]);
					SERIAL_PROTOCOLPGM(" b: ");
					SERIAL_PROTOCOL(plane_equation_coefficients[1]);
					SERIAL_PROTOCOLPGM(" d: ");
					SERIAL_PROTOCOLLN(plane_equation_coefficients[2]);


					set_bed_level_equation_lsq(plane_equation_coefficients);

					free(plane_equation_coefficients);

					#else // AUTO_BED_LEVELING_GRID not defined

					// Probe at 3 arbitrary points
					// probe 1
					float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING);

					// probe 2
					float z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

					// probe 3
					float z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

					clean_up_after_endstop_move();

					//Rapduch: We negated the Z points passed on this functions because the actual correction was inverted
					//set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);
					set_bed_level_equation_3pts(-z_at_pt_1, -z_at_pt_2, -z_at_pt_3);


					#endif // AUTO_BED_LEVELING_GRID
					
					#endif
					

					st_synchronize();

					// The following code correct the Z height difference from z-probe position and hotend tip position.
					// The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
					// When the bed is uneven, this height must be corrected.
					real_z = float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)
					x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER;
					y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER;
					z_tmp = current_position[Z_AXIS];

					apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);         //Apply the correction sending the probe offset
					current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					#ifdef Z_PROBE_SLED
					dock_sled(true, -SLED_DOCKING_OFFSET); // correct for over travel.
					#endif // Z_PROBE_SLED
					}
					break;
					#ifndef Z_PROBE_SLED
					case 30: // G30 Single Z Probe
					{
					engage_z_probe(); // Engage Z Servo endstop if available
					st_synchronize();
					// TODO: make sure the bed_level_rotation_matrix is identity or the planner will get set incorectly
					setup_for_endstop_move();

					feedrate = homing_feedrate[Z_AXIS];

					run_z_probe();
					SERIAL_PROTOCOLPGM(MSG_BED);
					SERIAL_PROTOCOLPGM(" X: ");
					SERIAL_PROTOCOL(current_position[X_AXIS]);
					SERIAL_PROTOCOLPGM(" Y: ");
					SERIAL_PROTOCOL(current_position[Y_AXIS]);
					SERIAL_PROTOCOLPGM(" Z: ");
					SERIAL_PROTOCOL(current_position[Z_AXIS]);
					SERIAL_PROTOCOLPGM("\n");

					clean_up_after_endstop_move();
					retract_z_probe(); // Retract Z Servo endstop if available
					}
					break;
					#else
					case 31: // dock the sled
					dock_sled(true);
					break;
					case 32: // undock the sled
					dock_sled(false);
					break;
					#endif // Z_PROBE_SLED
					#endif // ENABLE_AUTO_BED_LEVELING

					
					case 90: // G90
					relative_mode = false;
					break;
					case 91: // G91
					relative_mode = true;
					break;
					case 92: // G92
					if(!code_seen(axis_codes[E_AXIS]))
					st_synchronize();
					for(int8_t i=0; i < NUM_AXIS; i++) {
					if(code_seen(axis_codes[i])) {
					if(i == E_AXIS) {
					current_position[i] = code_value();
					plan_set_e_position(current_position[E_AXIS]);
					}
					else {
					#ifdef SCARA
					if (i == X_AXIS || i == Y_AXIS) {
					current_position[i] = code_value();
					}
					else {
					current_position[i] = code_value()+add_homing[i];
					}
					#else
					current_position[i] = code_value()+add_homing[i];
					#endif
					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					}
					}
					}
					break;
					}
					}

					else if(code_seen('M'))
					{
					switch( (int)code_value() )
					{
					#ifdef ULTIPANEL
					case 0: // M0 - Unconditional stop - Wait for user button press on LCD
					case 1: // M1 - Conditional stop - Wait for user button press on LCD
					{
					char *src = strchr_pointer + 2;

					codenum = 0;

					bool hasP = false, hasS = false;
					if (code_seen('P')) {
					codenum = code_value(); // milliseconds to wait
					hasP = codenum > 0;
					}
					if (code_seen('S')) {
					codenum = code_value() * 1000; // seconds to wait
					hasS = codenum > 0;
					}
					starpos = strchr(src, '*');
					if (starpos != NULL) *(starpos) = '\0';
					while (*src == ' ') ++src;
					if (!hasP && !hasS && *src != '\0') {
					lcd_setstatus(src);
					} else {
					LCD_MESSAGEPGM(MSG_USERWAIT);
					}

					lcd_ignore_click();
					st_synchronize();
					previous_millis_cmd = millis();
					if (codenum > 0){
					codenum += millis();  // keep track of when we started waiting
					while(millis() < codenum && !lcd_clicked()){
					manage_heater();
					manage_inactivity();
					lcd_update();
					#ifdef SIGMA_TOUCH_SCREEN
					touchscreen_update();
					#endif
					
					}
					lcd_ignore_click(false);
					}else{
					while(!lcd_clicked()){
					manage_heater();
					manage_inactivity();
					lcd_update();
					#ifdef SIGMA_TOUCH_SCREEN
					touchscreen_update();
					#endif
					}
					}
					if (IS_SD_PRINTING)
					LCD_MESSAGEPGM(MSG_RESUMING);
					else
					LCD_MESSAGEPGM(WELCOME_MSG);
					}
					break;
					#endif
					case 17:
						LCD_MESSAGEPGM(MSG_NO_MOVE);
						enable_x();
						enable_y();
						enable_z();
						enable_e0();
						enable_e1();
						enable_e2();
					break;

					#ifdef SDSUPPORT
					case 20: // M20 - list SD card
					SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
					card.ls();
					SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
					break;
					case 21: // M21 - init SD card

					card.initsd();

					break;
					case 22: //M22 - release SD card
					card.release();

					break;
					case 23: //M23 - Select file
						starpos = (strchr(strchr_pointer + 4,'*'));
						if(starpos!=NULL)	*(starpos)='\0';
						card.openFile(strchr_pointer + 4,true);
						break;
					case 24: //M24 - Start SD print
						card.startFileprint();
						starttime=millis();
						//Rapduch
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
						#endif
						break;
					case 25: //M25 - Pause SD print
					card.pauseSDPrint();
					break;
					case 26: //M26 - Set SD index
					if(card.cardOK && code_seen('S')) {
					card.setIndex(code_value_long());
					}
					break;
					case 27: //M27 - Get SD status
					card.getStatus();
					break;
					case 28: //M28 - Start SD write
					starpos = (strchr(strchr_pointer + 4,'*'));
					if(starpos != NULL){
					char* npos = strchr(cmdbuffer[bufindr], 'N');
					strchr_pointer = strchr(npos,' ') + 1;
					*(starpos) = '\0';
					}
					card.openFile(strchr_pointer+4,false);
					break;
					case 29: //M29 - Stop SD write
					//processed in write to file routine above
					//card,saving = false;
					break;
					case 30: //M30 <filename> Delete File
					if (card.cardOK){
					card.closefile();
					starpos = (strchr(strchr_pointer + 4,'*'));
					if(starpos != NULL){
					char* npos = strchr(cmdbuffer[bufindr], 'N');
					strchr_pointer = strchr(npos,' ') + 1;
					*(starpos) = '\0';
					}
					card.removeFile(strchr_pointer + 4);
					}
					break;
					case 32: //M32 - Select file and start SD print
					{
					if(card.sdprinting) {
					st_synchronize();

					}
					starpos = (strchr(strchr_pointer + 4,'*'));

					char* namestartpos = (strchr(strchr_pointer + 4,'!'));   //find ! to indicate filename string start.
					if(namestartpos==NULL)
					{
					namestartpos=strchr_pointer + 4; //default name position, 4 letters after the M
					}
					else
					namestartpos++; //to skip the '!'

					if(starpos!=NULL)
					*(starpos)='\0';

					bool call_procedure=(code_seen('P'));

					if(strchr_pointer>namestartpos)
					call_procedure=false;  //false alert, 'P' found within filename

					if( card.cardOK )
					{
					card.openFile(namestartpos,true,!call_procedure);
					if(code_seen('S'))
					if(strchr_pointer<namestartpos) //only if "S" is occuring _before_ the filename
					card.setIndex(code_value_long());
					card.startFileprint();
					if(!call_procedure)
					starttime=millis(); //procedure calls count as normal print time.
					
					//Rapduch
					#ifdef SIGMA_TOUCH_SCREEN
					genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING,0);
					#endif
					}
					
					
					} break;
					case 928: //M928 - Start SD write
					starpos = (strchr(strchr_pointer + 5,'*'));
					if(starpos != NULL){
					char* npos = strchr(cmdbuffer[bufindr], 'N');
					strchr_pointer = strchr(npos,' ') + 1;
					*(starpos) = '\0';
					}
					card.openLogFile(strchr_pointer+5);
					break;

					#endif //SDSUPPORT

					case 31: //M31 take time since the start of the SD print or an M109 command
					{
					stoptime=millis();
					char time[30];
					unsigned long t=(stoptime-starttime)/1000;
					int sec,min;
					min=t/60;
					sec=t%60;
					sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
					SERIAL_ECHO_START;
					SERIAL_ECHOLN(time);
					lcd_setstatus(time);
					autotempShutdown();
					}
					break;
					case 42: //M42 -Change pin status via gcode
					if (code_seen('S'))
					{
					int pin_status = code_value();
					int pin_number = LED_PIN;
					if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
					pin_number = code_value();
					for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(int)); i++)
					{
					if (sensitive_pins[i] == pin_number)
					{
					pin_number = -1;
					break;
					}
					}
					#if defined(FAN_PIN) && FAN_PIN > -1
					if (pin_number == FAN_PIN)
					fanSpeed = pin_status;
					#endif
					if (pin_number > -1)
					{
					pinMode(pin_number, OUTPUT);
					digitalWrite(pin_number, pin_status);
					analogWrite(pin_number, pin_status);
					}
					}
					break;

					// M48 Z-Probe repeatability measurement function.
					//
					// Usage:   M48 <n #_samples> <X X_position_for_samples> <Y Y_position_for_samples> <V Verbose_Level> <Engage_probe_for_each_reading> <L legs_of_movement_prior_to_doing_probe>
					//
					// This function assumes the bed has been homed.  Specificaly, that a G28 command
					// as been issued prior to invoking the M48 Z-Probe repeatability measurement function.
					// Any information generated by a prior G29 Bed leveling command will be lost and need to be
					// regenerated.
					//
					// The number of samples will default to 10 if not specified.  You can use upper or lower case
					// letters for any of the options EXCEPT n.  n must be in lower case because Marlin uses a capital
					// N for its communication protocol and will get horribly confused if you send it a capital N.
					//

					#ifdef ENABLE_AUTO_BED_LEVELING
					#ifdef Z_PROBE_REPEATABILITY_TEST

					case 48: // M48 Z-Probe repeatability
					{
					#if Z_MIN_PIN == -1
					#error "You must have a Z_MIN endstop in order to enable calculation of Z-Probe repeatability."
					#endif

					double sum=0.0;
					double mean=0.0;
					double sigma=0.0;
					double sample_set[50];
					int verbose_level=1, n=0, j, n_samples = 10, n_legs=0, engage_probe_for_each_reading=0 ;
					double X_current, Y_current, Z_current;
					double X_probe_location, Y_probe_location, Z_start_location, ext_position;
					
					if (code_seen('V') || code_seen('v')) {
					verbose_level = code_value();
					if (verbose_level<0 || verbose_level>4 ) {
					SERIAL_PROTOCOLPGM("?Verbose Level not plausable.\n");
					goto Sigma_Exit;
					}
					}

					if (verbose_level > 0)   {
					SERIAL_PROTOCOLPGM("M48 Z-Probe Repeatability test.   Version 2.00\n");
					SERIAL_PROTOCOLPGM("Full support at: http://3dprintboard.com/forum.php\n");
					}

					if (code_seen('n')) {
					n_samples = code_value();
					if (n_samples<4 || n_samples>50 ) {
					SERIAL_PROTOCOLPGM("?Specified sample size not plausable.\n");
					goto Sigma_Exit;
					}
					}

					X_current = X_probe_location = st_get_position_mm(X_AXIS);
					Y_current = Y_probe_location = st_get_position_mm(Y_AXIS);
					Z_current = st_get_position_mm(Z_AXIS);
					Z_start_location = st_get_position_mm(Z_AXIS) + Z_RAISE_BEFORE_PROBING;
					ext_position	 = st_get_position_mm(E_AXIS);

					if (code_seen('E') || code_seen('e') )
					engage_probe_for_each_reading++;

					if (code_seen('X') || code_seen('x') ) {
					X_probe_location = code_value() -  X_PROBE_OFFSET_FROM_EXTRUDER;
					if (X_probe_location<X_MIN_POS || X_probe_location>X_MAX_POS ) {
					SERIAL_PROTOCOLPGM("?Specified X position out of range.\n");
					goto Sigma_Exit;
					}
					}

					if (code_seen('Y') || code_seen('y') ) {
					Y_probe_location = code_value() -  Y_PROBE_OFFSET_FROM_EXTRUDER;
					if (Y_probe_location<Y_MIN_POS || Y_probe_location>Y_MAX_POS ) {
					SERIAL_PROTOCOLPGM("?Specified Y position out of range.\n");
					goto Sigma_Exit;
					}
					}

					if (code_seen('L') || code_seen('l') ) {
					n_legs = code_value();
					if ( n_legs==1 )
					n_legs = 2;
					if ( n_legs<0 || n_legs>15 ) {
					SERIAL_PROTOCOLPGM("?Specified number of legs in movement not plausable.\n");
					goto Sigma_Exit;
					}
					}

					//
					// Do all the preliminary setup work.   First raise the probe.
					//

					st_synchronize();
					plan_bed_level_matrix.set_to_identity();
					plan_buffer_line( X_current, Y_current, Z_start_location,
					ext_position,
					homing_feedrate[Z_AXIS]/60,
					active_extruder);
					st_synchronize();

					//
					// Now get everything to the specified probe point So we can safely do a probe to
					// get us close to the bed.  If the Z-Axis is far from the bed, we don't want to
					// use that as a starting point for each probe.
					//
					if (verbose_level > 2)
					SERIAL_PROTOCOL("Positioning probe for the test.\n");

					plan_buffer_line( X_probe_location, Y_probe_location, Z_start_location,
					ext_position,
					homing_feedrate[X_AXIS]/60,
					active_extruder);
					st_synchronize();

					current_position[X_AXIS] = X_current = st_get_position_mm(X_AXIS);
					current_position[Y_AXIS] = Y_current = st_get_position_mm(Y_AXIS);
					current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
					current_position[E_AXIS] = ext_position = st_get_position_mm(E_AXIS);

					//
					// OK, do the inital probe to get us close to the bed.
					// Then retrace the right amount and use that in subsequent probes
					//

					engage_z_probe();

					setup_for_endstop_move();
					run_z_probe();

					current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
					Z_start_location = st_get_position_mm(Z_AXIS) + Z_RAISE_BEFORE_PROBING;

					plan_buffer_line( X_probe_location, Y_probe_location, Z_start_location,
					ext_position,
					homing_feedrate[X_AXIS]/60,
					active_extruder);
					st_synchronize();
					current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);

					if (engage_probe_for_each_reading)
					retract_z_probe();

					for( n=0; n<n_samples; n++) {

					do_blocking_move_to( X_probe_location, Y_probe_location, Z_start_location); // Make sure we are at the probe location

					if ( n_legs)  {
					double radius=0.0, theta=0.0, x_sweep, y_sweep;
					int rotational_direction, l;

					rotational_direction = (unsigned long) millis() & 0x0001;			// clockwise or counter clockwise
					radius = (unsigned long) millis() % (long) (X_MAX_LENGTH/4); 			// limit how far out to go
					theta = (float) ((unsigned long) millis() % (long) 360) / (360./(2*3.1415926));	// turn into radians

					//SERIAL_ECHOPAIR("starting radius: ",radius);
					//SERIAL_ECHOPAIR("   theta: ",theta);
					//SERIAL_ECHOPAIR("   direction: ",rotational_direction);
					//SERIAL_PROTOCOLLNPGM("");

					for( l=0; l<n_legs-1; l++) {
					if (rotational_direction==1)
					theta += (float) ((unsigned long) millis() % (long) 20) / (360.0/(2*3.1415926)); // turn into radians
					else
					theta -= (float) ((unsigned long) millis() % (long) 20) / (360.0/(2*3.1415926)); // turn into radians

					radius += (float) ( ((long) ((unsigned long) millis() % (long) 10)) - 5);
					if ( radius<0.0 )
					radius = -radius;

					X_current = X_probe_location + cos(theta) * radius;
					Y_current = Y_probe_location + sin(theta) * radius;

					if ( X_current<X_MIN_POS)		// Make sure our X & Y are sane
					X_current = X_MIN_POS;
					if ( X_current>X_MAX_POS)
					X_current = X_MAX_POS;

					if ( Y_current<Y_MIN_POS)		// Make sure our X & Y are sane
					Y_current = Y_MIN_POS;
					if ( Y_current>Y_MAX_POS)
					Y_current = Y_MAX_POS;

					if (verbose_level>3 ) {
					SERIAL_ECHOPAIR("x: ", X_current);
					SERIAL_ECHOPAIR("y: ", Y_current);
					SERIAL_PROTOCOLLNPGM("");
					}

					do_blocking_move_to( X_current, Y_current, Z_current );
					}
					do_blocking_move_to( X_probe_location, Y_probe_location, Z_start_location); // Go back to the probe location
					}

					if (engage_probe_for_each_reading)  {
					engage_z_probe();
					delay(1000);
					}

					setup_for_endstop_move();
					run_z_probe();

					sample_set[n] = current_position[Z_AXIS];

					//
					// Get the current mean for the data points we have so far
					//
					sum=0.0;
					for( j=0; j<=n; j++) {
					sum = sum + sample_set[j];
					}
					mean = sum / (double (n+1));
					//
					// Now, use that mean to calculate the standard deviation for the
					// data points we have so far
					//

					sum=0.0;
					for( j=0; j<=n; j++) {
					sum = sum + (sample_set[j]-mean) * (sample_set[j]-mean);
					}
					sigma = sqrt( sum / (double (n+1)) );

					if (verbose_level > 1) {
					SERIAL_PROTOCOL(n+1);
					SERIAL_PROTOCOL(" of ");
					SERIAL_PROTOCOL(n_samples);
					SERIAL_PROTOCOLPGM("   z: ");
					SERIAL_PROTOCOL_F(current_position[Z_AXIS], 6);
					}

					if (verbose_level > 2) {
					SERIAL_PROTOCOL(" mean: ");
					SERIAL_PROTOCOL_F(mean,6);

					SERIAL_PROTOCOL("   sigma: ");
					SERIAL_PROTOCOL_F(sigma,6);
					}

					if (verbose_level > 0)
					SERIAL_PROTOCOLPGM("\n");

					plan_buffer_line( X_probe_location, Y_probe_location, Z_start_location,
					current_position[E_AXIS], homing_feedrate[Z_AXIS]/60, active_extruder);
					st_synchronize();

					if (engage_probe_for_each_reading)  {
					retract_z_probe();
					delay(1000);
					}
					}

					retract_z_probe();
					delay(1000);

					clean_up_after_endstop_move();

					//      enable_endstops(true);

					if (verbose_level > 0) {
					SERIAL_PROTOCOLPGM("Mean: ");
					SERIAL_PROTOCOL_F(mean, 6);
					SERIAL_PROTOCOLPGM("\n");
					}

					SERIAL_PROTOCOLPGM("Standard Deviation: ");
					SERIAL_PROTOCOL_F(sigma, 6);
					SERIAL_PROTOCOLPGM("\n\n");

					Sigma_Exit:
					break;
					}
					#endif		// Z_PROBE_REPEATABILITY_TEST
					#endif		// ENABLE_AUTO_BED_LEVELING

					case 104: // M104
					if(setTargetedHotend(104)){
					break;
					}
					if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
					#ifdef DUAL_X_CARRIAGE
					if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
					setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
					#endif
					setWatch();
					break;
					case 112: //  M112 -Emergency Stop
					kill();
					break;
					case 140: // M140 set bed temp
					if (code_seen('S')) setTargetBed(code_value());
					break;
					case 105 : // M105
					if(setTargetedHotend(105)){
					break;
					}
					#if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
					SERIAL_PROTOCOLPGM("ok T:");
					SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
					SERIAL_PROTOCOLPGM(" /");
					SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
					#if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
					SERIAL_PROTOCOLPGM(" B:");
					SERIAL_PROTOCOL_F(degBed(),1);
					SERIAL_PROTOCOLPGM(" /");
					SERIAL_PROTOCOL_F(degTargetBed(),1);
					#endif //TEMP_BED_PIN
					for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
					SERIAL_PROTOCOLPGM(" T");
					SERIAL_PROTOCOL(cur_extruder);
					SERIAL_PROTOCOLPGM(":");
					SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
					SERIAL_PROTOCOLPGM(" /");
					SERIAL_PROTOCOL_F(degTargetHotend(cur_extruder),1);
					}
					#else
					SERIAL_ERROR_START;
					SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
					#endif

					SERIAL_PROTOCOLPGM(" @:");
					#ifdef EXTRUDER_WATTS
					SERIAL_PROTOCOL((EXTRUDER_WATTS * getHeaterPower(tmp_extruder))/127);
					SERIAL_PROTOCOLPGM("W");
					#else
					SERIAL_PROTOCOL(getHeaterPower(tmp_extruder));
					#endif

					SERIAL_PROTOCOLPGM(" B@:");
					#ifdef BED_WATTS
					SERIAL_PROTOCOL((BED_WATTS * getHeaterPower(-1))/127);
					SERIAL_PROTOCOLPGM("W");
					#else
					SERIAL_PROTOCOL(getHeaterPower(-1));
					#endif

					#ifdef SHOW_TEMP_ADC_VALUES
					#if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
					SERIAL_PROTOCOLPGM("    ADC B:");
					SERIAL_PROTOCOL_F(degBed(),1);
					SERIAL_PROTOCOLPGM("C->");
					SERIAL_PROTOCOL_F(rawBedTemp()/OVERSAMPLENR,0);
					#endif
					for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
					SERIAL_PROTOCOLPGM("  T");
					SERIAL_PROTOCOL(cur_extruder);
					SERIAL_PROTOCOLPGM(":");
					SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
					SERIAL_PROTOCOLPGM("C->");
					SERIAL_PROTOCOL_F(rawHotendTemp(cur_extruder)/OVERSAMPLENR,0);
					}
					#endif

					SERIAL_PROTOCOLLN("");
					return;
					break;
					case 109:
					{// M109 - Wait for extruder heater to reach target.
					if(setTargetedHotend(109)){
					break;
					}
					LCD_MESSAGEPGM(MSG_HEATING);
					#ifdef AUTOTEMP
					autotemp_enabled=false;
					#endif
					if (code_seen('S')) {
					setTargetHotend(code_value(), tmp_extruder);
					#ifdef DUAL_X_CARRIAGE
					if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
					setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
					#endif
					CooldownNoWait = true;
					} else if (code_seen('R')) {
					setTargetHotend(code_value(), tmp_extruder);
					#ifdef DUAL_X_CARRIAGE
					if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
					setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
					#endif
					CooldownNoWait = false;
					}
					#ifdef AUTOTEMP
					if (code_seen('S')) autotemp_min=code_value();
					if (code_seen('B')) autotemp_max=code_value();
					if (code_seen('F'))
					{
					autotemp_factor=code_value();
					autotemp_enabled=true;
					}
					#endif

					setWatch();
					codenum = millis();

					/* See if we are heating up or cooling down */
					target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling

					cancel_heatup = false;

					#ifdef TEMP_RESIDENCY_TIME
					long residencyStart;
					residencyStart = -1;
					/* continue to loop until we have reached the target temp
					_and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
					while((!cancel_heatup)&&((residencyStart == -1) ||
					(residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL)))) ) {
					#else
					while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) {
					#endif //TEMP_RESIDENCY_TIME
					if( (millis() - codenum) > 1000UL )
					{ //Print Temp Reading and remaining time every 1 second while heating up/cooling down
					SERIAL_PROTOCOLPGM("T:");
					SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
					SERIAL_PROTOCOLPGM(" E:");
					SERIAL_PROTOCOL((int)tmp_extruder);
					#ifdef TEMP_RESIDENCY_TIME
					SERIAL_PROTOCOLPGM(" W:");
					if(residencyStart > -1)
					{
					codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
					SERIAL_PROTOCOLLN( codenum );
					}
					else
					{
					SERIAL_PROTOCOLLN( "?" );
					}
					#else
					SERIAL_PROTOCOLLN("");
					#endif
					codenum = millis();
					}
					manage_heater();
					manage_inactivity();
					//lcd_update();
					#ifdef SIGMA_TOUCH_SCREEN
					touchscreen_update();
					#endif
					
					if (card.sdprinting==false && !card.sdispaused) //Added ispaused from cardreader
					{
					break; //Break if we are trying to heat when the fileprinting has been stopped and is not paused
					}
					
					#ifdef TEMP_RESIDENCY_TIME
					/* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
					or when current temp falls outside the hysteresis after target temp was reached */
					if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
					(residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
					(residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
					{
					residencyStart = millis();
					}
					#endif //TEMP_RESIDENCY_TIME
					}
					Serial.println("Extruder Heated");
					LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
					starttime=millis();
					previous_millis_cmd = millis();
					}
					break;
					case 190: // M190 - Wait for bed heater to reach target.
					#if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
					LCD_MESSAGEPGM(MSG_BED_HEATING);
					if (code_seen('S')) {
					setTargetBed(code_value());
					CooldownNoWait = true;
					} else if (code_seen('R')) {
					setTargetBed(code_value());
					CooldownNoWait = false;
					}
					codenum = millis();
					
					cancel_heatup = false;
					target_direction = isHeatingBed(); // true if heating, false if cooling

					while ( (target_direction)&&(!cancel_heatup) ? (isHeatingBed()) : (isCoolingBed()&&(CooldownNoWait==false)) )
					{
					if(( millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
					{
					float tt=degHotend(active_extruder);
					SERIAL_PROTOCOLPGM("T:");
					SERIAL_PROTOCOL(tt);
					SERIAL_PROTOCOLPGM(" E:");
					SERIAL_PROTOCOL((int)active_extruder);
					SERIAL_PROTOCOLPGM(" B:");
					SERIAL_PROTOCOL_F(degBed(),1);
					SERIAL_PROTOCOLLN("");
					codenum = millis();
					}
					manage_heater();
					manage_inactivity();
					//lcd_update();
					#ifdef SIGMA_TOUCH_SCREEN
					touchscreen_update();
					#endif
					
					if (card.sdprinting==false && !card.sdispaused) //Added ispaused from cardreader
					{
					break; //Break if we are trying to heat when the fileprinting has been stopped and is not paused
					}
					}
					Serial.println("Bed Heated");
					LCD_MESSAGEPGM(MSG_BED_DONE);
					previous_millis_cmd = millis();
					#endif
					break;

					#if defined(FAN_PIN) && FAN_PIN > -1
					case 106: //M106 Fan On
					if (code_seen('S')){
					fanSpeed=constrain(code_value(),0,255);
					}
					else {
					fanSpeed=255;
					}
					break;
					case 107: //M107 Fan Off
					fanSpeed = 0;
					break;
					#endif //FAN_PIN
					#ifdef BARICUDA
					// PWM for HEATER_1_PIN
					#if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
					case 126: //M126 valve open
					if (code_seen('S')){
					ValvePressure=constrain(code_value(),0,255);
					}
					else {
					ValvePressure=255;
					}
					break;
					case 127: //M127 valve closed
					ValvePressure = 0;
					break;
					#endif //HEATER_1_PIN

					// PWM for HEATER_2_PIN
					#if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
					case 128: //M128 valve open
					if (code_seen('S')){
					EtoPPressure=constrain(code_value(),0,255);
					}
					else {
					EtoPPressure=255;
					}
					break;
					case 129: //M129 valve closed
					EtoPPressure = 0;
					break;
					#endif //HEATER_2_PIN
					#endif

					#if defined(PS_ON_PIN) && PS_ON_PIN > -1
					case 80: // M80 - Turn on Power Supply
					SET_OUTPUT(PS_ON_PIN); //GND
					WRITE(PS_ON_PIN, PS_ON_AWAKE);

					// If you have a switch on suicide pin, this is useful
					// if you want to start another print with suicide feature after
					// a print without suicide...
					#if defined SUICIDE_PIN && SUICIDE_PIN > -1
					SET_OUTPUT(SUICIDE_PIN);
					WRITE(SUICIDE_PIN, HIGH);
					#endif

					#ifdef ULTIPANEL
					powersupply = true;
					LCD_MESSAGEPGM(WELCOME_MSG);
					lcd_update();
					#endif
					break;
					#endif

					case 81: // M81 - Turn off Power Supply
					disable_heater();
					st_synchronize();
					disable_e0();
					disable_e1();
					disable_e2();
					finishAndDisableSteppers();
					fanSpeed = 0;
					delay(1000); // Wait a little before to switch off
					#if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
					st_synchronize();
					suicide();
					#elif defined(PS_ON_PIN) && PS_ON_PIN > -1
					SET_OUTPUT(PS_ON_PIN);
					WRITE(PS_ON_PIN, PS_ON_ASLEEP);
					#endif
					#ifdef ULTIPANEL
					powersupply = false;
					LCD_MESSAGEPGM(MACHINE_NAME" "MSG_OFF".");
					lcd_update();
					#endif
					break;

					case 82:
					axis_relative_modes[3] = false;
					break;
					case 83:
					axis_relative_modes[3] = true;
					break;
					case 18: //compatibility
					case 84: // M84
					if(code_seen('S')){
					stepper_inactive_time = code_value() * 1000;
					}
					else
					{
					bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
					if(all_axis)
					{
					st_synchronize();
					disable_e0();
					disable_e1();
					disable_e2();
					finishAndDisableSteppers();
					}
					else
					{
					st_synchronize();
					if(code_seen('X')) disable_x();
					if(code_seen('Y')) disable_y();
					if(code_seen('Z')) disable_z();
					#if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
					if(code_seen('E')) {
					disable_e0();
					disable_e1();
					disable_e2();
					}
					#endif
					}
					}
					break;
					case 85: // M85
					if(code_seen('S')) {
					max_inactive_time = code_value() * 1000;
					}
					break;
					case 92: // M92
					for(int8_t i=0; i < NUM_AXIS; i++)
					{
					if(code_seen(axis_codes[i]))
					{
					if(i == 3) { // E
					float value = code_value();
					if(value < 20.0) {
					float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
					max_e_jerk *= factor;
					max_feedrate[i] *= factor;
					axis_steps_per_sqr_second[i] *= factor;
					}
					axis_steps_per_unit[i] = value;
					}
					else {
					axis_steps_per_unit[i] = code_value();
					}
					}
					}
					break;
					case 115: // M115
					SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
					break;
					case 117: // M117 display message
					starpos = (strchr(strchr_pointer + 5,'*'));
					if(starpos!=NULL)
					*(starpos)='\0';
					lcd_setstatus(strchr_pointer + 5);
					break;
					case 114: // M114
					SERIAL_PROTOCOLPGM("X:");
					SERIAL_PROTOCOL(current_position[X_AXIS]);
					SERIAL_PROTOCOLPGM(" Y:");
					SERIAL_PROTOCOL(current_position[Y_AXIS]);
					SERIAL_PROTOCOLPGM(" Z:");
					SERIAL_PROTOCOL(current_position[Z_AXIS]);
					SERIAL_PROTOCOLPGM(" E:");
					SERIAL_PROTOCOL(current_position[E_AXIS]);

					SERIAL_PROTOCOLPGM(MSG_COUNT_X);
					SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
					SERIAL_PROTOCOLPGM(" Y:");
					SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
					SERIAL_PROTOCOLPGM(" Z:");
					SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

					SERIAL_PROTOCOLLN("");
					#ifdef SCARA
					SERIAL_PROTOCOLPGM("SCARA Theta:");
					SERIAL_PROTOCOL(delta[X_AXIS]);
					SERIAL_PROTOCOLPGM("   Psi+Theta:");
					SERIAL_PROTOCOL(delta[Y_AXIS]);
					SERIAL_PROTOCOLLN("");
					
					SERIAL_PROTOCOLPGM("SCARA Cal - Theta:");
					SERIAL_PROTOCOL(delta[X_AXIS]+add_homing[0]);
					SERIAL_PROTOCOLPGM("   Psi+Theta (90):");
					SERIAL_PROTOCOL(delta[Y_AXIS]-delta[X_AXIS]-90+add_homing[1]);
					SERIAL_PROTOCOLLN("");
					
					SERIAL_PROTOCOLPGM("SCARA step Cal - Theta:");
					SERIAL_PROTOCOL(delta[X_AXIS]/90*axis_steps_per_unit[X_AXIS]);
					SERIAL_PROTOCOLPGM("   Psi+Theta:");
					SERIAL_PROTOCOL((delta[Y_AXIS]-delta[X_AXIS])/90*axis_steps_per_unit[Y_AXIS]);
					SERIAL_PROTOCOLLN("");
					SERIAL_PROTOCOLLN("");
					#endif
					break;
					case 120: // M120
					enable_endstops(false) ;
					break;
					case 121: // M121
					enable_endstops(true) ;
					break;
					case 119: // M119
					SERIAL_PROTOCOLLN(MSG_M119_REPORT);
					#if defined(X_MIN_PIN) && X_MIN_PIN > -1
					SERIAL_PROTOCOLPGM(MSG_X_MIN);
					SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
					#endif
					#if defined(X_MAX_PIN) && X_MAX_PIN > -1
					SERIAL_PROTOCOLPGM(MSG_X_MAX);
					SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
					#endif
					#if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
					SERIAL_PROTOCOLPGM(MSG_Y_MIN);
					SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
					#endif
					#if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
					SERIAL_PROTOCOLPGM(MSG_Y_MAX);
					SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
					#endif
					#if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
					SERIAL_PROTOCOLPGM(MSG_Z_MIN);
					SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
					#endif
					#if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
					SERIAL_PROTOCOLPGM(MSG_Z_MAX);
					SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
					#endif
					break;
					//TODO: update for all axis, use for loop
					#ifdef BLINKM
					case 150: // M150
					{
					byte red;
					byte grn;
					byte blu;

					if(code_seen('R')) red = code_value();
					if(code_seen('U')) grn = code_value();
					if(code_seen('B')) blu = code_value();

					SendColors(red,grn,blu);
					}
					break;
					#endif //BLINKM
					case 200: // M200 D<millimeters> set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
					{
					float area = .0;
					float radius = .0;
					if(code_seen('D')) {
					radius = (float)code_value() * .5;
					if(radius == 0) {
					area = 1;
					} else {
					area = M_PI * pow(radius, 2);
					}
					} else {
					//reserved for setting filament diameter via UFID or filament measuring device
					break;
					
					
					}
					tmp_extruder = active_extruder;
					if(code_seen('T')) {
					tmp_extruder = code_value();
					if(tmp_extruder >= EXTRUDERS) {
					SERIAL_ECHO_START;
					SERIAL_ECHO(MSG_M200_INVALID_EXTRUDER);
					break;
					}
					}
					volumetric_multiplier[tmp_extruder] = 1 / area;
					}
					break;
					case 201: // M201
					for(int8_t i=0; i < NUM_AXIS; i++)
					{
					if(code_seen(axis_codes[i]))
					{
					max_acceleration_units_per_sq_second[i] = code_value();
					}
					}
					// steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
					reset_acceleration_rates();
					break;
					#if 0 // Not used for Sprinter/grbl gen6
					case 202: // M202
					for(int8_t i=0; i < NUM_AXIS; i++) {
					if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
					}
					break;
					#endif
					case 203: // M203 max feedrate mm/sec
					for(int8_t i=0; i < NUM_AXIS; i++) {
					if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
					}
					break;
					case 204: // M204 acclereration S normal moves T filmanent only moves
					{
					if(code_seen('S')) acceleration = code_value() ;
					if(code_seen('T')) retract_acceleration = code_value() ;
					}
					break;
					case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
					{
					if(code_seen('S')) minimumfeedrate = code_value();
					if(code_seen('T')) mintravelfeedrate = code_value();
					if(code_seen('B')) minsegmenttime = code_value() ;
					if(code_seen('X')) max_xy_jerk = code_value() ;
					if(code_seen('Z')) max_z_jerk = code_value() ;
					if(code_seen('E')) max_e_jerk = code_value() ;
					}
					break;
					case 206: // M206 additional homing offset
					for(int8_t i=0; i < 3; i++)
					{
					if(code_seen(axis_codes[i])) add_homing[i] = code_value();
					}
					#ifdef SCARA
					if(code_seen('T'))       // Theta
					{
					add_homing[0] = code_value() ;
					}
					if(code_seen('P'))       // Psi
					{
					add_homing[1] = code_value() ;
					}
					#endif
					break;
					#ifdef DELTA
					case 665: // M665 set delta configurations L<diagonal_rod> R<delta_radius> S<segments_per_sec>
					if(code_seen('L')) {
					delta_diagonal_rod= code_value();
					}
					if(code_seen('R')) {
					delta_radius= code_value();
					}
					if(code_seen('S')) {
					delta_segments_per_second= code_value();
					}
					
					recalc_delta_settings(delta_radius, delta_diagonal_rod);
					break;
					case 666: // M666 set delta endstop adjustemnt
					for(int8_t i=0; i < 3; i++)
					{
					if(code_seen(axis_codes[i])) endstop_adj[i] = code_value();
					}
					break;
					#endif
					#ifdef FWRETRACT
					case 207: //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
					{
					if(code_seen('S'))
					{
					retract_length = code_value() ;
					}
					if(code_seen('F'))
					{
					retract_feedrate = code_value()/60 ;
					}
					if(code_seen('Z'))
					{
					retract_zlift = code_value() ;
					}
					}break;
					case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
					{
					if(code_seen('S'))
					{
					retract_recover_length = code_value() ;
					}
					if(code_seen('F'))
					{
					retract_recover_feedrate = code_value()/60 ;
					}
					}break;
					case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
					{
					if(code_seen('S'))
					{
					int t= code_value() ;
					switch(t)
					{
					case 0:
					{
					autoretract_enabled=false;
					retracted[0]=false;
					#if EXTRUDERS > 1
					retracted[1]=false;
					#endif
					#if EXTRUDERS > 2
					retracted[2]=false;
					#endif
					}break;
					case 1:
					{
					autoretract_enabled=true;
					retracted[0]=false;
					#if EXTRUDERS > 1
					retracted[1]=false;
					#endif
					#if EXTRUDERS > 2
					retracted[2]=false;
					#endif
					}break;
					default:
					SERIAL_ECHO_START;
					SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
					SERIAL_ECHO(cmdbuffer[bufindr]);
					SERIAL_ECHOLNPGM("\"");
					}
					}

					}break;
					#endif // FWRETRACT
					#if EXTRUDERS > 1
					case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
					{
					if(setTargetedHotend(218)){
					break;
					}
					if(code_seen('X'))
					{
					extruder_offset[X_AXIS][tmp_extruder] = code_value();
					}
					if(code_seen('Y'))
					{
					extruder_offset[Y_AXIS][tmp_extruder] = code_value();
					}
					#ifdef DUAL_X_CARRIAGE
					if(code_seen('Z'))
					{
					extruder_offset[Z_AXIS][tmp_extruder] = code_value();
					}
					#endif
					SERIAL_ECHO_START;
					SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
					for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
					{
					SERIAL_ECHO(" ");
					SERIAL_ECHO(extruder_offset[X_AXIS][tmp_extruder]);
					SERIAL_ECHO(",");
					SERIAL_ECHO(extruder_offset[Y_AXIS][tmp_extruder]);
					#ifdef DUAL_X_CARRIAGE
					SERIAL_ECHO(",");
					SERIAL_ECHO(extruder_offset[Z_AXIS][tmp_extruder]);
					#endif
					}
					SERIAL_ECHOLN("");
					}break;
					#endif
					case 220: // M220 S<factor in percent>- set speed factor override percentage
					{
					if(code_seen('S'))
					{
					feedmultiply = code_value() ;
					}
					}
					break;
					case 221: // M221 S<factor in percent>- set extrude factor override percentage
					{
					if(code_seen('S'))
					{
					int tmp_code = code_value();
					if (code_seen('T'))
					{
					if(setTargetedHotend(221)){
					break;
					}
					extruder_multiply[tmp_extruder] = tmp_code;
					}
					else
					{
					extrudemultiply = tmp_code ;
					}
					}
					}
					break;

					case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
					{
					if(code_seen('P')){
					int pin_number = code_value(); // pin number
					int pin_state = -1; // required pin state - default is inverted

					if(code_seen('S')) pin_state = code_value(); // required pin state

					if(pin_state >= -1 && pin_state <= 1){

					for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(int)); i++)
					{
					if (sensitive_pins[i] == pin_number)
					{
					pin_number = -1;
					break;
					}
					}

					if (pin_number > -1)
					{
					st_synchronize();

					pinMode(pin_number, INPUT);

					int target;
					switch(pin_state){
					case 1:
					target = HIGH;
					break;

					case 0:
					target = LOW;
					break;

					case -1:
					target = !digitalRead(pin_number);
					break;
					}

					while(digitalRead(pin_number) != target){
					manage_heater();
					manage_inactivity();
					//lcd_update();
					#ifdef SIGMA_TOUCH_SCREEN
					touchscreen_update();
					#endif
					}
					}
					}
					}
					}
					break;

					#if NUM_SERVOS > 0
					case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
					{
					int servo_index = -1;
					int servo_position = 0;
					if (code_seen('P'))
					servo_index = code_value();
					if (code_seen('S')) {
					servo_position = code_value();
					if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
					#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
					servos[servo_index].attach(0);
					#endif
					servos[servo_index].write(servo_position);
					#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
					delay(PROBE_SERVO_DEACTIVATION_DELAY);
					servos[servo_index].detach();
					#endif
					}
					else {
					SERIAL_ECHO_START;
					SERIAL_ECHO("Servo ");
					SERIAL_ECHO(servo_index);
					SERIAL_ECHOLN(" out of range");
					}
					}
					else if (servo_index >= 0) {
					SERIAL_PROTOCOL(MSG_OK);
					SERIAL_PROTOCOL(" Servo ");
					SERIAL_PROTOCOL(servo_index);
					SERIAL_PROTOCOL(": ");
					SERIAL_PROTOCOL(servos[servo_index].read());
					SERIAL_PROTOCOLLN("");
					}
					}
					break;
					#endif // NUM_SERVOS > 0

					#if (LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) || defined(LCD_USE_I2C_BUZZER)))
					case 300: // M300
					{
					int beepS = code_seen('S') ? code_value() : 110;
					int beepP = code_seen('P') ? code_value() : 1000;
					if (beepS > 0)
					{
					#if BEEPER > 0
					tone(BEEPER, beepS);
					delay(beepP);
					noTone(BEEPER);
					#elif defined(ULTRALCD)
					lcd_buzz(beepS, beepP);
					#elif defined(LCD_USE_I2C_BUZZER)
					lcd_buzz(beepP, beepS);
					#endif
					}
					else
					{
					delay(beepP);
					}
					}
					break;
					#endif // M300

					#ifdef PIDTEMP
					case 301: // M301
					{
					if(code_seen('P')) Kp = code_value();
					if(code_seen('I')) Ki = scalePID_i(code_value());
					if(code_seen('D')) Kd = scalePID_d(code_value());

					#ifdef PID_ADD_EXTRUSION_RATE
					if(code_seen('C')) Kc = code_value();
					#endif

					updatePID();
					SERIAL_PROTOCOL(MSG_OK);
					SERIAL_PROTOCOL(" p:");
					SERIAL_PROTOCOL(Kp);
					SERIAL_PROTOCOL(" i:");
					SERIAL_PROTOCOL(unscalePID_i(Ki));
					SERIAL_PROTOCOL(" d:");
					SERIAL_PROTOCOL(unscalePID_d(Kd));
					#ifdef PID_ADD_EXTRUSION_RATE
					SERIAL_PROTOCOL(" c:");
					//Kc does not have scaling applied above, or in resetting defaults
					SERIAL_PROTOCOL(Kc);
					#endif
					SERIAL_PROTOCOLLN("");
					}
					break;
					#endif //PIDTEMP
					#ifdef PIDTEMPBED
					case 304: // M304
					{
					if(code_seen('P')) bedKp = code_value();
					if(code_seen('I')) bedKi = scalePID_i(code_value());
					if(code_seen('D')) bedKd = scalePID_d(code_value());

					updatePID();
					SERIAL_PROTOCOL(MSG_OK);
					SERIAL_PROTOCOL(" p:");
					SERIAL_PROTOCOL(bedKp);
					SERIAL_PROTOCOL(" i:");
					SERIAL_PROTOCOL(unscalePID_i(bedKi));
					SERIAL_PROTOCOL(" d:");
					SERIAL_PROTOCOL(unscalePID_d(bedKd));
					SERIAL_PROTOCOLLN("");
					}
					break;
					#endif //PIDTEMP
					case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
					{
					#ifdef CHDK
					
					SET_OUTPUT(CHDK);
					WRITE(CHDK, HIGH);
					chdkHigh = millis();
					chdkActive = true;
					
					#else
					
					#if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
					const uint8_t NUM_PULSES=16;
					const float PULSE_LENGTH=0.01524;
					for(int i=0; i < NUM_PULSES; i++) {
					WRITE(PHOTOGRAPH_PIN, HIGH);
					_delay_ms(PULSE_LENGTH);
					WRITE(PHOTOGRAPH_PIN, LOW);
					_delay_ms(PULSE_LENGTH);
					}
					delay(7.33);
					for(int i=0; i < NUM_PULSES; i++) {
					WRITE(PHOTOGRAPH_PIN, HIGH);
					_delay_ms(PULSE_LENGTH);
					WRITE(PHOTOGRAPH_PIN, LOW);
					_delay_ms(PULSE_LENGTH);
					}
					#endif
					#endif //chdk end if
					}
					break;
					#ifdef DOGLCD
					case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
					{
					if (code_seen('C')) {
					lcd_setcontrast( ((int)code_value())&63 );
					}
					SERIAL_PROTOCOLPGM("lcd contrast value: ");
					SERIAL_PROTOCOL(lcd_contrast);
					SERIAL_PROTOCOLLN("");
					}
					break;
					#endif
					#ifdef PREVENT_DANGEROUS_EXTRUDE
					case 302: // allow cold extrudes, or set the minimum extrude temperature
					{
					float temp = .0;
					if (code_seen('S')) temp=code_value();
					set_extrude_min_temp(temp);
					}
					break;
					#endif
					case 303: // M303 PID autotune
					{
					float temp = 150.0;
					int e=0;
					int c=5;
					if (code_seen('E')) e=code_value();
					if (e<0)
					temp=70;
					if (code_seen('S')) temp=code_value();
					if (code_seen('C')) c=code_value();
					PID_autotune(temp, e, c);
					}
					break;
					#ifdef SCARA
					case 360:  // M360 SCARA Theta pos1
					SERIAL_ECHOLN(" Cal: Theta 0 ");
					//SoftEndsEnabled = false;              // Ignore soft endstops during calibration
					//SERIAL_ECHOLN(" Soft endstops disabled ");
					if(Stopped == false) {
					//get_coordinates(); // For X Y Z E F
					delta[0] = 0;
					delta[1] = 120;
					calculate_SCARA_forward_Transform(delta);
					destination[0] = delta[0]/axis_scaling[X_AXIS];
					destination[1] = delta[1]/axis_scaling[Y_AXIS];
					
					prepare_move();
					//ClearToSend();
					return;
					}
					break;

					case 361:  // SCARA Theta pos2
					SERIAL_ECHOLN(" Cal: Theta 90 ");
					//SoftEndsEnabled = false;              // Ignore soft endstops during calibration
					//SERIAL_ECHOLN(" Soft endstops disabled ");
					if(Stopped == false) {
					//get_coordinates(); // For X Y Z E F
					delta[0] = 90;
					delta[1] = 130;
					calculate_SCARA_forward_Transform(delta);
					destination[0] = delta[0]/axis_scaling[X_AXIS];
					destination[1] = delta[1]/axis_scaling[Y_AXIS];
					
					prepare_move();
					//ClearToSend();
					return;
					}
					break;
					case 362:  // SCARA Psi pos1
					SERIAL_ECHOLN(" Cal: Psi 0 ");
					//SoftEndsEnabled = false;              // Ignore soft endstops during calibration
					//SERIAL_ECHOLN(" Soft endstops disabled ");
					if(Stopped == false) {
					//get_coordinates(); // For X Y Z E F
					delta[0] = 60;
					delta[1] = 180;
					calculate_SCARA_forward_Transform(delta);
					destination[0] = delta[0]/axis_scaling[X_AXIS];
					destination[1] = delta[1]/axis_scaling[Y_AXIS];
					
					prepare_move();
					//ClearToSend();
					return;
					}
					break;
					case 363:  // SCARA Psi pos2
					SERIAL_ECHOLN(" Cal: Psi 90 ");
					//SoftEndsEnabled = false;              // Ignore soft endstops during calibration
					//SERIAL_ECHOLN(" Soft endstops disabled ");
					if(Stopped == false) {
					//get_coordinates(); // For X Y Z E F
					delta[0] = 50;
					delta[1] = 90;
					calculate_SCARA_forward_Transform(delta);
					destination[0] = delta[0]/axis_scaling[X_AXIS];
					destination[1] = delta[1]/axis_scaling[Y_AXIS];
					
					prepare_move();
					//ClearToSend();
					return;
					}
					break;
					case 364:  // SCARA Psi pos3 (90 deg to Theta)
					SERIAL_ECHOLN(" Cal: Theta-Psi 90 ");
					// SoftEndsEnabled = false;              // Ignore soft endstops during calibration
					//SERIAL_ECHOLN(" Soft endstops disabled ");
					if(Stopped == false) {
					//get_coordinates(); // For X Y Z E F
					delta[0] = 45;
					delta[1] = 135;
					calculate_SCARA_forward_Transform(delta);
					destination[0] = delta[0]/axis_scaling[X_AXIS];
					destination[1] = delta[1]/axis_scaling[Y_AXIS];
					
					prepare_move();
					//ClearToSend();
					return;
					}
					break;
					case 365: // M364  Set SCARA scaling for X Y Z
					for(int8_t i=0; i < 3; i++)
					{
					if(code_seen(axis_codes[i]))
					{
					
					axis_scaling[i] = code_value();
					
					}
					}
					break;
					#endif
					case 400: // M400 finish all moves
					{
					st_synchronize();
					}
					break;
					#if defined(ENABLE_AUTO_BED_LEVELING) && defined(SERVO_ENDSTOPS) && not defined(Z_PROBE_SLED)
					case 401:
					{
					engage_z_probe();    // Engage Z Servo endstop if available
					}
					break;

					case 402:
					{
					retract_z_probe();    // Retract Z Servo endstop if enabled
					}
					break;
					#endif

					#ifdef FILAMENT_SENSOR
					case 404:  //M404 Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width
					{
					#if (FILWIDTH_PIN > -1)
					if(code_seen('N')) filament_width_nominal=code_value();
					else{
					SERIAL_PROTOCOLPGM("Filament dia (nominal mm):");
					SERIAL_PROTOCOLLN(filament_width_nominal);
					}
					#endif
					}
					break;
					
					case 405:  //M405 Turn on filament sensor for control
					{
					
					
					if(code_seen('D')) meas_delay_cm=code_value();
					
					if(meas_delay_cm> MAX_MEASUREMENT_DELAY)
					meas_delay_cm = MAX_MEASUREMENT_DELAY;
					
					if(delay_index2 == -1)  //initialize the ring buffer if it has not been done since startup
					{
					int temp_ratio = widthFil_to_size_ratio();
					
					for (delay_index1=0; delay_index1<(MAX_MEASUREMENT_DELAY+1); ++delay_index1 ){
					measurement_delay[delay_index1]=temp_ratio-100;  //subtract 100 to scale within a signed byte
					}
					delay_index1=0;
					delay_index2=0;
					}
					
					filament_sensor = true ;
					
					//SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
					//SERIAL_PROTOCOL(filament_width_meas);
					//SERIAL_PROTOCOLPGM("Extrusion ratio(%):");
					//SERIAL_PROTOCOL(extrudemultiply);
					}
					break;
					
					case 406:  //M406 Turn off filament sensor for control
					{
					filament_sensor = false ;
					}
					break;
					
					case 407:   //M407 Display measured filament diameter
					{
					
					
					
					SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
					SERIAL_PROTOCOLLN(filament_width_meas);
					}
					break;
					#endif

					case 500: // M500 Store settings in EEPROM
					{
					Config_StoreSettings();
					}
					break;
					case 501: // M501 Read settings from EEPROM
					{
					Config_RetrieveSettings();
					}
					break;
					case 502: // M502 Revert to default settings
					{
					Config_ResetDefault();
					}
					break;
					case 503: // M503 print settings currently in memory
					{
					Config_PrintSettings();
					}
					break;
					#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
					case 540:
					{
					if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
					}
					break;
					#endif

					#ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
					case CUSTOM_M_CODE_SET_Z_PROBE_OFFSET:
					{
					float value;
					if (code_seen('Z'))
					{
					value = code_value();
					if ((Z_PROBE_OFFSET_RANGE_MIN <= value) && (value <= Z_PROBE_OFFSET_RANGE_MAX))
					{
					zprobe_zoffset = -value; // compare w/ line 278 of ConfigurationStore.cpp
					SERIAL_ECHO_START;
					SERIAL_ECHOLNPGM(MSG_ZPROBE_ZOFFSET " " MSG_OK);
					SERIAL_PROTOCOLLN("");
					}
					else
					{
					SERIAL_ECHO_START;
					SERIAL_ECHOPGM(MSG_ZPROBE_ZOFFSET);
					SERIAL_ECHOPGM(MSG_Z_MIN);
					SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MIN);
					SERIAL_ECHOPGM(MSG_Z_MAX);
					SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MAX);
					SERIAL_PROTOCOLLN("");
					}
					}
					else
					{
					SERIAL_ECHO_START;
					SERIAL_ECHOLNPGM(MSG_ZPROBE_ZOFFSET " : ");
					SERIAL_ECHO(-zprobe_zoffset);
					SERIAL_PROTOCOLLN("");
					}
					break;
					}
					#endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET

					#ifdef FILAMENTCHANGEENABLE
					case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
					{
					float target[4];
					float lastpos[4];
					target[X_AXIS]=current_position[X_AXIS];
					target[Y_AXIS]=current_position[Y_AXIS];
					target[Z_AXIS]=current_position[Z_AXIS];
					target[E_AXIS]=current_position[E_AXIS];
					lastpos[X_AXIS]=current_position[X_AXIS];
					lastpos[Y_AXIS]=current_position[Y_AXIS];
					lastpos[Z_AXIS]=current_position[Z_AXIS];
					lastpos[E_AXIS]=current_position[E_AXIS];
					//retract by E
					if(code_seen('E'))
					{
					target[E_AXIS]+= code_value();
					}
					else
					{
					#ifdef FILAMENTCHANGE_FIRSTRETRACT
					target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT ;
					#endif
					}
					plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

					//lift Z
					if(code_seen('Z'))
					{
					target[Z_AXIS]+= code_value();
					}
					else
					{
					#ifdef FILAMENTCHANGE_ZADD
					target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
					#endif
					}
					plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

					//move xy
					if(code_seen('X'))
					{
					target[X_AXIS]+= code_value();
					}
					else
					{
					#ifdef FILAMENTCHANGE_XPOS
					target[X_AXIS]= FILAMENTCHANGE_XPOS ;
					#endif
					}
					if(code_seen('Y'))
					{
					target[Y_AXIS]= code_value();
					}
					else
					{
					#ifdef FILAMENTCHANGE_YPOS
					target[Y_AXIS]= FILAMENTCHANGE_YPOS ;
					#endif
					}

					plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

					if(code_seen('L'))
					{
					target[E_AXIS]+= code_value();
					}
					else
					{
					#ifdef FILAMENTCHANGE_FINALRETRACT
					target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT ;
					#endif
					}

					plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

					//finish moves
					st_synchronize();
					//disable extruder steppers so filament can be removed
					disable_e0();
					disable_e1();
					disable_e2();
					delay(100);
					LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
					uint8_t cnt=0;
					while(!lcd_clicked()){
					cnt++;
					manage_heater();
					manage_inactivity();
					lcd_update();
					if(cnt==0)
					{
					#if BEEPER > 0
					SET_OUTPUT(BEEPER);

					WRITE(BEEPER,HIGH);
					delay(3);
					WRITE(BEEPER,LOW);
					delay(3);
					#else
					#if !defined(LCD_FEEDBACK_FREQUENCY_HZ) || !defined(LCD_FEEDBACK_FREQUENCY_DURATION_MS)
					lcd_buzz(1000/6,100);
					#else
					lcd_buzz(LCD_FEEDBACK_FREQUENCY_DURATION_MS,LCD_FEEDBACK_FREQUENCY_HZ);
					#endif
					#endif
					}
					}

					//return to normal
					if(code_seen('L'))
					{
					target[E_AXIS]+= -code_value();
					}
					else
					{
					#ifdef FILAMENTCHANGE_FINALRETRACT
					target[E_AXIS]+=(-1)*FILAMENTCHANGE_FINALRETRACT ;
					#endif
					}
					current_position[E_AXIS]=target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
					plan_set_e_position(current_position[E_AXIS]);
					plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //should do nothing
					plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move xy back
					plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move z back
					plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //final untretract
					}
					break;
					#endif //FILAMENTCHANGEENABLE
					#ifdef DUAL_X_CARRIAGE
					case 605: // Set dual x-carriage movement mode:
					//    M605 S0: Full control mode. The slicer has full control over x-carriage movement
					//    M605 S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
					//    M605 S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
					//                         millimeters x-offset and an optional differential hotend temperature of
					//                         mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
					//                         the first with a spacing of 100mm in the x direction and 2 degrees hotter.
					//
					//    Note: the X axis should be homed after changing dual x-carriage mode.
					{
					st_synchronize();

					if (code_seen('S'))
					dual_x_carriage_mode = code_value();

					if (dual_x_carriage_mode == DXC_DUPLICATION_MODE)
					{
					if (code_seen('X'))
					duplicate_extruder_x_offset = max(code_value(),X2_MIN_POS - x_home_pos(0));

					if (code_seen('R'))
					duplicate_extruder_temp_offset = code_value();

					SERIAL_ECHO_START;
					SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
					SERIAL_ECHO(" ");
					SERIAL_ECHO(extruder_offset[X_AXIS][0]);
					SERIAL_ECHO(",");
					SERIAL_ECHO(extruder_offset[Y_AXIS][0]);
					SERIAL_ECHO(" ");
					SERIAL_ECHO(duplicate_extruder_x_offset);
					SERIAL_ECHO(",");
					SERIAL_ECHOLN(extruder_offset[Y_AXIS][1]);
					}
					else if (dual_x_carriage_mode != DXC_FULL_CONTROL_MODE && dual_x_carriage_mode != DXC_AUTO_PARK_MODE)
					{
					dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
					}

					active_extruder_parked = false;
					extruder_duplication_enabled = false;
					delayed_move_time = 0;
					}
					break;
					#endif //DUAL_X_CARRIAGE

					case 907: // M907 Set digital trimpot motor current using axis codes.
					{
					#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
					for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
					if(code_seen('B')) digipot_current(4,code_value());
					if(code_seen('S')) for(int i=0;i<=4;i++) digipot_current(i,code_value());
					#endif
					#ifdef MOTOR_CURRENT_PWM_XY_PIN
					if(code_seen('X')) digipot_current(0, code_value());
					#endif
					#ifdef MOTOR_CURRENT_PWM_Z_PIN
					if(code_seen('Z')) digipot_current(1, code_value());
					#endif
					#ifdef MOTOR_CURRENT_PWM_E_PIN
					if(code_seen('E')) digipot_current(2, code_value());
					#endif
					#ifdef DIGIPOT_I2C
					// this one uses actual amps in floating point
					for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_i2c_set_current(i, code_value());
					// for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
					for(int i=NUM_AXIS;i<DIGIPOT_I2C_NUM_CHANNELS;i++) if(code_seen('B'+i-NUM_AXIS)) digipot_i2c_set_current(i, code_value());
					#endif
					}
					break;
					case 908: // M908 Control digital trimpot directly.
					{
					#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
					uint8_t channel,current;
					if(code_seen('P')) channel=code_value();
					if(code_seen('S')) current=code_value();
					digitalPotWrite(channel, current);
					#endif
					}
					break;
					case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
					{
					#if defined(X_MS1_PIN) && X_MS1_PIN > -1
					if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value());
					for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
					if(code_seen('B')) microstep_mode(4,code_value());
					microstep_readings();
					#endif
					}
					break;
					case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
					{
						#if defined(X_MS1_PIN) && X_MS1_PIN > -1
							if(code_seen('S')) switch((int)code_value()){
								case 1:
								for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1);
								if(code_seen('B')) microstep_ms(4,code_value(),-1);
								break;
								case 2:
								for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value());
								if(code_seen('B')) microstep_ms(4,-1,code_value());
								break;
							}
							microstep_readings();
						#endif
					}
					break;
					case 352:{
						quick_guide = true;
						Config_StoreSettings();
					}
					break;
					case 999: // M999: Restart after being stopped
					Stopped = false;
					lcd_reset_alert_level();
					gcode_LastN = Stopped_gcode_LastN;
					FlushSerialRequestResend();
					break;
					}
					}

					else if(code_seen('T'))
					{
					tmp_extruder = code_value();
					if(tmp_extruder >= EXTRUDERS) {
						SERIAL_ECHO_START;
						SERIAL_ECHO("T");
						SERIAL_ECHO(tmp_extruder);
						SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
					}
					else {
						boolean make_move = false;
						if(code_seen('F')) {
						make_move = true;
						next_feedrate = code_value();
						if(next_feedrate > 0.0) {
							feedrate = next_feedrate;
						}
					}
					#if EXTRUDERS > 1
					if(tmp_extruder != active_extruder) {
					// Save current position to return to after applying extruder offset
					//Rapduch toolchange
						z_restaurada = current_position[Z_AXIS]; //Save the Z position that will be restored after changing tool
						memcpy(destination, current_position, sizeof(destination));
						#ifdef DUAL_X_CARRIAGE
						if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE && Stopped == false && (delayed_move_time != 0 || current_position[X_AXIS] != x_home_pos(active_extruder)))
						{
							// Park old head: 1) raise 2) move to park position 3) lower
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT, current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
							plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT, current_position[E_AXIS], max_feedrate[X_AXIS], active_extruder);
							plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
							st_synchronize();					
						}
					
						if (dual_x_carriage_mode == DXC_FULL_SIGMA_MODE && current_position[X_AXIS] != x_home_pos(active_extruder)) //DUAL FULL CONTROL and NOT HOMED
						{
							// Park old head: 1) raise 2) move to park position 3) lower
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,current_position[E_AXIS], max_feedrate[Z_AXIS]/2, active_extruder);
							plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,current_position[E_AXIS], XY_SIGMA_TRAVEL_SPEED, active_extruder);
							plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS] ,current_position[E_AXIS], max_feedrate[Z_AXIS]/2, active_extruder);
							st_synchronize();
							//current_position[Z_AXIS]= current_position[Z_AXIS]+TOOLCHANGE_PARK_ZLIFT;
						}
						Serial.print("POSITION HOME: ");
						Serial.println(x_home_pos(active_extruder));

						// apply Y & Z extruder offset (x offset is already used in determining home pos)
						//current_position[Y_AXIS] = current_position[Y_AXIS] - extruder_offset[Y_AXIS][active_extruder] + extruder_offset[Y_AXIS][tmp_extruder];
						//current_position[Z_AXIS] = current_position[Z_AXIS] - extruder_offset[Z_AXIS][active_extruder] + extruder_offset[Z_AXIS][tmp_extruder];
						plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS]- extruder_offset[Y_AXIS][active_extruder] + extruder_offset[Y_AXIS][tmp_extruder], current_position[Z_AXIS] - extruder_offset[Z_AXIS][tmp_extruder] + extruder_offset[Z_AXIS][active_extruder], current_position[E_AXIS], max_feedrate[X_AXIS], active_extruder);
						active_extruder = tmp_extruder;

						// This function resets the max/min values - the current position may be overwritten below.
						axis_is_at_home(X_AXIS);

						//Rapduch
						if (dual_x_carriage_mode == DXC_FULL_SIGMA_MODE)
						{
							memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
							active_extruder_parked = true;
						}					
						else if (dual_x_carriage_mode == DXC_FULL_CONTROL_MODE)
						{
							current_position[X_AXIS] = inactive_extruder_x_pos;
							inactive_extruder_x_pos = destination[X_AXIS];
						}
						else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE)
						{
							active_extruder_parked = (active_extruder == 0); // this triggers the second extruder to move into the duplication position
							if (active_extruder == 0 || active_extruder_parked) current_position[X_AXIS] = inactive_extruder_x_pos;
							else current_position[X_AXIS] = destination[X_AXIS] + duplicate_extruder_x_offset;
							inactive_extruder_x_pos = destination[X_AXIS];
							extruder_duplication_enabled = false;
						}
						else
						{
							// record raised toolhead position for use by unpark
							memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
							raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
							active_extruder_parked = true;
							delayed_move_time = 0;
						}
					#else
						// Offset extruder (only by XY)
						int i;
						for(i = 0; i < 2; i++) {
							current_position[i] = current_position[i] -
							extruder_offset[i][active_extruder] +
							extruder_offset[i][tmp_extruder];
						}
						// Set the new active extruder and position
						active_extruder = tmp_extruder;
					#endif //else DUAL_X_CARRIAGE

					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					//Rapduch purge
					//Maybe we could purge here
					//Purge line
					//current_position[E_AXIS]+=10;
					//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], (current_position[E_AXIS]),600/60,active_extruder);
					
					
					// Move to the old position if 'F' was in the parameters
					if(make_move && Stopped == false) {
					prepare_move();
					}
					}
					#endif
					SERIAL_ECHO_START;
					SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
					SERIAL_PROTOCOLLN((int)active_extruder);
					}
					}

					else
					{
					SERIAL_ECHO_START;
					SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
					SERIAL_ECHO(cmdbuffer[bufindr]);
					SERIAL_ECHOLNPGM("\"");
					}

					ClearToSend();
					}


					//Rapduch: Better hardcoded toolChange
					#ifdef DUAL_X_CARRIAGE
					void changeToolSigma(int tool){
					tmp_extruder = tool;
					z_restaurada = current_position[Z_AXIS]; //Save the Z position that will be restored after changing tool
					memcpy(destination, current_position, sizeof(destination));
					
					if (dual_x_carriage_mode == DXC_FULL_SIGMA_MODE && current_position[X_AXIS] != x_home_pos(active_extruder)) //DUAL FULL CONTROL and NOT HOMED
					{
					// Park old head: 1) raise 2) move to park position
					// No need to lower the Z axis, the new tool has to move to position before lowering Z
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,current_position[E_AXIS], max_feedrate[Z_AXIS]/2, active_extruder);
					plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,current_position[E_AXIS], max_feedrate[X_AXIS]/2, active_extruder);
					st_synchronize();
					current_position[Z_AXIS]= current_position[Z_AXIS]+TOOLCHANGE_PARK_ZLIFT;
					}
					
					// apply Y & Z extruder offset (x offset is already used in determining home pos)
					current_position[Y_AXIS] = current_position[Y_AXIS] -
					extruder_offset[Y_AXIS][active_extruder] +
					extruder_offset[Y_AXIS][tmp_extruder];
					current_position[Z_AXIS] = current_position[Z_AXIS] -
					extruder_offset[Z_AXIS][active_extruder] +
					extruder_offset[Z_AXIS][tmp_extruder];

					active_extruder = tmp_extruder;

					// This function resets the max/min values - the current position may be overwritten below.
					axis_is_at_home(X_AXIS);

					//Rapduch
					if (dual_x_carriage_mode == DXC_FULL_SIGMA_MODE)
					{
					memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
					active_extruder_parked = true;
					}
					
					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					//Rapduch purge
					//Maybe we could purge here
					
					SERIAL_ECHO_START;
					SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
					SERIAL_PROTOCOLLN((int)active_extruder);
					}
					#endif



					//Rapduch
					//Hardcoded  function for tool change
					#ifdef  DUAL_X_CARRIAGE
					void changeTool(int ntool) { //ntool select the tool that will be active
						tmp_extruder = ntool;
						if(tmp_extruder >= EXTRUDERS) {
							SERIAL_ECHO_START;
							SERIAL_ECHO("T");
							SERIAL_ECHO(tmp_extruder);
							SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
						}
						else {
							boolean make_move = false;
							//if(code_seen('F')) {
							//make_move = true;
							//next_feedrate = code_value();
							//if(next_feedrate > 0.0) {
							//feedrate = next_feedrate;
							//}
							//}
							//#if EXTRUDERS > 1
							if(tmp_extruder != active_extruder) {
								// Save current position to return to after applying extruder offset
								memcpy(destination, current_position, sizeof(destination));
								//#ifdef DUAL_X_CARRIAGE
								//if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE && Stopped == false &&
								//(delayed_move_time != 0 || current_position[X_AXIS] != x_home_pos(active_extruder)))
								//{
								
								/*
								// Park old head: 1) raise 2) move to park position 3) lower
								plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT+5,
								current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
								plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT+5,
								current_position[E_AXIS], max_feedrate[X_AXIS], active_extruder);
								plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS],
								current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
								st_synchronize();
								//}
								*/
								
								// apply Y & Z extruder offset (x offset is already used in determining home pos)
								/*current_position[Y_AXIS] = current_position[Y_AXIS] -
								extruder_offset[Y_AXIS][active_extruder] +
								extruder_offset[Y_AXIS][tmp_extruder];
								current_position[Z_AXIS] = current_position[Z_AXIS] -
								extruder_offset[Z_AXIS][active_extruder] +
								extruder_offset[Z_AXIS][tmp_extruder];*/
								
								plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS]- extruder_offset[Y_AXIS][active_extruder] + extruder_offset[Y_AXIS][tmp_extruder], current_position[Z_AXIS] - extruder_offset[Z_AXIS][tmp_extruder] + extruder_offset[Z_AXIS][active_extruder], current_position[E_AXIS], max_feedrate[X_AXIS], active_extruder);
								active_extruder = tmp_extruder;
								//current_position[Y_AXIS] =  current_position[Y_AXIS]- extruder_offset[Y_AXIS][active_extruder] + extruder_offset[Y_AXIS][tmp_extruder];
								// This function resets the max/min values - the current position may be overwritten below.
								axis_is_at_home(X_AXIS);

								//if (dual_x_carriage_mode == DXC_FULL_CONTROL_MODE)
								//{
								//current_position[X_AXIS] = inactive_extruder_x_pos;
								//inactive_extruder_x_pos = destination[X_AXIS];
								//}
								//else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE)
								//{
								//active_extruder_parked = (active_extruder == 0); // this triggers the second extruder to move into the duplication position
								//if (active_extruder == 0 || active_extruder_parked)
								//current_position[X_AXIS] = inactive_extruder_x_pos;
								//else
								//current_position[X_AXIS] = destination[X_AXIS] + duplicate_extruder_x_offset;
								//inactive_extruder_x_pos = destination[X_AXIS];
								//extruder_duplication_enabled = false;
								//}
								//else
								//{
								// record raised toolhead position for use by unpark
								memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
								raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT+5;
								active_extruder_parked = true;
								delayed_move_time = 0;	
								//}
								//#else
								//// Offset extruder (only by XY)
								//int i;
								//for(i = 0; i < 2; i++) {
								//current_position[i] = current_position[i] -
								//extruder_offset[i][active_extruder] +
								//extruder_offset[i][tmp_extruder];
								//}
								//// Set the new active extruder and position
								//active_extruder = tmp_extruder;
								//#endif //else DUAL_X_CARRIAGE
								//#ifdef DELTA
								//
								//calculate_delta(current_position); // change cartesian kinematic  to  delta kinematic;
								////sent position to plan_set_position();
								//plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],current_position[E_AXIS]);
								//
								//#else
								plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
								//Rapduch purge
								//Maybe we could purge here
								//Purge line
								//current_position[E_AXIS]+=10;
								//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], (current_position[E_AXIS]),600/60,active_extruder);
					
								//#endif
								//// Move to the old position if 'F' was in the parameters
								//if(make_move && Stopped == false) {
								//prepare_move();
								//}
							}
						//#endif
						SERIAL_ECHO_START;
						SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
						SERIAL_PROTOCOLLN((int)active_extruder);
						}
					}
					#endif //DUAL X CARRIAGE


					void FlushSerialRequestResend()
					{
					//char cmdbuffer[bufindr][100]="Resend:";
					MYSERIAL.flush();
					SERIAL_PROTOCOLPGM(MSG_RESEND);
					SERIAL_PROTOCOLLN(gcode_LastN + 1);
					ClearToSend();
					}

					void ClearToSend()
					{
					previous_millis_cmd = millis();
					#ifdef SDSUPPORT
					if(fromsd[bufindr])
					return;
					#endif //SDSUPPORT
					SERIAL_PROTOCOLLNPGM(MSG_OK);
					}

					void get_coordinates()
					{
						bool seen[4]={false,false,false,false};
						for(int8_t i=0; i < NUM_AXIS; i++) {
						if(code_seen(axis_codes[i]))
						{
					
							destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
							if(i == 0) {
								//Serial.print("X old: ");
								//Serial.println(destination[i]);
								destination[i]+= 48.5;
								//Serial.print("X new: ");
								//Serial.println(destination[i]);
							}
							seen[i]=true;
						}
						else destination[i] = current_position[i]; //Are these else lines really needed?
					}
					if(code_seen('F')) {
						next_feedrate = code_value();
						if(next_feedrate > 0.0) feedrate = next_feedrate;
					}
					}

					void get_arc_coordinates()
					{
					#ifdef SF_ARC_FIX
					bool relative_mode_backup = relative_mode;
					relative_mode = true;
					#endif
					get_coordinates();
					#ifdef SF_ARC_FIX
					relative_mode=relative_mode_backup;
					#endif

					if(code_seen('I')) {
					offset[0] = code_value();
					}
					else {
					offset[0] = 0.0;
					}
					if(code_seen('J')) {
					offset[1] = code_value();
					}
					else {
					offset[1] = 0.0;
					}
					}

					void clamp_to_software_endstops(float target[3])
					{
					if (min_software_endstops) {
					if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
					if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
					if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
					}

					if (max_software_endstops) {
					if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
					if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
					if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
					}
					}

					#ifdef DELTA
					void recalc_delta_settings(float radius, float diagonal_rod)
					{
					delta_tower1_x= -SIN_60*radius; // front left tower
					delta_tower1_y= -COS_60*radius;
					delta_tower2_x=  SIN_60*radius; // front right tower
					delta_tower2_y= -COS_60*radius;
					delta_tower3_x= 0.0;                  // back middle tower
					delta_tower3_y= radius;
					delta_diagonal_rod_2= sq(diagonal_rod);
					}

					void calculate_delta(float cartesian[3])
					{
					delta[X_AXIS] = sqrt(delta_diagonal_rod_2
					- sq(delta_tower1_x-cartesian[X_AXIS])
					- sq(delta_tower1_y-cartesian[Y_AXIS])
					) + cartesian[Z_AXIS];
					delta[Y_AXIS] = sqrt(delta_diagonal_rod_2
					- sq(delta_tower2_x-cartesian[X_AXIS])
					- sq(delta_tower2_y-cartesian[Y_AXIS])
					) + cartesian[Z_AXIS];
					delta[Z_AXIS] = sqrt(delta_diagonal_rod_2
					- sq(delta_tower3_x-cartesian[X_AXIS])
					- sq(delta_tower3_y-cartesian[Y_AXIS])
					) + cartesian[Z_AXIS];
					/*
					SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
					SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
					SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);

					SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
					SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
					SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
					*/
					}
					#endif

					void prepare_move()
					{
						clamp_to_software_endstops(destination);
						previous_millis_cmd = millis();
					
						#ifdef DUAL_X_CARRIAGE
							if (active_extruder_parked)//We recently have done a toolChange
							{
								//Rapduch: movements if whe have done a toolchange. It returns the new tool to the last printing position
								if (dual_x_carriage_mode == DXC_FULL_SIGMA_MODE && current_position[E_AXIS]==destination[E_AXIS]) //Sigma mode and we are in a travel move after toolchange
								{
									plan_buffer_line(current_position[X_AXIS], destination[Y_AXIS], raised_parked_position[Z_AXIS],current_position[E_AXIS], min(max_feedrate[X_AXIS],max_feedrate[Y_AXIS]/2), active_extruder);
									plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], raised_parked_position[Z_AXIS],current_position[E_AXIS], min(max_feedrate[X_AXIS],max_feedrate[Y_AXIS]/2), active_extruder);
									current_position[X_AXIS]=destination[X_AXIS];
									current_position[Y_AXIS]=destination[Y_AXIS];
									if (current_position[Z_AXIS]==destination[Z_AXIS]) //Correct the offset
									{
										plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], z_restaurada, current_position[E_AXIS], min(max_feedrate[X_AXIS],max_feedrate[Y_AXIS]/2), active_extruder);
										destination[Z_AXIS]=z_restaurada;
									}
									st_synchronize();
					
									active_extruder_parked = false;
								}			
					
								if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0)
								{
									// move duplicate extruder into correct duplication position.
									plan_set_position(inactive_extruder_x_pos, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
									plan_buffer_line(current_position[X_AXIS] + duplicate_extruder_x_offset, current_position[Y_AXIS], current_position[Z_AXIS],
									current_position[E_AXIS], max_feedrate[X_AXIS], 1);
									plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
									st_synchronize();
									extruder_duplication_enabled = true;
									active_extruder_parked = false;
								}
								else if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE) // handle unparking of head
								{
									if (current_position[E_AXIS] == destination[E_AXIS])
									{
										// this is a travel move - skit it but keep track of current position (so that it can later
										// be used as start of first non-travel move)
										if (delayed_move_time != 0xFFFFFFFFUL)
										{
											memcpy(current_position, destination, sizeof(current_position));
											if (destination[Z_AXIS] > raised_parked_position[Z_AXIS]) raised_parked_position[Z_AXIS] = destination[Z_AXIS];
											delayed_move_time = millis();
											return;
										}
									}
									delayed_move_time = 0;
									// unpark extruder: 1) raise, 2) move into starting XY position, 3) lower
									plan_buffer_line(raised_parked_position[X_AXIS], raised_parked_position[Y_AXIS], raised_parked_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
									plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], raised_parked_position[Z_AXIS], current_position[E_AXIS], min(max_feedrate[X_AXIS],max_feedrate[Y_AXIS]), active_extruder);
									plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
									active_extruder_parked = false;
								}
							}
						#endif //DUAL_X_CARRIAGE

						//It is not safe now. But the idea is to add an offset here to print always inside the bed
						//current_position[X_AXIS]+=54;

						//ACTUAL MOVE Prepared
						// Do not use feedmultiply for E or Z only moves
						if((current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
							plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
						}
						else {
							plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
						}

						for(int8_t i=0; i < NUM_AXIS; i++) {
							current_position[i] = destination[i];
						}
					}
					
					void prepare_arc_move(char isclockwise) {
					float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

					// Trace the arc
					mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

					// As far as the parser is concerned, the position is now == target. In reality the
					// motion control system might still be processing the action and the real tool position
					// in any intermediate location.
					for(int8_t i=0; i < NUM_AXIS; i++) {
					current_position[i] = destination[i];
					}
					previous_millis_cmd = millis();
					}

					#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1

					#if defined(FAN_PIN)
					#if CONTROLLERFAN_PIN == FAN_PIN
					#error "You cannot set CONTROLLERFAN_PIN equal to FAN_PIN"
					#endif
					#endif

					unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
					unsigned long lastMotorCheck = 0;

					void controllerFan()
					{
					if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
					{
					lastMotorCheck = millis();

					if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN) || (soft_pwm_bed > 0)
					#if EXTRUDERS > 2
					|| !READ(E2_ENABLE_PIN)
					#endif
					#if EXTRUDER > 1
					#if defined(X2_ENABLE_PIN) && X2_ENABLE_PIN > -1
					|| !READ(X2_ENABLE_PIN)
					#endif
					|| !READ(E1_ENABLE_PIN)
					#endif
					|| !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...
					{
					lastMotor = millis(); //... set time to NOW so the fan will turn on
					}

					if ((millis() - lastMotor) >= (CONTROLLERFAN_SECS*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...
					{
					digitalWrite(CONTROLLERFAN_PIN, 0);
					analogWrite(CONTROLLERFAN_PIN, 0);
					}
					else
					{
					// allows digital or PWM fan output to be used (see M42 handling)
					digitalWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
					analogWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
					}
					}
					}
					#endif

					#ifdef SCARA
					void calculate_SCARA_forward_Transform(float f_scara[3])
					{
					// Perform forward kinematics, and place results in delta[3]
					// The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014
					
					float x_sin, x_cos, y_sin, y_cos;
					
					//SERIAL_ECHOPGM("f_delta x="); SERIAL_ECHO(f_scara[X_AXIS]);
					//SERIAL_ECHOPGM(" y="); SERIAL_ECHO(f_scara[Y_AXIS]);
					
					x_sin = sin(f_scara[X_AXIS]/SCARA_RAD2DEG) * Linkage_1;
					x_cos = cos(f_scara[X_AXIS]/SCARA_RAD2DEG) * Linkage_1;
					y_sin = sin(f_scara[Y_AXIS]/SCARA_RAD2DEG) * Linkage_2;
					y_cos = cos(f_scara[Y_AXIS]/SCARA_RAD2DEG) * Linkage_2;
					
					//  SERIAL_ECHOPGM(" x_sin="); SERIAL_ECHO(x_sin);
					//  SERIAL_ECHOPGM(" x_cos="); SERIAL_ECHO(x_cos);
					//  SERIAL_ECHOPGM(" y_sin="); SERIAL_ECHO(y_sin);
					//  SERIAL_ECHOPGM(" y_cos="); SERIAL_ECHOLN(y_cos);
					
					delta[X_AXIS] = x_cos + y_cos + SCARA_offset_x;  //theta
					delta[Y_AXIS] = x_sin + y_sin + SCARA_offset_y;  //theta+phi
					
					//SERIAL_ECHOPGM(" delta[X_AXIS]="); SERIAL_ECHO(delta[X_AXIS]);
					//SERIAL_ECHOPGM(" delta[Y_AXIS]="); SERIAL_ECHOLN(delta[Y_AXIS]);
					}

					void calculate_delta(float cartesian[3]){
					//reverse kinematics.
					// Perform reversed kinematics, and place results in delta[3]
					// The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014
					
					float SCARA_pos[2];
					static float SCARA_C2, SCARA_S2, SCARA_K1, SCARA_K2, SCARA_theta, SCARA_psi;
					
					SCARA_pos[X_AXIS] = cartesian[X_AXIS] * axis_scaling[X_AXIS] - SCARA_offset_x;  //Translate SCARA to standard X Y
					SCARA_pos[Y_AXIS] = cartesian[Y_AXIS] * axis_scaling[Y_AXIS] - SCARA_offset_y;  // With scaling factor.
					
					#if (Linkage_1 == Linkage_2)
					SCARA_C2 = ( ( sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) ) / (2 * (float)L1_2) ) - 1;
					#else
					SCARA_C2 =   ( sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) - (float)L1_2 - (float)L2_2 ) / 45000;
					#endif
					
					SCARA_S2 = sqrt( 1 - sq(SCARA_C2) );
					
					SCARA_K1 = Linkage_1 + Linkage_2 * SCARA_C2;
					SCARA_K2 = Linkage_2 * SCARA_S2;
					
					SCARA_theta = ( atan2(SCARA_pos[X_AXIS],SCARA_pos[Y_AXIS])-atan2(SCARA_K1, SCARA_K2) ) * -1;
					SCARA_psi   =   atan2(SCARA_S2,SCARA_C2);
					
					delta[X_AXIS] = SCARA_theta * SCARA_RAD2DEG;  // Multiply by 180/Pi  -  theta is support arm angle
					delta[Y_AXIS] = (SCARA_theta + SCARA_psi) * SCARA_RAD2DEG;  //       -  equal to sub arm angle (inverted motor)
					delta[Z_AXIS] = cartesian[Z_AXIS];
					
					/*
					SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
					SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
					SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);
					
					SERIAL_ECHOPGM("scara x="); SERIAL_ECHO(SCARA_pos[X_AXIS]);
					SERIAL_ECHOPGM(" y="); SERIAL_ECHOLN(SCARA_pos[Y_AXIS]);
					
					SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
					SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
					SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
					
					SERIAL_ECHOPGM("C2="); SERIAL_ECHO(SCARA_C2);
					SERIAL_ECHOPGM(" S2="); SERIAL_ECHO(SCARA_S2);
					SERIAL_ECHOPGM(" Theta="); SERIAL_ECHO(SCARA_theta);
					SERIAL_ECHOPGM(" Psi="); SERIAL_ECHOLN(SCARA_psi);
					SERIAL_ECHOLN(" ");*/
					}

					#endif

					#ifdef TEMP_STAT_LEDS
					static bool blue_led = false;
					static bool red_led = false;
					static uint32_t stat_update = 0;

					void handle_status_leds(void) {
					float max_temp = 0.0;
					if(millis() > stat_update) {
					stat_update += 500; // Update every 0.5s
					for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
					max_temp = max(max_temp, degHotend(cur_extruder));
					max_temp = max(max_temp, degTargetHotend(cur_extruder));
					}
					#if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
					max_temp = max(max_temp, degTargetBed());
					max_temp = max(max_temp, degBed());
					#endif
					if((max_temp > 55.0) && (red_led == false)) {
					digitalWrite(STAT_LED_RED, 1);
					digitalWrite(STAT_LED_BLUE, 0);
					red_led = true;
					blue_led = false;
					}
					if((max_temp < 54.0) && (blue_led == false)) {
					digitalWrite(STAT_LED_RED, 0);
					digitalWrite(STAT_LED_BLUE, 1);
					red_led = false;
					blue_led = true;
					}
					}
					}
					#endif

					void manage_inactivity()
					{
						if(buflen < (BUFSIZE-1)) get_command();
						if( (millis() - previous_millis_cmd) >  max_inactive_time ) if(max_inactive_time) kill();
						if(stepper_inactive_time)  {
							if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
							{
								if(blocks_queued() == false) {
									disable_x();
									disable_y();
									disable_z();
									disable_e0();
									disable_e1();
									disable_e2();
								}
							}
						}
					
						#ifdef CHDK //Check if pin should be set to LOW after M240 set it to HIGH
							if (chdkActive && (millis() - chdkHigh > CHDK_DELAY))
							{
								chdkActive = false;
								WRITE(CHDK, LOW);
							}
						#endif
					
						#if defined(KILL_PIN) && KILL_PIN > -1
							if( 0 == READ(KILL_PIN) ) kill();
						#endif
						#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
							controllerFan(); //Check if fan should be turned on to cool stepper drivers down
						#endif
						#ifdef EXTRUDER_RUNOUT_PREVENT
							if( (millis() - previous_millis_cmd) >  EXTRUDER_RUNOUT_SECONDS*1000 ) 
								if(degHotend(active_extruder)>EXTRUDER_RUNOUT_MINTEMP)
								{
									bool oldstatus=READ(E0_ENABLE_PIN);
									enable_e0();
									float oldepos=current_position[E_AXIS];
									float oldedes=destination[E_AXIS];
									plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
									destination[E_AXIS]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS],
									EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], active_extruder);
									current_position[E_AXIS]=oldepos;
									destination[E_AXIS]=oldedes;
									plan_set_e_position(oldepos);
									previous_millis_cmd=millis();
									st_synchronize();
									WRITE(E0_ENABLE_PIN,oldstatus);
								}
						#endif
						#if defined(DUAL_X_CARRIAGE)
							// handle delayed move timeout
							if (delayed_move_time != 0 && (millis() - delayed_move_time) > 1000 && Stopped == false)
							{
								// travel moves have been received so enact them
								delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
								memcpy(destination,current_position,sizeof(destination));
								prepare_move();
							}
						#endif
						#ifdef TEMP_STAT_LEDS
							handle_status_leds();
						#endif
						check_axes_activity();
					}

					void kill()
					{
					cli(); // Stop interrupts
					disable_heater();

					disable_x();
					disable_y();
					disable_z();
					disable_e0();
					disable_e1();
					disable_e2();

					#if defined(PS_ON_PIN) && PS_ON_PIN > -1
					pinMode(PS_ON_PIN,INPUT);
					#endif
					SERIAL_ERROR_START;
					SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
					LCD_ALERTMESSAGEPGM(MSG_KILLED);
					suicide();
					while(1) { /* Intentionally left empty */ } // Wait for reset
					}

					void Stop()
					{
						disable_heater();
						if(Stopped == false) {
							Stopped = true;
							Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
							SERIAL_ERROR_START;
							SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
							LCD_MESSAGEPGM(MSG_STOPPED);
						}
					}

					bool IsStopped() { return Stopped; };

					#ifdef FAST_PWM_FAN
					void setPwmFrequency(uint8_t pin, int val)
					{
					val &= 0x07;
					switch(digitalPinToTimer(pin))
					{

					#if defined(TCCR0A)
					case TIMER0A:
					case TIMER0B:
					//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
					//         TCCR0B |= val;
					break;
					#endif

					#if defined(TCCR1A)
					case TIMER1A:
					case TIMER1B:
					//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
					//         TCCR1B |= val;
					break;
					#endif

					#if defined(TCCR2)
					case TIMER2:
					case TIMER2:
					TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
					TCCR2 |= val;
					break;
					#endif

					#if defined(TCCR2A)
					case TIMER2A:
					case TIMER2B:
					TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
					TCCR2B |= val;
					break;
					#endif

					#if defined(TCCR3A)
					case TIMER3A:
					case TIMER3B:
					case TIMER3C:
					TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
					TCCR3B |= val;
					break;
					#endif

					#if defined(TCCR4A)
					case TIMER4A:
					case TIMER4B:
					case TIMER4C:
					TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
					TCCR4B |= val;
					break;
					#endif

					#if defined(TCCR5A)
					case TIMER5A:
					case TIMER5B:
					case TIMER5C:
					TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
					TCCR5B |= val;
					break;
					#endif

					}
					}
					#endif //FAST_PWM_FAN

					bool setTargetedHotend(int code){
					tmp_extruder = active_extruder;
					if(code_seen('T')) {
					tmp_extruder = code_value();
					if(tmp_extruder >= EXTRUDERS) {
					SERIAL_ECHO_START;
					switch(code){
					case 104:
					SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
					break;
					case 105:
					SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
					break;
					case 109:
					SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
					break;
					case 218:
					SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
					break;
					case 221:
					SERIAL_ECHO(MSG_M221_INVALID_EXTRUDER);
					break;
					}
					SERIAL_ECHOLN(tmp_extruder);
					return true;
					}
					}
					return false;
					}
					

					void left_test_print_code(){
						////////////////////
						//LEFT Z TEST PRINT/
						////////////////////
						
						current_position[E_AXIS]+=15;  //0.5 + 0.15 per ajustar una bona alçada
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],INSERT_SLOW_SPEED/60,active_extruder);
						st_synchronize();
						current_position[Z_AXIS]+=0.65; //0.5 + 0.15 per ajustar una bona alçada
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						current_position[E_AXIS]-=4;  //0.5 + 0.15 per ajustar una bona alçada
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],INSERT_SLOW_SPEED/60,active_extruder);
						
						
						
						//SKIRT_v2
						//Positioning
						current_position[Y_AXIS] = 250+11+10; current_position[Z_AXIS] = 0.2;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 75;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[E_AXIS]+=4;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],INSERT_SLOW_SPEED/60,active_extruder);
						st_synchronize();
						//start print the skirt
						current_position[Y_AXIS] = 205; current_position[E_AXIS] += ((271-205)*0.33*current_position[Z_AXIS]*10/55.119)*1.5;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 110; current_position[E_AXIS] += ((110-75)*0.33*current_position[Z_AXIS]*10/55.119)*1.5;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						current_position[Y_AXIS] = 250+11+10; current_position[E_AXIS] += ((271-205)*0.33*current_position[Z_AXIS]*10/55.119)*1.5;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 75; current_position[E_AXIS] += ((110-75)*0.33*current_position[Z_AXIS]*10/55.119)*1.5;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						
						//LINE1 v2
						current_position[X_AXIS] = 80; current_position[Y_AXIS] = 250+11+5; current_position[Z_AXIS]= 0.25;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 80; current_position[Y_AXIS] = 210;  current_position[E_AXIS]+=((266-206)*0.33*0.1*10/55.119)*1.1;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						//LINE2 v2
						current_position[X_AXIS] = 86; current_position[Y_AXIS] = 250+11+5; current_position[Z_AXIS]= 0.2;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 86; current_position[Y_AXIS] = 210;  current_position[E_AXIS]+=((266-206)*0.33*0.1*10/55.119)*1.1;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						//LINE3 v2
						current_position[X_AXIS] = 92; current_position[Y_AXIS] = 250+11+5; current_position[Z_AXIS]= 0.15;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 92; current_position[Y_AXIS] = 210;  current_position[E_AXIS]+=((266-206)*0.33*0.1*10/55.119)*1.1;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						//LINE4 v2
						current_position[X_AXIS] = 98; current_position[Y_AXIS] = 250+11+5; current_position[Z_AXIS]= 0.1;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 98; current_position[Y_AXIS] = 210;  current_position[E_AXIS]+=((266-206)*0.33*0.1*10/55.119)*1.1;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						//LINE5 v2
						current_position[X_AXIS] = 104; current_position[Y_AXIS] = 250+11+5; current_position[Z_AXIS]= 0.05;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 104; current_position[Y_AXIS] = 210;  current_position[E_AXIS]+=((266-206)*0.33*0.1*10/55.119)*1.1;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						
						
						
						//RETIRE HOTEND
						current_position[E_AXIS]-=4;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],INSERT_FAST_SPEED,active_extruder);
						st_synchronize();
						current_position[Z_AXIS]+= 0.5; 
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						enquecommand_P(PSTR("G28 X Y"));
						st_synchronize();
									
						
						//SELECT LINES SCREEN						
						genie.WriteObject(GENIE_OBJ_FORM,FORM_LEFT_Z_TEST,0);
					}
					void right_test_print_code(){	
						
						current_position[E_AXIS]+=15; 
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],INSERT_SLOW_SPEED/60,active_extruder);
						st_synchronize();
						current_position[Z_AXIS]=0.2; //0.5 + 0.15 per ajustar una bona alçada
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],6000/60,active_extruder);
						st_synchronize();
						current_position[E_AXIS]-=4; //0.5 + 0.15 per ajustar una bona alçada
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],12000/60,active_extruder);
						st_synchronize();
						
						//SKIRT
						/*current_position[X_AXIS] = 80; current_position[Y_AXIS] = 149-5; 
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[E_AXIS]+=4;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],12000/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 80; current_position[Y_AXIS] = 250+11+5; current_position[E_AXIS] += ((266-144)*0.33*current_position[Z_AXIS]*10/55.119)*1.5;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 105; current_position[Y_AXIS] = 250+11+5; current_position[E_AXIS] += ((105-80)*0.33*current_position[Z_AXIS]*10/55.119)*1.5;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 105; current_position[Y_AXIS] = 149-5; current_position[E_AXIS] += ((266-144)*0.33*current_position[Z_AXIS]*10/55.119)*1.5;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 80; current_position[Y_AXIS] = 149-5; current_position[E_AXIS] += ((105-80)*0.33*current_position[Z_AXIS]*10/55.119)*1.5;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();	*/					
						
						//SKIRT v2
						current_position[Y_AXIS] = 200; 
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 75; 
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();	
						current_position[E_AXIS]+=4;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],INSERT_SLOW_SPEED/60,active_extruder);
						st_synchronize();	
						current_position[X_AXIS] = 110; current_position[E_AXIS] += ((110-75)*0.33*current_position[Z_AXIS]*10/55.119)*1.5;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						current_position[Y_AXIS] = 140; current_position[E_AXIS] += ((200-140)*0.33*current_position[Z_AXIS]*10/55.119)*1.5;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 75; current_position[E_AXIS] += ((110-75)*0.33*current_position[Z_AXIS]*10/55.119)*1.5;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						current_position[Y_AXIS] = 200; current_position[E_AXIS] += ((200-121)*0.33*current_position[Z_AXIS]*10/55.119)*1.5;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						
						//LINES
						//LINE 1
						current_position[X_AXIS] = 80; current_position[Y_AXIS] = 195; current_position[Z_AXIS]= 0.25;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[Y_AXIS] = 145; current_position[E_AXIS]+=((195-145)*0.33*0.1*10/55.119)*1.1;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						//LINE2
						current_position[X_AXIS] = 86; current_position[Y_AXIS] = 195; current_position[Z_AXIS]= 0.2;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[Y_AXIS] = 145; current_position[E_AXIS]+=((195-145)*0.33*0.1*10/55.119)*1.1;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						//LINE3
						current_position[X_AXIS] = 92; current_position[Y_AXIS] = 195; current_position[Z_AXIS]= 0.15;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[Y_AXIS] = 145; current_position[E_AXIS]+=((195-145)*0.33*0.1*10/55.119)*1.1;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						//LINE4
						current_position[X_AXIS] = 98; current_position[Y_AXIS] = 195; current_position[Z_AXIS]= 0.1;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[Y_AXIS] = 145; current_position[E_AXIS]+=((195-145)*0.33*0.1*10/55.119)*1.1;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						//LINE5
						current_position[X_AXIS] = 104; current_position[Y_AXIS] = 195; current_position[Z_AXIS]= 0.05;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[Y_AXIS] = 145; current_position[E_AXIS]+=((195-145)*0.33*0.1*10/55.119)*1.1;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
							
						/*								
						// LEFT LINE
						current_position[X_AXIS] = 85; current_position[Y_AXIS] = 149; current_position[Z_AXIS]= 0.15;// current_position[E_AXIS] -= 2;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 85; current_position[Y_AXIS] = 200; current_position[E_AXIS]+=((200-149)*0.33*0.1*10/55.119)*1.2;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						//CENTER LINE
						current_position[X_AXIS] = 92; current_position[Y_AXIS] = 149; current_position[Z_AXIS]= 0.1; //current_position[E_AXIS]-=2;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 92; current_position[Y_AXIS] = 200; current_position[E_AXIS]+=((200-149)*0.33*0.1*10/55.119)*1.2;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();
						// RIGHT LINE
						current_position[X_AXIS] = 100; current_position[Y_AXIS] = 149; current_position[Z_AXIS]= 0.05; //current_position[E_AXIS]-=2;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS] = 100; current_position[Y_AXIS] = 200; current_position[E_AXIS]+=((200-149)*0.33*0.1*10/55.119)*1.2;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						st_synchronize();*/
						
						//RETIRE HOTEND
						current_position[E_AXIS]-=4;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],INSERT_FAST_SPEED/60,active_extruder);
						st_synchronize();
						current_position[Z_AXIS]+= 0.3; //current_position[E_AXIS]-=4;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();
						current_position[X_AXIS]= X2_MAX_POS; //current_position[E_AXIS]-=4;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],3000/60,active_extruder);
						st_synchronize();						
						enquecommand_P(PSTR("G28 X Y"));
						st_synchronize();
						current_position[Z_AXIS]-= 0.3;
						plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS],1500/60,active_extruder);
						
						st_synchronize();						
						//SELECT LINES SCREEN
						z_print_test =false;
						genie.WriteObject(GENIE_OBJ_FORM,FORM_RIGHT_Z_TEST,0);
					}
					
					
					
					void home_axis_from_code()
					{
						#ifdef ENABLE_AUTO_BED_LEVELING
							plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)
						#endif //ENABLE_AUTO_BED_LEVELING

						#ifdef Z_SIGMA_HOME
							int saved_active_extruder = active_extruder;
							Serial.print("Extruder active: ");
							Serial.println(saved_active_extruder);
						#endif
					
						saved_feedrate = feedrate;
						saved_feedmultiply = feedmultiply;
						feedmultiply = 100;
						previous_millis_cmd = millis();

						enable_endstops(true); //Activate endstops

						for(int8_t i=0; i < NUM_AXIS; i++) {
							destination[i] = current_position[i];
						}
						feedrate = 0.0;

						home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])));

						#if Z_HOME_DIR > 0                      // If homing away from BED do Z first. WE ARE NOT USING THIS IN SIGMA
							if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
								HOMEAXIS(Z);
							}
						#endif

						#ifdef QUICK_HOME
							if((home_all_axis)||( code_seen(axis_codes[X_AXIS]) && code_seen(axis_codes[Y_AXIS])) )  //first diagonal move
							{
								current_position[X_AXIS] = 0;current_position[Y_AXIS] = 0;

								#ifndef DUAL_X_CARRIAGE
									int x_axis_home_dir = home_dir(X_AXIS); //Set X directions
								#else
									int x_axis_home_dir = x_home_dir(active_extruder);
									extruder_duplication_enabled = false;
								#endif

								plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					
								destination[X_AXIS] = 1.5 * max_length(X_AXIS) * x_axis_home_dir;
								destination[Y_AXIS] = 1.5 * max_length(Y_AXIS) * home_dir(Y_AXIS);
					
								feedrate = homing_feedrate[X_AXIS];
					
								if(homing_feedrate[Y_AXIS]<feedrate) feedrate = homing_feedrate[Y_AXIS];
					
								if (max_length(X_AXIS) > max_length(Y_AXIS)) {
									feedrate *= sqrt(pow(max_length(Y_AXIS) / max_length(X_AXIS), 2) + 1);
								} else {
									feedrate *= sqrt(pow(max_length(X_AXIS) / max_length(Y_AXIS), 2) + 1);
								}
					
								plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
								st_synchronize();

								axis_is_at_home(X_AXIS);
								axis_is_at_home(Y_AXIS);
					
								plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
								destination[X_AXIS] = current_position[X_AXIS];
								destination[Y_AXIS] = current_position[Y_AXIS];
								plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
								feedrate = 0.0;
								st_synchronize();
								endstops_hit_on_purpose();

								current_position[X_AXIS] = destination[X_AXIS];
								current_position[Y_AXIS] = destination[Y_AXIS];
								current_position[Z_AXIS] = destination[Z_AXIS];
							}
						#endif

						if((home_all_axis) || (code_seen(axis_codes[X_AXIS]))) // First do X
						{
							#ifdef DUAL_X_CARRIAGE
								#ifdef Z_SIGMA_HOME
									if(saved_active_extruder == RIGHT_EXTRUDER)
									{
										//active_extruder=LEFT_EXTRUDER; //Always use the same routine (left extruder probes)
									}
								#endif
								int tmp_extruder = active_extruder;
								extruder_duplication_enabled = false;
								active_extruder = !active_extruder;
								HOMEAXIS(X);
								inactive_extruder_x_pos = current_position[X_AXIS];
								active_extruder = tmp_extruder;
								HOMEAXIS(X);
								// reset state used by the different modes
								memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
								delayed_move_time = 0;
								active_extruder_parked = true;
							#else
								HOMEAXIS(X);
							#endif
						}

						if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
							HOMEAXIS(Y);
						}

						if(code_seen(axis_codes[X_AXIS]))
						{
							if(code_value_long() != 0) {
								current_position[X_AXIS]=code_value()+add_homing[0];
							}
						}

						if(code_seen(axis_codes[Y_AXIS])) {
							if(code_value_long() != 0) {
								current_position[Y_AXIS]=code_value()+add_homing[1];
							}
						}

						#if Z_HOME_DIR < 0   // If homing towards BED do Z last
							//Rapduch
							#ifdef Z_SIGMA_HOME
								if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
					
					
									if (saved_active_extruder == RIGHT_EXTRUDER){
										active_extruder=LEFT_EXTRUDER;
										axis_is_at_home(X_AXIS);
										plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
									}
					
									destination[X_AXIS] = round(Z_SIGMA_HOME_X_POINT-X_SIGMA_PROBE_OFFSET_FROM_EXTRUDER);
									destination[Y_AXIS] = round(Z_SIGMA_HOME_Y_POINT-Y_SIGMA_PROBE_OFFSET_FROM_EXTRUDER);
									destination[Z_AXIS] = Z_SIGMA_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
									feedrate = SIGMA_Z_HOME_TRAVEL_SPEED;
									current_position[Z_AXIS] = 0;
					
									plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
									plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);//Left Extruder
									st_synchronize();
									current_position[X_AXIS] = destination[X_AXIS];
									current_position[Y_AXIS] = destination[Y_AXIS];
									HOMEAXIS(Z);
					
									Serial.println("Z SIGMA Homed");
									//At this point our probe is homed, no offset is added.

									//Now lets home with the second extruder:
									//st_synchronize();
									//
									//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+10, current_position[E_AXIS], feedrate, 1);
									////plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS]+10, current_position[E_AXIS], feedrate, 1);
									//destination[X_AXIS] = round(Z_SIGMA_HOME_SECOND_X_POINT);
									//destination[Y_AXIS] = round(Z_SIGMA_HOME_SECOND_Y_POINT);
									//destination[Z_AXIS] = Z_SIGMA_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
									//feedrate = XY_SIGMA_TRAVEL_SPEED;
									//current_position[Z_AXIS] = 0;
									//SERIAL_ECHO("Z SIGMA Homed");
									//
									//plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
									//plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, tmp_extruder); //Second extruder(temporal)
									//current_position[X_AXIS] = destination[X_AXIS];
									//current_position[Y_AXIS] = destination[Y_AXIS];
									//HOMEAXIS(Z);
					
								}
							#else
					
								#ifndef Z_SAFE_HOMING
									if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
										#if defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
											destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
											feedrate = max_feedrate[Z_AXIS];
											plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
											st_synchronize();
										#endif
										HOMEAXIS(Z);
									}
								#else                   // Z Safe mode activated.
									if(home_all_axis) {
										destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
										destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
										destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
										feedrate = XY_TRAVEL_SPEED;
										current_position[Z_AXIS] = 0;

										plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
										plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
										st_synchronize();
										current_position[X_AXIS] = destination[X_AXIS];
										current_position[Y_AXIS] = destination[Y_AXIS];

										HOMEAXIS(Z);
									}
					
									//Let's see if X and Y are homed and probe is inside bed area.
									if(code_seen(axis_codes[Z_AXIS])) {
										if ( (axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]) \
										&& (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER >= X_MIN_POS) \
										&& (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER <= X_MAX_POS) \
										&& (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER >= Y_MIN_POS) \
										&& (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER <= Y_MAX_POS)){
					
											current_position[Z_AXIS] = 0;
											plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
											destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
											feedrate = max_feedrate[Z_AXIS];
											plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
											st_synchronize();

											HOMEAXIS(Z);
										} else if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
											LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
											SERIAL_ECHO_START;
											SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
										} else {
											LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
											SERIAL_ECHO_START;
											SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
										}
									}
								#endif
					
							#endif
					
						#endif

						if(code_seen(axis_codes[Z_AXIS])) { //Sets Z position
							if(code_value_long() != 0) {
								current_position[Z_AXIS]=code_value()+add_homing[2];
							}
						}
					
						//Rapduch
						#ifdef Z_SIGMA_HOME
							if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
								current_position[Z_AXIS] += zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)
							}
						#else
							#ifdef ENABLE_AUTO_BED_LEVELING  //ADDING the Z offset
								//if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
								//current_position[Z_AXIS] += zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)
								//}
							#endif
						#endif

						plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

						#ifdef ENDSTOPS_ONLY_FOR_HOMING
							enable_endstops(false);
						#endif

						feedrate = saved_feedrate;
						feedmultiply = saved_feedmultiply;
						previous_millis_cmd = millis();
						endstops_hit_on_purpose();

						//Rapduch
						#ifdef Z_SIGMA_HOME  //This to return the left extruder at Xhome position
							if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
					
							feedrate = homing_feedrate[Z_AXIS];
							current_position[Z_AXIS]+=Z_SIGMA_RAISE_AFTER_HOMING;
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
							current_position[X_AXIS] = x_home_pos(active_extruder);
							feedrate = homing_feedrate[X_AXIS];
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
							current_position[Z_AXIS] = 0;
							feedrate = homing_feedrate[Z_AXIS];
							plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
							st_synchronize();
							
								if(saved_active_extruder==RIGHT_EXTRUDER)
								{
									changeToolSigma(RIGHT_EXTRUDER); //Get again the same tool
									Serial.print("Extruder released active: ");
									Serial.println(saved_active_extruder);
								}
								////Activate Probing sequence
								//setup_for_endstop_move();
								//feedrate = homing_feedrate[Z_AXIS];
								//active_extruder=1; //extruder1(right) instead of extruder0(left)
								////tmp_extruder=active_extruder;
								//probe_pt(Z_SIGMA_HOME_SECOND_X_POINT,Z_SIGMA_HOME_SECOND_Y_POINT, Z_SIGMA_RAISE_BEFORE_HOMING);
								//clean_up_after_endstop_move();
							}
						#endif
						home_made = true;
					}

