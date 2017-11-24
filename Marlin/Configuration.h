#ifndef CONFIGURATION_H
#define CONFIGURATION_H


////////////////////////////////PROTOSIGMA///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Which variables has been affected in the firmware to DHUB machines
//SIGMA_Z_HOME_TRAVEL_SPEED , XY_SIGMA_TRAVEL_SPEED, HOMING_FEEDRATE, DEFAULT_AXIS_STEPS_PER_UNIT, DEFAULT_MAX_FEEDRATE, DEFAULT_MAX_ACCELERATION, X_MAX_POS   
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This configuration file contains the basic settings.
// Advanced settings can be found in Configuration_adv.h
// BASIC SETTINGS: select your board type, temperature sensor type, axis scaling, and endstop configuration

//===========================================================================
//============================= DELTA Printer ===============================
//===========================================================================
// For a Delta printer replace the configuration files with the files in the
// example_configurations/delta directory.
//

//===========================================================================
//============================= SCARA Printer ===============================
//===========================================================================
// For a Delta printer replace the configuration files with the files in the
// example_configurations/SCARA directory.
//
//This is the version declaration for Sigma, v followed by '-' first indicate the hardware, it must have 2 ditgits. Then the '-' and then the firmware, it has to have 3 digits separets by '.'. -> This is useful to
//get the hw and fw version to Cura-BCN3D and update the new firmware

#define VERSION_STRING  "01-1.2.7RC"
#define BUILD_DATE  "|M11.24"
#define VERSION_NUMBER  127
//#define BUILD_DATE  " "
#define UI_SerialID  "At Bottom Sticker"
//#define DEFAULT_QUICK_GUIDE  0;
// User-specified version info of this build to display in [Pronterface, etc] terminal window during
// startup. Implementation of an idea by Prof Braino to inform user that any changes made to this
// build by the user have been successfully uploaded into firmware.
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ // build date and time
#define STRING_CONFIG_H_AUTHOR "(none, default config)" // Who made the changes.

// SERIAL_PORT selects which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
#define SERIAL_PORT 0

// This determines the communication speed of the printer
#define BAUDRATE 250000

// This enables the serial port associated to the Bluetooth interface
//#define BTENABLED              // Enable BT interface on AT90USB devices


//// The following define selects which electronics board you have. Please choose the one that matches your setup
// 1  = RepRapBCN Electronics v0
// 10 = Gen7 custom (Alfons3 Version) "https://github.com/Alfons3/Generation_7_Electronics"
// 11 = Gen7 v1.1, v1.2 = 11
// 12 = Gen7 v1.3
// 13 = Gen7 v1.4
// 2  = Cheaptronic v1.0
// 20 = Sethi 3D_1
// 3  = MEGA/RAMPS up to 1.2 = 3
// 33 = RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Bed)
// 34 = RAMPS 1.3 / 1.4 (Power outputs: Extruder0, Extruder1, Bed)
// 35 = RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Fan)
// 36 = RAMPS 1.3 / 1.4 (Power outputs: Extruder0, Extruder1, Fan)
// 4  = Duemilanove w/ ATMega328P pin assignment
// 5  = Gen6
// 51 = Gen6 deluxe
// 6  = Sanguinololu < 1.2
// 62 = Sanguinololu 1.2 and above
// 63 = Melzi
// 64 = STB V1.1
// 65 = Azteeg X1
// 66 = Melzi with ATmega1284 (MaKr3d version)
// 67 = Azteeg X3
// 68 = Azteeg X3 Pro
// 7  = Ultimaker
// 71 = Ultimaker (Older electronics. Pre 1.5.4. This is rare)
// 72 = Ultimainboard 2.x (Uses TEMP_SENSOR 20)
// 77 = 3Drag Controller
// 8  = Teensylu
// 80 = Rumba
// 81 = Printrboard (AT90USB1286)
// 82 = Brainwave (AT90USB646)
// 83 = SAV Mk-I (AT90USB1286)
// 84 = Teensy++2.0 (AT90USB1286) // CLI compile: DEFINES=AT90USBxx_TEENSYPP_ASSIGNMENTS HARDWARE_MOTHERBOARD=84  make
// 9  = Gen3+
// 70 = Megatronics
// 701= Megatronics v2.0
// 703= Megatronics v3.0
// 702= Minitronics v1.0
// 90 = Alpha OMCA board
// 91 = Final OMCA board                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
// 301= Rambo
// 21 = Elefu Ra Board (v3)
// 88 = 5DPrint D8 Driver Board
// 999 = Leapfrog
//15 = BCN3D Sigma Rev2

//Rapduch
//Defining Boards supported
#define BCN3D_BOARD		15
#define MEGATRONICS_V3	703

//Defining Machines supported
#define BCN3D_SIGMA_PRINTER_SIGMA	3107
#define BCN3D_SIGMA_PRINTER_SIGMAX 0713

#ifndef MOTHERBOARD
//#define MOTHERBOARD MEGATRONICS_V3 //Megatronics v3
#define MOTHERBOARD BCN3D_BOARD //Marcotronics
#endif

#ifndef BCN3D_PRINTER_SETUP
	#define BCN3D_PRINTER_SETUP BCN3D_SIGMA_PRINTER_SIGMA
#endif

#ifndef BCN3D_SCREEN_VERSION_SETUP
	#define BCN3D_SCREEN_VERSION_SETUP BCN3D_SIGMA_PRINTER_SIGMA
#endif
#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
	#define PRINTER_NAME "BCN3D Sigma"
#elif BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
	#define PRINTER_NAME "BCN3D Sigmax"
#endif

#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
	//#define ENABLE_DUPLI_MIRROR
#elif BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
	#define ENABLE_DUPLI_MIRROR
#endif

#if MOTHERBOARD == MEGATRONICS_V3
	#define PROTO1
	//#define PROTO2
#endif

//	DUAL MODE SETTINGS
// This defines the Z offset threshold tolerance
#define RAFT_Z_THRESHOLD 0.05
//	END DUAL MODE SETTINGS

#define EXTRUDERS 2

// Define this to set a custom name for your generic Mendel,
// #define CUSTOM_MENDEL_NAME "This Mendel"

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
// #define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

// This defines the number of extruders
#define EXTRUDERS 2

//// The following define selects which power supply you have. Please choose the one that matches your setup
// 1 = ATX
// 2 = X-Box 360 203Watts (the blue wire connected to PS_ON and the red wire to VCC)

#define POWER_SUPPLY 1


//////////////////////////////////////////////////////////////////////////
// Enable this for dual x-carriage printers.
// A dual x-carriage design has the advantage that the inactive extruder can be parked which
// prevents hot-end ooze contaminating the print. It also reduces the weight of each x-carriage
// allowing faster printing speeds.
#define DUAL_X_CARRIAGE
#ifdef DUAL_X_CARRIAGE
// Configuration for second X-carriage
// Note: the first x-carriage is defined as the x-carriage which homes to the minimum endstop;
// the second x-carriage always homes to the maximum endstop.
#define X2_MIN_POS 0     // set minimum to ensure second x-carriage doesn't hit the parked first X-carriage
#define X2_MAX_POS X_MAX_POS    // set maximum to the distance between toolheads when both heads are homed
#define X2_HOME_DIR 1     // the second X-carriage always homes to the maximum endstop position
#define X2_HOME_POS X2_MAX_POS // default home position is the maximum carriage position
//#define X2_HOME_POS 259.5 // default home position is the maximum carriage position
// However: In this mode the EXTRUDER_OFFSET_X value for the second extruder provides a software
// override for X2_HOME_POS. This also allow recalibration of the distance between the two endstops
// without modifying the firmware (through the "M218 T1 X???" command).
// Remember: you should set the second extruder x-offset to 0 in your slicer.

// Pins for second x-carriage stepper driver (defined here to avoid further complicating pins.h)
#if MOTHERBOARD == BCN3D_BOARD
//#define X2_ENABLE_PIN	2
//#define X2_STEP_PIN		5
//#define X2_DIR_PIN		3
//#define X2_ENABLE_PIN	75//4
//#define X2_STEP_PIN		76//5
//#define X2_DIR_PIN		73//3
#else
#define X2_ENABLE_PIN 23
#define X2_STEP_PIN 22
#define X2_DIR_PIN 60
#endif

// There are a few selectable movement modes for dual x-carriages using M605 S<mode>
//    M605 S0: Full control mode. The slicer has full control over x-carriage movement
//    M605 S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
//    M605 S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
//                         millimeters x-offset and an optional differential hotend temperature of
//                         mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
//                         the first with a spacing of 100mm in the x direction and 2 degrees hotter.
//
//    Note: the X axis should be homed after changing dual x-carriage mode.
//    M605 S3: Default mode
//	  M605 S4: Duplication Mirror mode
//    M605 S5 [Xnnn] [Rmmm]: Duplication Raft mode
//	  M605 S6: Duplication Raft Mirror mode
//
//    Note: the X axis should be homed after changing dual x-carriage mode.
#define DEFAULT_DUAL_X_CARRIAGE_MODE 3

#define DXC_FULL_SIGMA_MODE 3 //SIGMA MODE: Combination of dual park and dual full control

// As the x-carriages are independent we can now account for any relative Z offset
#define EXTRUDER1_Z_OFFSET 0.0           // z offset relative to extruder 0

// Default settings in "Auto-park Mode"
#define TOOLCHANGE_PARK_ZLIFT   0.2      // the distance to raise Z axis when parking an extruder
#define TOOLCHANGE_UNPARK_ZLIFT 1        // the distance to raise Z axis when unparking an extruder

// Default x offset in duplication mode (typically set to half print bed width)
#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
	#define DEFAULT_DUPLICATION_X_OFFSET 105
#elif BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
	#define DEFAULT_DUPLICATION_X_OFFSET 210
#endif


#endif //DUAL_X_CARRIAGE/////////////////////////////////////////////////////

#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
	#define BCN3D_NOZZLE_DEFAULD_SIZE 0.4
#elif BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
	#define BCN3D_NOZZLE_DEFAULD_SIZE 0.5
#endif

// Define this to have the electronics keep the power supply off on startup. If you don't know what this is leave it.
// #define PS_DEFAULT_OFF

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================
//
//--NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
//
//// Temperature sensor settings:
// -2 is thermocouple with MAX6675 (only for sensor 0)
// -1 is thermocouple with AD595
// 0 is not used
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
// 2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
// 3 is Mendel-parts thermistor (4.7k pullup)
// 4 is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
// 5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
// 6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
// 7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
// 71 is 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
// 8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
// 9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
// 10 is 100k RS thermistor 198-961 (4.7k pullup)
// 11 is 100k beta 3950 1% thermistor (4.7k pullup)
// 12 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
// 13 is 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE" 
// 20 is the PT100 circuit found in the Ultimainboard V2.x
// 60 is 100k Maker's Tool Works Kapton Bed Thermistor beta=3950
//
//    1k ohm pullup tables - This is not normal, you would have to have changed out your 4.7k for 1k
//                          (but gives greater accuracy and more stable PID)
// 51 is 100k thermistor - EPCOS (1k pullup)
// 52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
// 55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
//
// 1047 is Pt1000 with 4k7 pullup
// 1010 is Pt1000 with 1k pullup (non standard)
// 147 is Pt100 with 4k7 pullup
// 110 is Pt100 with 1k pullup (non standard)
#define RELATIVE_TEMP_PRINT

#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 1
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_BED 1

// This makes temp sensor 1 a redundant sensor for sensor 0. If the temperatures difference between these sensors is to high the print will be aborted.
//#define TEMP_SENSOR_1_AS_REDUNDANT
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 1  // (seconds)
#define TEMP_HYSTERESIS 0.5       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     2.5       // (degC) Window around target to start the residency timer x degC early.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define BED_MINTEMP 5

//To clean the extruder's the best temperature configuration
#define	PLA_LOAD_TEMP			215
#define	PLA_UNLOAD_TEMP			170
#define	PLA_PRINT_TEMP			215
#define PLA_BED_TEMP			65

#define	ABS_LOAD_TEMP			260
#define	ABS_UNLOAD_TEMP			230
#define	ABS_PRINT_TEMP			260
#define ABS_BED_TEMP			90

#define	PVA_LOAD_TEMP			230
#define	PVA_UNLOAD_TEMP			160
#define	PVA_PRINT_TEMP			220
#define PVA_BED_TEMP			60


#define EXTRUDER_LEFT_CLEAN_TEMP 170
#define EXTRUDER_RIGHT_CLEAN_TEMP 170

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 315
#define HEATER_1_MAXTEMP 315
#define HEATER_2_MAXTEMP 315
#define BED_MAXTEMP 110

// If your bed has low resistance e.g. .6 ohm and throws the fuse you can duty cycle it to reduce the
// average current. The value should be an integer and the heat bed will be turned on for 1 interval of
// HEATER_BED_DUTY_CYCLE_DIVIDER intervals.
//#define HEATER_BED_DUTY_CYCLE_DIVIDER 4

// If you want the M105 heater power reported in watts, define the BED_WATTS, and (shared for all extruders) EXTRUDER_WATTS
//#define EXTRUDER_WATTS (12.0*12.0/6.7) //  P=I^2/R
//#define BED_WATTS (12.0*12.0/1.1)      // P=I^2/R

// PID settings:
// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX 255 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#ifdef PIDTEMP
 // #define PID_DEBUG // Sends debug data to the serial port.
  //#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
  
  //Rapduch ATENCIÓ
  #define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
                                  // is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
  #define PID_INTEGRAL_DRIVE_MAX 255  //limit for the integral term
  #define K1 0.95 //smoothing factor within the PID
 // #define PID_dT ((OVERSAMPLENR * 10.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the temperature routine
  #define PID_dT ((16.0 * 8.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the temperature routine


// If you are using a pre-configured hotend then you can use one of the value sets by uncommenting it
// Ultimaker

#if MOTHERBOARD == MEGATRONICS_V3
	#ifdef PROTO1
	#define  DEFAULT_Kp 15.16
	#define  DEFAULT_Ki 1.16
	#define  DEFAULT_Kd 49.38
	
	#endif

	#ifdef PROTO2
		 #define  DEFAULT_Kp 23.12
		 #define  DEFAULT_Ki 2.12
		 //#define  DEFAULT_Kd 62.98
	#endif
#endif


#if MOTHERBOARD == BCN3D_BOARD
	////////DEPRECATED First PID
	////////#define  DEFAULT_Kp 23.12
	////////#define  DEFAULT_Ki 2.12
	////////#define  DEFAULT_Kd 62.98
	//#define  DEFAULT_Kp 16.51
	//#define  DEFAULT_Ki 1.17
	//#define  DEFAULT_Kd 58.05
	#define  DEFAULT_Kp  15.16
	#define  DEFAULT_Ki  1.16
	#define  DEFAULT_Kd  49.38

#endif						
#endif // PIDTEMP
// Bed Temperature Control
// Select PID or bang-bang with PIDTEMPBED. If bang-bang, BED_LIMIT_SWITCHING will enable hysteresis
//
// Uncomment this to enable PID on the bed. It uses the same frequency PWM as the extruder.
// If your PID_dT above is the default, and correct for your hardware/configuration, that means 7.689Hz,
// which is fine for driving a square wave into a resistive load and does not significantly impact you FET heating.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use bed PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
#define PIDTEMPBED
//
//#define BED_LIMIT_SWITCHING

// This sets the max power delivered to the bed, and replaces the HEATER_BED_DUTY_CYCLE_DIVIDER option.
// all forms of bed control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the bed just like HEATER_BED_DUTY_CYCLE_DIVIDER did,
// so you shouldn't use it unless you are OK with PWM on your bed.  (see the comment on enabling PIDTEMPBED)
#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current

#ifdef PIDTEMPBED
//120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
//from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)

#if MOTHERBOARD == MEGATRONICS_V3
#ifdef PROTO1
	#define  DEFAULT_bedKp 302.2
	#define  DEFAULT_bedKi 56.76
	#define  DEFAULT_bedKd 402.27
#endif

#ifdef PROTO2
	#define  DEFAULT_bedKp 270.22
	#define  DEFAULT_bedKi 44.23
	#define  DEFAULT_bedKd 370.78
#endif
#endif

#if MOTHERBOARD == BCN3D_BOARD
	//DEPRECATED first PID
   //#define  DEFAULT_bedKp 270.22
   //#define  DEFAULT_bedKi 44.23
   //#define  DEFAULT_bedKd 370.78
   #define  DEFAULT_bedKp 218.76
   #define  DEFAULT_bedKi 38.70
   #define  DEFAULT_bedKd 321.59
#endif
// FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
#endif // PIDTEMPBED



//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
//#define PREVENT_LENGTHY_EXTRUDE

#define EXTRUDE_MINTEMP 150 //With DUAL X it only counts extruder0 temp
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH) //prevent extrusion of very large distances.



/*================== Thermal Runaway Protection ==============================
This is a feature to protect your printer from burn up in flames if it has
a thermistor coming off place (this happened to a friend of mine recently and
motivated me writing this feature).

The issue: If a thermistor come off, it will read a lower temperature than actual.
The system will turn the heater on forever, burning up the filament and anything
else around.

After the temperature reaches the target for the first time, this feature will 
start measuring for how long the current temperature stays below the target 
minus _HYSTERESIS (set_temperature - THERMAL_RUNAWAY_PROTECTION_HYSTERESIS).

If it stays longer than _PERIOD, it means the thermistor temperature
cannot catch up with the target, so something *may be* wrong. Then, to be on the
safe side, the system will he halt.

Bear in mind the count down will just start AFTER the first time the 
thermistor temperature is over the target, so you will have no problem if
your extruder heater takes 2 minutes to hit the target on heating.

*/
// If you want to enable this feature for all your extruder heaters,
// uncomment the 2 defines below:

// Parameters for all extruder heaters
#define THERMAL_RUNAWAY_PROTECTION_PERIOD 120 //in seconds
#define THERMAL_RUNAWAY_PROTECTION_HYSTERESIS 20 // in degree Celsius

// If you want to enable this feature for your bed heater,
// uncomment the 2 defines below:

// Parameters for the bed heater
#define THERMAL_RUNAWAY_PROTECTION_BED_PERIOD 1200 //in seconds
#define THERMAL_RUNAWAY_PROTECTION_BED_HYSTERESIS 30 // in percentage
//===========================================================================


//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// Uncomment the following line to enable CoreXY kinematics
// #define COREXY

// coarse Endstop Settings
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifndef ENDSTOPPULLUPS
  // fine endstop settings: Individual pullups. will be ignored if ENDSTOPPULLUPS is defined
  // #define ENDSTOPPULLUP_XMAX
  // #define ENDSTOPPULLUP_YMAX
  // #define ENDSTOPPULLUP_ZMAX
  // #define ENDSTOPPULLUP_XMIN
  // #define ENDSTOPPULLUP_YMIN
  // #define ENDSTOPPULLUP_ZMIN
#endif

#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool X_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool Y_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool Z_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool X_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool Y_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool Z_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
//#define DISABLE_MAX_ENDSTOPS
//#define DISABLE_MIN_ENDSTOPS

// Disable max endstops for compatibility with endstop checking routine
#if defined(COREXY) && !defined(DISABLE_MAX_ENDSTOPS)
  #define DISABLE_MAX_ENDSTOPS
#endif

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis when it's not being used.
#define DISABLE_X true
#define DISABLE_Y true
#define DISABLE_Z true
#define DISABLE_E false // For all extruders
#define DISABLE_INACTIVE_EXTRUDER false //disable only inactive extruders and keep active extruder enabled


#if MOTHERBOARD == MEGATRONICS_V3
#define INVERT_X_DIR true    // for Mendel set to false, for Orca set to true
#define INVERT_Y_DIR false    // for Mendel set to true, for Orca set to false
#define INVERT_Z_DIR false     // for Mendel set to false, for Orca set to true
#define INVERT_E0_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E1_DIR false    // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E2_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false
#endif

#if MOTHERBOARD == BCN3D_BOARD
#define INVERT_X_DIR false    // for Mendel set to false, for Orca set to true
#define INVERT_Y_DIR false    // for Mendel set to true, for Orca set to false
#define INVERT_Z_DIR true     // for Mendel set to false, for Orca set to true
#define INVERT_E0_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E1_DIR true    // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E2_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false
#endif

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1 //Homing for dual x
#define Y_HOME_DIR  1
#define Z_HOME_DIR -1

#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.

// Travel limits after homing
//32+222+28=282
#if MOTHERBOARD == MEGATRONICS_V3
#ifdef PROTO1
	#define X_MAX_POS 317.9 //Distance between extruders
	#define X_MIN_POS 0
	#define Y_MAX_POS 280
	#define Y_MIN_POS 0
	#define Z_MAX_POS 150
	#define Z_MIN_POS 0
#endif

#ifdef PROTO2
	#define X_MAX_POS 318 //Distance between extruders
	#define X_MIN_POS 0
	#define Y_MAX_POS 280
	#define Y_MIN_POS 0
	#define Z_MAX_POS 150

	#define Z_MIN_POS 0
#endif
#endif

#if MOTHERBOARD == BCN3D_BOARD
	#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
		#define X_MAX_POS 305.6//312 //Distance between extruders
	#endif
	#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
		#define X_MAX_POS 526.7//312 //Distance between extruders
	#endif
	//#define X_MAX_POS 210 //Bed X
	#define X_MIN_POS 0
	#define Y_MAX_POS 295
	#define Y_MIN_POS 0
	#define Z_MAX_POS 210
	#define Z_MIN_POS 0
#endif


#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)
//============================= Bed Auto Leveling ===========================


#define ENABLE_AUTO_BED_LEVELING // Delete the comment to enable (remove // at the start of the line)
#define Z_PROBE_REPEATABILITY_TEST  // If not commented out, Z-Probe Repeatability test will be included if Auto Bed Leveling is Enabled.

#ifdef ENABLE_AUTO_BED_LEVELING

// There are 2 different ways to pick the X and Y locations to probe:

//  - "grid" mode
//    Probe every point in a rectangular grid
//    You must specify the rectangle, and the density of sample points
//    This mode is preferred because there are more measurements.
//    It used to be called ACCURATE_BED_LEVELING but "grid" is more descriptive

//  - "3-point" mode
//    Probe 3 arbitrary points on the bed (that aren't colinear)
//    You must specify the X & Y coordinates of all 3 points

  //#define AUTO_BED_LEVELING_GRID
  // with AUTO_BED_LEVELING_GRID, the bed is sampled in a
  // AUTO_BED_LEVELING_GRID_POINTSxAUTO_BED_LEVELING_GRID_POINTS grid
  // and least squares solution is calculated
  // Note: this feature occupies 10'206 byte
  #ifdef AUTO_BED_LEVELING_GRID

    // set the rectangle in which to probe
    #define LEFT_PROBE_BED_POSITION 15
    #define RIGHT_PROBE_BED_POSITION 170
    #define BACK_PROBE_BED_POSITION 180
    #define FRONT_PROBE_BED_POSITION 20

     // set the number of grid points per dimension
     // I wouldn't see a reason to go above 3 (=9 probing points on the bed)
    #define AUTO_BED_LEVELING_GRID_POINTS 2


  #else  // not AUTO_BED_LEVELING_GRID
    // with no grid, just probe 3 arbitrary points.  A simple cross-product
    // is used to esimate the plane of the print bed

      #define ABL_PROBE_PT_1_X 20
      #define ABL_PROBE_PT_1_Y 260
      #define ABL_PROBE_PT_2_X 200
      #define ABL_PROBE_PT_2_Y 260
      #define ABL_PROBE_PT_3_X 120
      #define ABL_PROBE_PT_3_Y 20

  #endif // AUTO_BED_LEVELING_GRID


  // these are the offsets to the probe relative to the extruder tip (Hotend - Probe)
  #define X_PROBE_OFFSET_FROM_EXTRUDER  20
  #define Y_PROBE_OFFSET_FROM_EXTRUDER	24
  #define Z_PROBE_OFFSET_FROM_EXTRUDER  5
  
  //Rapduch
  //#define X_PROBE2_OFFSET_FROM_EXTRUDER -25
  //#define Y_PROBE2_OFFSET_FROM_EXTRUDER -29
  //#define Z_PROBE2_OFFSET_FROM_EXTRUDER -12.35

  #define Z_RAISE_BEFORE_HOMING 7       // (in mm) Raise Z before homing (G28) for Probe Clearance.
                                        // Be sure you have this distance over your Z_MAX_POS in case

  #define XY_TRAVEL_SPEED 8000         // X and Y axis travel speed between probes, in mm/min
  #define XY_TRAVEL_SPEED15 12000         // X and Y axis travel speed between probes, in mm/min  

  #define Z_RAISE_BEFORE_PROBING 5    //How much the extruder will be raised before traveling to the first probing point.
  #define Z_RAISE_BETWEEN_PROBINGS 2.5  //How much the extruder will be raised when traveling from between next probing points

  //#define Z_PROBE_SLED // turn on if you have a z-probe mounted on a sled like those designed by Charles Bell
  //#define SLED_DOCKING_OFFSET 5 // the extra distance the X axis must travel to pickup the sled. 0 should be fine but you can push it further if you'd like.

  //If defined, the Probe servo will be turned on only during movement and then turned off to avoid jerk
  //The value is the delay to turn the servo off after powered on - depends on the servo speed; 300ms is good value, but you can try lower it.
  // You MUST HAVE the SERVO_ENDSTOPS defined to use here a value higher than zero otherwise your code will not compile.

//  #define PROBE_SERVO_DEACTIVATION_DELAY 300


//If you have enabled the Bed Auto Leveling and are using the same Z Probe for Z Homing,
//it is highly recommended you let this Z_SAFE_HOMING enabled!!!

  #define Z_SAFE_HOMING   // This feature is meant to avoid Z homing with probe outside the bed area.
                          // When defined, it will:
                          // - Allow Z homing only after X and Y homing AND stepper drivers still enabled
                          // - If stepper drivers timeout, it will need X and Y homing again before Z homing
                          // - Position the probe in a defined XY point before Z Homing when homing all axis (G28)
                          // - Block Z homing only when the probe is outside bed area.

  #ifdef Z_SAFE_HOMING
    #define Z_SAFE_HOMING_X_POINT (X_MAX_LENGTH/2)    // X point for Z homing when homing all axis (G28)
    #define Z_SAFE_HOMING_Y_POINT (Y_MAX_LENGTH/2)    // Y point for Z homing when homing all axis (G28)
  #endif
  

#endif // ENABLE_AUTO_BED_LEVELING------------------------------------------------------------------------------





//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// Custom M code points
#define CUSTOM_M_CODES
#ifdef CUSTOM_M_CODES
  #define CUSTOM_M_CODE_SET_Z_PROBE_OFFSET 851
  #define Z_PROBE_OFFSET_RANGE_MIN -15
  #define Z_PROBE_OFFSET_RANGE_MAX -5
#endif


// EEPROM
// The microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable EEPROM support
#define EEPROM_SETTINGS
//to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
//#define EEPROM_CHITCHAT

// Preheat Constants
#define PLA_PREHEAT_HOTEND_TEMP 215
#define PLA_PREHEAT_HPB_TEMP 45
#define PLA_PREHEAT_FAN_SPEED 0   // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP 70
#define ABS_PREHEAT_FAN_SPEED 0   // Insert Value between 0 and 255

//LCD and SD support
//#define ULTRA_LCD  //general LCD support, also 16x2
//#define DOGLCD  // Support for SPI LCD 128x64 (Controller ST7565R graphic Display Family)
#define SDSUPPORT // Enable SD Card Support in Hardware Console
//#define SDSLOW // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SD_CHECK_AND_RETRY // Use CRC checks and retries on the SD communication
//#define ENCODER_PULSES_PER_STEP 1 // Increase if you have a high resolution encoder
//#define ENCODER_STEPS_PER_MENU_ITEM 5 // Set according to ENCODER_PULSES_PER_STEP or your liking
//#define ULTIMAKERCONTROLLER //as available from the Ultimaker online store.
//#define ULTIPANEL  //the UltiPanel as on Thingiverse
//#define LCD_FEEDBACK_FREQUENCY_HZ 1000	// this is the tone frequency the buzzer plays when on UI feedback. ie Screen Click
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100 // the duration the buzzer plays the UI feedback sound. ie Screen Click


//////////////////////////////////////////////////////////////////////////////////////////////////
//**********************************************************************************************//
//----------------------------------SIGMA DEFINITIONS-------------------------------------------//
//**********************************************************************************************//
//////////////////////////////////////////////////////////////////////////////////////////////////
//4D LCD Touch Screen for RepRapSigma
#define SIGMA_TOUCH_SCREEN

//Second Extruder endstop enable (needed for BED AUTOCAL)
#define SIGMA_ENDSTOP

//Extruder Cal Wizard
#define EXTRUDER_CALIBRATION_WIZARD

//Auto Bed Calib
#define SIGMA_BED_AUTOCALIB

//Quick guide control
//#define DEFAULT_QUICK_GUIDE false;
#define DEFAULT_PRINT_TEMP  PLA_PRINT_TEMP;
#define DEFAULT_INSERT_TEMP PLA_LOAD_TEMP;
#define DEFAULT_REMOVE_TEMP PLA_UNLOAD_TEMP;
#define DEFAULT_BED_TEMP	PLA_BED_TEMP;
#define DEFAULT_OLD_PRINT_TEMP PLA_PRINT_TEMP;
#define DEFAULT_OLD_INSERT_TEMP PLA_LOAD_TEMP;
#define DEFAULT_OLD_REMOVE_TEMP PLA_UNLOAD_TEMP;
#define DEFAULT_OLD_BED_TEMP	PLA_BED_TEMP;


//Rapduch
//Insert Filament parameters
#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
	#define BOWDEN_LENGTH 875
#endif
#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
	#define BOWDEN_LENGTH 1050
#endif
#define EXTRUDER_LENGTH 50
#define INSERT_FAST_SPEED 3600	//max speed 60mm/s
#define INSERT_SLOW_SPEED 150
#define REMOVE_FIL_TEMP		170
#define INSERT_FIL_TEMP		230

//For better understanding on which extruder is selected
#define LEFT_EXTRUDER 0 
#define RIGHT_EXTRUDER 1

//Rapduch 
//For sigma Autolevel
#define Z_SIGMA_HOME
#define Z_SIGMA_AUTOLEVEL


#ifdef Z_SIGMA_HOME
	#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
		#define Z_SIGMA_HOME_X_POINT 58
	#endif
	#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
		#define Z_SIGMA_HOME_X_POINT 58
	#endif
	#define Z_SIGMA_HOME_Y_POINT 150
	
	#define SIGMA_Z_HOME_TRAVEL_SPEED 13000
	
	#define Z_SIGMA_RAISE_BEFORE_HOMING 10
	#define Z_SIGMA_RAISE_AFTER_HOMING 5

#endif


#ifdef Z_SIGMA_AUTOLEVEL
	#if MOTHERBOARD == BCN3D_BOARD
		#define XY_SIGMA_TRAVEL_SPEED 4000
	#else
		#define XY_SIGMA_TRAVEL_SPEED 8000
	#endif

	#if MOTHERBOARD == MEGATRONICS_V3
		#ifdef PROTO1
			#define X_SIGMA_PROBE_OFFSET_FROM_EXTRUDER  19
			#define Y_SIGMA_PROBE_OFFSET_FROM_EXTRUDER	24
			#define Z_SIGMA_PROBE_OFFSET_FROM_EXTRUDER  5.3 //It is negative, it is compensated
		#endif
		#ifdef PROTO2
			#define X_SIGMA_PROBE_OFFSET_FROM_EXTRUDER  
			#define Y_SIGMA_PROBE_OFFSET_FROM_EXTRUDER	24
			#define Z_SIGMA_PROBE_OFFSET_FROM_EXTRUDER  5 //It is negative, it is compensated
		#endif
	#endif
	
	#if MOTHERBOARD == BCN3D_BOARD
		#define X_SIGMA_PROBE_OFFSET_FROM_EXTRUDER  17.05//20
		#define Y_SIGMA_PROBE_OFFSET_FROM_EXTRUDER	22.2
		#define Z_SIGMA_PROBE_OFFSET_FROM_EXTRUDER  2.8//2.80//3.4 //It is negative, it is compensated
		
		#define X_SIGMA_SECOND_PROBE_OFFSET_FROM_EXTRUDER	-13.4
		#define Y_SIGMA_SECOND_PROBE_OFFSET_FROM_EXTRUDER	22.2
		#define Z_SIGMA_SECOND_PROBE_OFFSET_FROM_EXTRUDER	2.8//2.90
		
	#endif
	
	
	#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
		//Left extruder probe point
		#define X_SIGMA_PROBE_1_LEFT_EXTR 55
		#define Y_SIGMA_PROBE_1_LEFT_EXTR 265
		//#define Y_SIGMA_PROBE_1_LEFT_EXTR 275
	
		#define X_SIGMA_PROBE_2_LEFT_EXTR 55
		#define Y_SIGMA_PROBE_2_LEFT_EXTR 10
		//#define Y_SIGMA_PROBE_2_LEFT_EXTR 10
	
		#define X_SIGMA_PROBE_3_LEFT_EXTR 254
		#define Y_SIGMA_PROBE_3_LEFT_EXTR 10
		//#define Y_SIGMA_PROBE_3_LEFT_EXTR 10
	
		//Right extruder probe point
		#define X_SIGMA_PROBE_1_RIGHT_EXTR 254
		#define Y_SIGMA_PROBE_1_RIGHT_EXTR 265
		//#define Y_SIGMA_PROBE_1_RIGHT_EXTR 275
	
		#define X_SIGMA_PROBE_2_RIGHT_EXTR 254
		#define Y_SIGMA_PROBE_2_RIGHT_EXTR 10
		//#define Y_SIGMA_PROBE_2_RIGHT_EXTR 10
	
		#define X_SIGMA_PROBE_3_RIGHT_EXTR 55
		#define Y_SIGMA_PROBE_3_RIGHT_EXTR 10
		//#define Y_SIGMA_PROBE_3_RIGHT_EXTR 10
		
		#define X_GAP_AVOID_COLLISION_LEFT	10
		#define X_GAP_AVOID_COLLISION_RIGHT	13
		
	#endif
	#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
		//Left extruder probe point
		#define X_SIGMA_PROBE_1_LEFT_EXTR 58.5
		#define Y_SIGMA_PROBE_1_LEFT_EXTR 265
		//#define Y_SIGMA_PROBE_1_LEFT_EXTR 275
	
		#define X_SIGMA_PROBE_2_LEFT_EXTR 58.5
		#define Y_SIGMA_PROBE_2_LEFT_EXTR 10
		//#define Y_SIGMA_PROBE_2_LEFT_EXTR 10
	
		#define X_SIGMA_PROBE_3_LEFT_EXTR 468 //254
		#define Y_SIGMA_PROBE_3_LEFT_EXTR 10
		//#define Y_SIGMA_PROBE_3_LEFT_EXTR 10
	
		//Right extruder probe point
		#define X_SIGMA_PROBE_1_RIGHT_EXTR 468//254
		#define Y_SIGMA_PROBE_1_RIGHT_EXTR 265
		//#define Y_SIGMA_PROBE_1_RIGHT_EXTR 275
	
		#define X_SIGMA_PROBE_2_RIGHT_EXTR 468 ///254
		#define Y_SIGMA_PROBE_2_RIGHT_EXTR 10
		//#define Y_SIGMA_PROBE_2_RIGHT_EXTR 10
	
		#define X_SIGMA_PROBE_3_RIGHT_EXTR 58.5
		#define Y_SIGMA_PROBE_3_RIGHT_EXTR 10
		//#define Y_SIGMA_PROBE_3_RIGHT_EXTR 10
		
		#define X_GAP_AVOID_COLLISION_LEFT	14.5
		#define X_GAP_AVOID_COLLISION_RIGHT	19.5
	#endif
#endif

#ifdef  SIGMA_BED_AUTOCALIB
//Calibration WIZARD --------
	#define PAS_M5 0.80
	#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
		//Screw positions on BED for
		#define CARGOL_1_X  152
		//#define CARGOL_1_X  104;
		#define CARGOL_1_Y  272.5

		#define CARGOL_2_X  74.5
		//#define CARGOL_2_X  17;
		#define CARGOL_2_Y  19

		#define CARGOL_3_X  230.5
		//#define CARGOL_3_X  192;
		#define CARGOL_3_Y 19
	#elif BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
		//Screw positions on BED for
		#define CARGOL_1_X  263.5
		//#define CARGOL_1_X  104;
		#define CARGOL_1_Y  272.5

		#define CARGOL_2_X  86
		//#define CARGOL_2_X  17;
		#define CARGOL_2_Y  19

		#define CARGOL_3_X  441
		//#define CARGOL_3_X  192;
		#define CARGOL_3_Y  19
	#endif
	// -END BED calibration WIZARD
#endif // ENABLE_AUTO_BED_LEVELING

#ifdef EXTRUDER_CALIBRATION_WIZARD
	#define SECOND_EXTRUDER_X {40.5, 39.5, 40, 39}
	#define SECOND_EXTRUDER_Y {150.5, 149.5, 150, 149}
		
		
	#define X_CALIB_STARTING_X 117.5
	#define X_CALIB_STARTING_Y 99.5
	#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
		#define X_OFFSET_CALIB_PROCEDURES 109
		#define X_OFFSET_BEDCOMPENSATION_PROCEDURE 102
	#endif
#endif

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
//#define EXTRUDER_OFFSET_X {0.0, X2_MAX_POS} // (in mm) for each extruder, offset of the hotend on the X axis
#define EXTRUDER_OFFSET_X {0.0, X2_HOME_POS} // (in mm) for each extruder, offset of the hotend on the X axis
	
	
#if MOTHERBOARD == MEGATRONICS_V3	
	#ifdef PROTO1
		#define EXTRUDER_OFFSET_Y {0.0,  0.1}  // (in mm) for each extruder, offset of the hotend on the Y axis
		#define EXTRUDER_OFFSET_Z {0.0 , 0.0}
	#endif

	#ifdef PROTO2
		#define EXTRUDER_OFFSET_Y {0.0,  0.1}  // (in mm) for each extruder, offset of the hotend on the Y axis
		#define EXTRUDER_OFFSET_Z {0.0 , 0.1}
	#endif
#endif

#if MOTHERBOARD == BCN3D_BOARD
		#define EXTRUDER_OFFSET_Y {0.0,  -0.15}  // (in mm) for each extruder, offset of the hotend on the Y axis
		#define EXTRUDER_OFFSET_Z {0.0 , 0.0}
#endif

#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
	#define NOZZLE_PARK_DISTANCE_BED_X0	47
	#define NOZZLE_PARK_DISTANCE_BED_Y0	-2.5
	#define PRINTER_BED_X_SIZE	210.0
#endif
#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
	#define NOZZLE_PARK_DISTANCE_BED_X0	54.5
	#define NOZZLE_PARK_DISTANCE_BED_Y0	-2.5
	#define PRINTER_BED_X_SIZE	420.0
#endif
//----------------------------------------------------------------------------------------------


//MANUAL HOMING & FEEDRATES---------------------
// The position of the homing switches
//#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
//#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)

//Manual homing switch locations:
// For deltabots this means top and center of the Cartesian print volume.
//#define MANUAL_X_HOME_POS -32
#define MANUAL_X_HOME_POS -52.5
#define MANUAL_Y_HOME_POS Y_MAX_POS
#define MANUAL_Z_HOME_POS Z_MIN_POS

//// MOVEMENT SETTINGS
#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
#define NUM_LINES 10
//#define HOMING_FEEDRATE {50*60, 50*60, 4*60, 0}  // set the homing speeds (mm/min)
#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
	#define HOMING_FEEDRATE {60*60, 90*60, 12*60, 0} ///{75*60, 65*60, 6*60, 0}  // old homing speeds (mm/min)
#else
	#define HOMING_FEEDRATE {60*60, 90*60, 12*60, 0} ///{75*60, 65*60, 6*60, 0}  // old homing speeds (mm/min)
#endif
#define CALIB_FEEDRATE_ZAXIS 6*60 // bed homing speeds (mm/min)
// default settings if screen not defined
//#ifndef SIGMA_TOUCH_SCREEN
//#define DEFAULT_AXIS_STEPS_PER_UNIT   {80,80,2560,96.43}  // default steps per unit for Ultimaker
//#endif

#ifdef SIGMA_TOUCH_SCREEN //If Sigma Touch Screen enabled
	#if MOTHERBOARD == BCN3D_BOARD
		//#define DEFAULT_AXIS_STEPS_PER_UNIT {160,160,3200,608}  //1/32 microstepping for BCN3D Board
		#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
		#define DEFAULT_AXIS_STEPS_PER_UNIT {80,80,1600,152}	  //1/16 microstepping for BCN3D Board
		#elif BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
		#define DEFAULT_AXIS_STEPS_PER_UNIT {80,80,1600,492.45}	  //1/16 microstepping for bondtech kit BCN3D
		#endif
		//#define DEFAULT_AXIS_STEPS_PER_UNIT {40,40,800,102}	  //MK7 1/8 microstepping for BCN3D Board
	#else
		#if MOTHERBOARD == MEGATRONICS_V3
			#ifdef PROTO1
				#define DEFAULT_AXIS_STEPS_PER_UNIT   {80,80,1600,152}  // 1/16 for Megatronicsv3 MK8
			#endif

			#ifdef PROTO2
				#define DEFAULT_AXIS_STEPS_PER_UNIT   {80,80,1600,102}  // 1/16 for Megatronicsv3 MK7
			#endif
		#endif	
	#endif
#endif


#if BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMA
	#define DEFAULT_MAX_FEEDRATE          {200, 200, 12, 60}    // (mm/sec)
	#define DEFAULT_MAX_ACCELERATION      {2250,2250,80,800}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
	#define DEFAULT_HYSTERESIS_MM         0, 0, 0, 0  // X, Y, Z, E hysteresis in mm. These are the extra distances that are performed when an axis changes direction to compensate for any mechanical hysteresis your printer has.
	#define DEFAULT_HYSTERESIS
	//#define DEFAULT_MAX_ACCELERATION      {2000,2000,50,1000}
	//#define DEFAULT_MAX_FEEDRATE          {250, 250, 3.5, 50}    // (mm/sec)
	//#define DEFAULT_MAX_ACCELERATION      {1000,1000,100,100}    // X, Y, Z, E maximum star

	#define DEFAULT_ACCELERATION          2000    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
	#define DEFAULT_RETRACT_ACCELERATION  2000   // X, Y, Z and E max acceleration in mm/s^2 for retracts
	//#define DEFAULT_ACCELERATION          1000    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
	//#define DEFAULT_RETRACT_ACCELERATION  2000   // X, Y, Z and E max acceleration in mm/s^2 for retracts

	// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
	#define DEFAULT_XYJERK                5.0    // (mm/sec)
	#define DEFAULT_ZJERK                 0.4     // (mm/sec)
	#define DEFAULT_EJERK                 5.0    // (mm/sec)
#elif BCN3D_PRINTER_SETUP == BCN3D_SIGMA_PRINTER_SIGMAX
	#define DEFAULT_MAX_FEEDRATE          {200, 200, 12, 40}    // (mm/sec)
	#define DEFAULT_MAX_ACCELERATION      {2250,2250,80,800}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
	#define DEFAULT_HYSTERESIS_MM         0, 0, 0, 0  // X, Y, Z, E hysteresis in mm. These are the extra distances that are performed when an axis changes direction to compensate for any mechanical hysteresis your printer has.
	#define DEFAULT_HYSTERESIS
	//#define DEFAULT_MAX_ACCELERATION      {2000,2000,50,1000}
	//#define DEFAULT_MAX_FEEDRATE          {250, 250, 3.5, 50}    // (mm/sec)
	//#define DEFAULT_MAX_ACCELERATION      {1000,1000,100,100}    // X, Y, Z, E maximum star

	#define DEFAULT_ACCELERATION          1750    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
	#define DEFAULT_RETRACT_ACCELERATION  2000   // X, Y, Z and E max acceleration in mm/s^2 for retracts
	//#define DEFAULT_ACCELERATION          1000    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
	//#define DEFAULT_RETRACT_ACCELERATION  2000   // X, Y, Z and E max acceleration in mm/s^2 for retracts

	// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
	#define DEFAULT_XYJERK                12.5    // (mm/sec)
	#define DEFAULT_ZJERK                 0.4     // (mm/sec)
	#define DEFAULT_EJERK                 5.0    // (mm/sec)
#endif


//------------------------------------------------------------------------------------------------------



// The MaKr3d Makr-Panel with graphic controller and SD support
// http://reprap.org/wiki/MaKr3d_MaKrPanel
//#define MAKRPANEL

// The RepRapDiscount Smart Controller (white PCB)
// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
//#define REPRAP_DISCOUNT_SMART_CONTROLLER

// The GADGETS3D G3D LCD/SD Controller (blue PCB)
// http://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
//#define G3D_PANEL

// The RepRapDiscount FULL GRAPHIC Smart Controller (quadratic white PCB)
// http://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
//
// ==> REMEMBER TO INSTALL U8glib to your ARDUINO library folder: http://code.google.com/p/u8glib/wiki/u8glib
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

// The RepRapWorld REPRAPWORLD_KEYPAD v1.1
// http://reprapworld.com/?products_details&products_id=202&cPath=1591_1626
//#define REPRAPWORLD_KEYPAD
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0 // how much should be moved when a key is pressed, eg 10.0 means 10mm per click

// The Elefu RA Board Control Panel
// http://www.elefu.com/index.php?route=product/product&product_id=53
// REMEMBER TO INSTALL LiquidCrystal_I2C.h in your ARUDINO library folder: https://github.com/kiyoshigawa/LiquidCrystal_I2C
//#define RA_CONTROL_PANEL

//automatic expansion
#if defined (MAKRPANEL)
 #define DOGLCD
 #define SDSUPPORT
 #define ULTIPANEL
 #define NEWPANEL
 #define DEFAULT_LCD_CONTRAST 17
#endif

#if defined (REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
 #define DOGLCD
 #define U8GLIB_ST7920
 #define REPRAP_DISCOUNT_SMART_CONTROLLER
#endif

#if defined(ULTIMAKERCONTROLLER) || defined(REPRAP_DISCOUNT_SMART_CONTROLLER) || defined(G3D_PANEL)
 #define ULTIPANEL
 #define NEWPANEL
#endif

#if defined(REPRAPWORLD_KEYPAD)
  #define NEWPANEL
  #define ULTIPANEL
#endif
#if defined(RA_CONTROL_PANEL)
 #define ULTIPANEL
 #define NEWPANEL
 #define LCD_I2C_TYPE_PCA8574
 #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
#endif

//I2C PANELS

//#define LCD_I2C_SAINSMART_YWROBOT
#ifdef LCD_I2C_SAINSMART_YWROBOT
  // This uses the LiquidCrystal_I2C library ( https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home )
  // Make sure it is placed in the Arduino libraries directory.
  #define LCD_I2C_TYPE_PCF8575
  #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
  #define NEWPANEL
  #define ULTIPANEL
#endif

// PANELOLU2 LCD with status LEDs, separate encoder and click inputs
//#define LCD_I2C_PANELOLU2
#ifdef LCD_I2C_PANELOLU2
  // This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
  // Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
  // (v1.2.3 no longer requires you to define PANELOLU in the LiquidTWI2.h library header file)
  // Note: The PANELOLU2 encoder click input can either be directly connected to a pin
  //       (if BTN_ENC defined to != -1) or read through I2C (when BTN_ENC == -1).
  #define LCD_I2C_TYPE_MCP23017
  #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
  #define LCD_USE_I2C_BUZZER //comment out to disable buzzer on LCD
  #define NEWPANEL
  #define ULTIPANEL

  #ifndef ENCODER_PULSES_PER_STEP
	#define ENCODER_PULSES_PER_STEP 4
  #endif

  #ifndef ENCODER_STEPS_PER_MENU_ITEM
	#define ENCODER_STEPS_PER_MENU_ITEM 1
  #endif


  #ifdef LCD_USE_I2C_BUZZER
	#define LCD_FEEDBACK_FREQUENCY_HZ 1000
	#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100
  #endif

#endif

// Panucatt VIKI LCD with status LEDs, integrated click & L/R/U/P buttons, separate encoder inputs
//#define LCD_I2C_VIKI
#ifdef LCD_I2C_VIKI
  // This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
  // Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
  // Note: The pause/stop/resume LCD button pin should be connected to the Arduino
  //       BTN_ENC pin (or set BTN_ENC to -1 if not used)
  #define LCD_I2C_TYPE_MCP23017
  #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
  #define LCD_USE_I2C_BUZZER //comment out to disable buzzer on LCD (requires LiquidTWI2 v1.2.3 or later)
  #define NEWPANEL
  #define ULTIPANEL
#endif

// Shift register panels
// ---------------------
// 2 wire Non-latching LCD SR from:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics#!shiftregister-connection
//#define SR_LCD
#ifdef SR_LCD
   #define SR_LCD_2W_NL    // Non latching 2 wire shift register
   //#define NEWPANEL
#endif


#ifdef ULTIPANEL
//  #define NEWPANEL  //enable this if you have a click-encoder panel
  #define SDSUPPORT
  #define ULTRA_LCD
  #ifdef DOGLCD // Change number of lines to match the DOG graphic display
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 5
  #else
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 4
  #endif
#else //no panel but just LCD
  #ifdef ULTRA_LCD
  #ifdef DOGLCD // Change number of lines to match the 128x64 graphics display
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 5
  #else
    #define LCD_WIDTH 16
    #define LCD_HEIGHT 2
  #endif
  #endif
#endif

// default LCD contrast for dogm-like LCD displays
#ifdef DOGLCD
# ifndef DEFAULT_LCD_CONTRAST
#  define DEFAULT_LCD_CONTRAST 32
# endif
#endif

// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
//#define FAST_PWM_FAN

// Temperature status LEDs that display the hotend and bet temperature.
// If all hotends and bed temperature and temperature setpoint are < 54C then the BLUE led is on.
// Otherwise the RED led is on. There is 1C hysteresis.
//#define TEMP_STAT_LEDS

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency
// which is not ass annoying as with the hardware PWM. On the other hand, if this frequency
// is too low, you should also increment SOFT_PWM_SCALE.
//#define FAN_SOFT_PWM

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
#define SOFT_PWM_SCALE 0

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
// #define PHOTOGRAPH_PIN     23

// SF send wrong arc g-codes when using Arc Point as fillet procedure
//#define SF_ARC_FIX

// Support for the BariCUDA Paste Extruder.
//#define BARICUDA

//define BlinkM/CyzRgb Support
//#define BLINKM

/*********************************************************************\
* R/C SERVO support
* Sponsored by TrinityLabs, Reworked by codexmas
**********************************************************************/

// Number of servos
//
// If you select a configuration below, this will receive a default value and does not need to be set manually
// set it manually if you have more servos than extruders and wish to manually control some
// leaving it undefined or defining as 0 will disable the servo subsystem
// If unsure, leave commented / disabled
//
//#define NUM_SERVOS 3 // Servo index starts with 0 for M280 command

// Servo Endstops
//
// This allows for servo actuated endstops, primary usage is for the Z Axis to eliminate calibration or bed height changes.
// Use M206 command to correct for switch height offset to actual nozzle height. Store that setting with M500.
//
//#define SERVO_ENDSTOPS {-1, -1, 0} // Servo index for X, Y, Z. Disable with -1
//#define SERVO_ENDSTOP_ANGLES {0,0, 0,0, 70,0} // X,Y,Z Axis Extend and Retract angles

/**********************************************************************\
 * Support for a filament diameter sensor
 * Also allows adjustment of diameter at print time (vs  at slicing)
 * Single extruder only at this point (extruder 0)
 * 
 * Motherboards
 * 34 - RAMPS1.4 - uses Analog input 5 on the AUX2 connector 
 * 81 - Printrboard - Uses Analog input 2 on the Aux 2 connector
 * 301 - Rambo  - uses Analog input 3
 * Note may require analog pins to be defined for different motherboards
 **********************************************************************/
// Uncomment below to enable
//#define FILAMENT_SENSOR

#define FILAMENT_SENSOR_EXTRUDER_NUM	0  //The number of the extruder that has the filament sensor (0,1,2)
#define MEASUREMENT_DELAY_CM			14  //measurement delay in cm.  This is the distance from filament sensor to middle of barrel

#define DEFAULT_NOMINAL_FILAMENT_DIA  3.0  //Enter the diameter (in mm) of the filament generally used (3.0 mm or 1.75 mm) - this is then used in the slicer software.  Used for sensor reading validation
#define MEASURED_UPPER_LIMIT          3.30  //upper limit factor used for sensor reading validation in mm
#define MEASURED_LOWER_LIMIT          1.90  //lower limit factor for sensor reading validation in mm
#define MAX_MEASUREMENT_DELAY			20  //delay buffer size in bytes (1 byte = 1cm)- limits maximum measurement delay allowable (must be larger than MEASUREMENT_DELAY_CM  and lower number saves RAM)

//defines used in the code
#define DEFAULT_MEASURED_FILAMENT_DIA  DEFAULT_NOMINAL_FILAMENT_DIA  //set measured to nominal initially 


/////////////////////////////////BCN3D SIGMA  CONFIG////////////////////////////////////////////////////////////////////

//////	BED COMPENSATION G36

#define RETRACT_SPEED_G36		2100


//////	FULL CALIBRATION SETTINGS

#define CALIBFULL_HOTEND_STANDBY_TEMP		170

//////	SDFILES LISTING

#define SDFILES_LIST_NUM				 5
#define SDFILES_LIST_GCODE_LENGHT		 20		//String GcodePrinting

//////	PURGE

#define PURGE_TEMP_HYSTERESIS		 10			//(mm)
#define PURGE_DISTANCE_INSERTED		 30			//(mm)

//////	PID ITERATIONS AUTOTUNE

#define AUTOTUNE_ITERATIONS		 10			// iterations

//////	CHANGE FILAMENT

#define CHANGE_FIL_TEMP_HYSTERESIS		 10				//(ºC)

//////	NYLON CLEANING

#define NYLON_TEMP_HYSTERESIS				 5			//(ºC)
#define NYLON_TEMP_HEATUP_THRESHOLD			 260		//(ºC)
#define NYLON_TEMP_COOLDOWN_THRESHOLD		 60			//(ºC)

//////	PAUSE PRINT


#define PAUSE_G69_RETRACT		 4				//(mm)
#define PAUSE_G69_XYMOVE		 5				//(mm)
#define PAUSE_G70_ZMOVE			 2				//(mm)
#define PAUSE_G70_PURGE			 10				//(mm)

//////	THERMISTOR LECTURE PROTECTION 

#define	THERMAL_LECTURE_FAILURE

//////	INACTIVITY SHUTDOWN HEATING

#define TIMERCOOLDOWN 10*60 // 10min

////// PRINTING RECOVERY

#define RECOVERY_PRINT

////// FIRST RUN WIZARD

#define FIRST_START_WIZARD

///////	GIFs SETUP 

//	FRAME RATE

#define GIF_FRAMERATE				40   /// 40ms------> 25fps

//NUMBER OF FRAMES PER GIF

#define GIF_FRAMES_SUCCESS				40
#define GIF_FRAMES_NYLONSTEP3			22
#define GIF_FRAMES_PURGELOAD			35    
#define GIF_FRAMES_BEDSCREW				43     
#define GIF_FRAMES_ZCALIB				49
#define GIF_FRAMES_ZSET					49
#define GIF_FRAMES_CALIBTEST			36
#define GIF_FRAMES_PROCESSING			15
#define GIF_FRAMES_CHANGEFILAMENTTEMP   43
#define GIF_FRAMES_ADJUSTINGTEMPS		44
#define GIF_FRAMES_NYLONTEMPS			43
#define GIF_FRAMES_ERROR				44
#define GIF_FRAMES_PREHEAT				41
#define GIF_FRAMES_INIT_FIRST_RUN		124

///////	SMART PURGE SETUP 

#define SMARTPURGE_SETUP_1

#define WAITPERIOD_PRESS_BUTTON 200
#define WAITPERIOD_PRESS_BUTTON2 400
//#define SCREENTEST

//#define ErroWindowEnable

//GIF PROCESSING DEFINITIONS

#define PROCESSING_STOP						'\0'
#define PROCESSING_DEFAULT					'A'
#define PROCESSING_SUCCESS					'B'
#define PROCESSING_SUCCESS_FIRST_RUN		'C'
#define PROCESSING_BED_SUCCESS				'D'
#define PROCESSING_SAVE_PRINT_SUCCESS		'E'
#define PROCESSING_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP4				'F'
#define PROCESSING_PURGE_LOAD				'G'
#define PROCESSING_CHANGE_FILAMENT_TEMPS	'H'
#define PROCESSING_ADJUSTING				'I'
#define PROCESSING_UTILITIES_MAINTENANCE_NYLONCLEANING_TEMPS				'J'
#define PROCESSING_BED						'K'
#define PROCESSING_CALIB_ZL					'L'
#define PROCESSING_CALIB_ZR					'M'
#define PROCESSING_ERROR					'N'
#define PROCESSING_BED_FIRST				'O'
#define PROCESSING_TEST						'P'
#define PROCESSING_UTILITIES_MAINTENANCE_NYLONCLEANING_STEP3				'Q'

//PRINTER STATE DEFINITIONS

#define STATE_NONE							'\0'
#define STATE_MENU							'A'
#define STATE_LOADUNLOAD_FILAMENT			'B'
#define STATE_NYLONCLEANING					'C'
#define STATE_CALIBRATION					'D'
#define STATE_AUTOTUNEPID					'E'
/*
#define PROCESSING_Z_SET_0					'R'
#define PROCESSING_Z_SET_1					'S'
*/
#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //__CONFIGURATION_H
