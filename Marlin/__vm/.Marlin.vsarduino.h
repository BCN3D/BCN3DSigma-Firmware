/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino/Genuino Mega w/ ATmega2560 (Mega 2560), Platform=avr, Package=arduino
*/

#define __AVR_ATmega2560__
#define ARDUINO 168
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define F_CPU 16000000L
#define ARDUINO 168
#define ARDUINO_AVR_MEGA2560
#define ARDUINO_ARCH_AVR
extern "C" void __cxa_pure_virtual() {;}


#include "C:\Program Files (x86)\Arduino\hardware\arduino\avr\variants\mega\pins_arduino.h" 
#include "C:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\arduino.h"
#include <..\Marlin\Marlin.pde>
#include <..\Marlin\BlinkM.cpp>
#include <..\Marlin\BlinkM.h>
#include <..\Marlin\Configuration.h>
#include <..\Marlin\ConfigurationStore.cpp>
#include <..\Marlin\ConfigurationStore.h>
#include <..\Marlin\Configuration_adv.h>
#include <..\Marlin\DOGMbitmaps.h>
#include <..\Marlin\Hysteresis.cpp>
#include <..\Marlin\Hysteresis.h>
#include <..\Marlin\LCD_Handler.h>
#include <..\Marlin\LiquidCrystalRus.cpp>
#include <..\Marlin\LiquidCrystalRus.h>
#include <..\Marlin\Marlin.h>
#include <..\Marlin\Marlin.ino>
#include <..\Marlin\Marlin_main.cpp>
#include <..\Marlin\SD_ListFiles.cpp>
#include <..\Marlin\SD_ListFiles.h>
#include <..\Marlin\Sd2Card.cpp>
#include <..\Marlin\Sd2Card.h>
#include <..\Marlin\Sd2PinMap.h>
#include <..\Marlin\SdBaseFile.cpp>
#include <..\Marlin\SdBaseFile.h>
#include <..\Marlin\SdFatConfig.h>
#include <..\Marlin\SdFatStructs.h>
#include <..\Marlin\SdFatUtil.cpp>
#include <..\Marlin\SdFatUtil.h>
#include <..\Marlin\SdFile.cpp>
#include <..\Marlin\SdFile.h>
#include <..\Marlin\SdInfo.h>
#include <..\Marlin\SdVolume.cpp>
#include <..\Marlin\SdVolume.h>
#include <..\Marlin\Servo.cpp>
#include <..\Marlin\Servo.h>
#include <..\Marlin\Touch_Screen_Definitions.h>
#include <..\Marlin\cardreader.cpp>
#include <..\Marlin\cardreader.h>
#include <..\Marlin\digipot_mcp4451.cpp>
#include <..\Marlin\dogm_font_data_marlin.h>
#include <..\Marlin\dogm_lcd_implementation.h>
#include <..\Marlin\fastio.h>
#include <..\Marlin\genieArduino.cpp>
#include <..\Marlin\genieArduino.h>
#include <..\Marlin\language.cpp>
#include <..\Marlin\language.h>
#include <..\Marlin\motion_control.cpp>
#include <..\Marlin\motion_control.h>
#include <..\Marlin\pins.h>
#include <..\Marlin\planner.cpp>
#include <..\Marlin\planner.h>
#include <..\Marlin\qr_solve.cpp>
#include <..\Marlin\qr_solve.h>
#include <..\Marlin\speed_lookuptable.h>
#include <..\Marlin\stepper.cpp>
#include <..\Marlin\stepper.h>
#include <..\Marlin\temperature.cpp>
#include <..\Marlin\temperature.h>
#include <..\Marlin\thermistortables.h>
#include <..\Marlin\ultralcd.cpp>
#include <..\Marlin\ultralcd.h>
#include <..\Marlin\ultralcd_implementation_hitachi_HD44780.h>
#include <..\Marlin\ultralcd_st7920_u8glib_rrd.h>
#include <..\Marlin\vector_3.cpp>
#include <..\Marlin\vector_3.h>
#include <..\Marlin\watchdog.cpp>
#include <..\Marlin\watchdog.h>
