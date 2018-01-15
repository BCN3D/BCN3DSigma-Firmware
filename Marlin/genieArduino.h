/////////////////////// GenieArduino 08/22/2017 ///////////////////////
//
//      Library to utilize the 4D Systems Genie interface to displays
//      that have been created using the Visi-Genie creator platform.
//      This is intended to be used with the Arduino platform.
//
//      Improvements/Updates by
//        4D Systems Engineering, August 2017, www.4dsystems.com.au
//		  	Antonio Brewer & 4D Systems Engineering, July 2017, www.4dsystems.com.au
//        4D Systems Engineering, October 2015, www.4dsystems.com.au
//        4D Systems Engineering, September 2015, www.4dsystems.com.au
//        4D Systems Engineering, August 2015, www.4dsystems.com.au
//        4D Systems Engineering, May 2015, www.4dsystems.com.au
//        Matt Jenkins, March 2015, www.majenko.com
//        Clinton Keith, January 2015, www.clintonkeith.com
//        4D Systems Engineering, July 2014, www.4dsystems.com.au
//        Clinton Keith, March 2014, www.clintonkeith.com
//        Clinton Keith, January 2014, www.clintonkeith.com
//        4D Systems Engineering, January 2014, www.4dsystems.com.au
//        4D Systems Engineering, September 2013, www.4dsystems.com.au
//      Written by
//        Rob Gray (GRAYnomad), June 2013, www.robgray.com
//      Based on code by
//        Gordon Henderson, February 2013, <projects@drogon.net>
//
//      Copyright (c) 2012-2013 4D Systems Pty Ltd, Sydney, Australia
/*********************************************************************
 * This file is part of genieArduino:
 *    genieArduino is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    genieArduino is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with genieArduino.
 *    If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************/
#if defined (SPARK)
  #include "application.h"
  #define lowByte(w) ((uint8_t)((w) & 0xFF))
  #define highByte(w) ((uint8_t)((w) >> 8))
#else
  #if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif
#endif

#include <inttypes.h>

#include <stdint.h>

#ifndef genieArduino_h
#define genieArduino_h

#define GENIE_VERSION    "GenieArduino 08-22-2017"   // MM-DD-YYYY

// Genie commands & replys:

#define GENIE_ACK               0x06
#define GENIE_NAK               0x15
#define GENIE_PING              0x80
#define GENIE_READY             0x81
#define GENIE_DISCONNECTED      0x82

#define GENIE_READ_OBJ          0
#define GENIE_WRITE_OBJ         1
#define GENIE_WRITE_STR         2
#define GENIE_WRITE_STRU        3
#define GENIE_WRITE_CONTRAST    4
#define GENIE_REPORT_OBJ        5
#define GENIE_REPORT_EVENT      7
#define GENIEM_WRITE_BYTES      8
#define GENIEM_WRITE_DBYTES     9
#define GENIEM_REPORT_BYTES     10
#define GENIEM_REPORT_DBYTES    11

// Objects
//    the manual says:
//        Note: Object IDs may change with future releases; it is not
//        advisable to code their values as constants.

#define GENIE_OBJ_DIPSW         0
#define GENIE_OBJ_KNOB          1
#define GENIE_OBJ_ROCKERSW      2
#define GENIE_OBJ_ROTARYSW      3
#define GENIE_OBJ_SLIDER        4
#define GENIE_OBJ_TRACKBAR      5
#define GENIE_OBJ_WINBUTTON     6
#define GENIE_OBJ_ANGULAR_METER 7
#define GENIE_OBJ_COOL_GAUGE    8
#define GENIE_OBJ_CUSTOM_DIGITS 9
#define GENIE_OBJ_FORM          10
#define GENIE_OBJ_GAUGE         11
#define GENIE_OBJ_IMAGE         12
#define GENIE_OBJ_KEYBOARD      13
#define GENIE_OBJ_LED           14
#define GENIE_OBJ_LED_DIGITS    15
#define GENIE_OBJ_METER         16
#define GENIE_OBJ_STRINGS       17
#define GENIE_OBJ_THERMOMETER   18
#define GENIE_OBJ_USER_LED      19
#define GENIE_OBJ_VIDEO         20
#define GENIE_OBJ_STATIC_TEXT   21
#define GENIE_OBJ_SOUND         22
#define GENIE_OBJ_TIMER         23
#define GENIE_OBJ_SPECTRUM      24
#define GENIE_OBJ_SCOPE         25
#define GENIE_OBJ_TANK          26
#define GENIE_OBJ_USERIMAGES    27
#define GENIE_OBJ_PINOUTPUT     28
#define GENIE_OBJ_PININPUT      29
#define GENIE_OBJ_4DBUTTON      30
#define GENIE_OBJ_ANIBUTTON     31
#define GENIE_OBJ_COLORPICKER   32
#define GENIE_OBJ_USERBUTTON    33
// reserved for magic functions 34
#define GENIE_OBJ_ISMARTGAUGE   35
#define GENIE_OBJ_ISMARTSLIDER  36
#define GENIE_OBJ_ISMARTKNOB    37

// Do not modify current values. Recommended settings.


#define DISPLAY_TIMEOUT         2000
#define AUTO_PING_CYCLE         1250


// Structure to store replys returned from a display

#define GENIE_FRAME_SIZE        6 // do NOT touch this.

struct FrameReportObj {
    uint8_t        cmd;
    uint8_t        object;
    uint8_t        index;
    uint8_t        data_msb;
    uint8_t        data_lsb;
};

struct MagicReportHeader {
    uint8_t         cmd;
    uint8_t         index;
    uint8_t         length;
};

/////////////////////////////////////////////////////////////////////
// The Genie frame definition
//
// The union allows the data to be referenced as an array of uint8_t
// or a structure of type FrameReportObj, eg
//
//    genieFrame f;
//    f.bytes[4];
//    f.reportObject.data_lsb
//
//    both methods get the same byte
//
union genieFrame {
    uint8_t             bytes[GENIE_FRAME_SIZE];
    FrameReportObj      reportObject;
};

#define MAX_GENIE_EVENTS    16      // MUST be a power of 2

struct EventQueueStruct {
    genieFrame    frames[MAX_GENIE_EVENTS];
    uint8_t        rd_index;
    uint8_t        wr_index;
    uint8_t        n_events;
};

typedef void        (*UserEventHandlerPtr) (void);
typedef void        (*UserBytePtr)(uint8_t, uint8_t);
typedef void        (*UserDoubleBytePtr)(uint8_t, uint8_t);

/////////////////////////////////////////////////////////////////////
// User API functions
// These function prototypes are the user API to the library
//
class Genie {
public:
    Genie                            ();
    bool        Begin                (Stream &serial);
    void        debug                (Stream &serial, uint8_t level);
    bool        online               ();
    bool        online               (uint8_t activeForm);
    uint32_t    uptime               ();
    uint8_t     form                 ();
    void        form                 (uint8_t newForm);
    void        recover              (uint8_t pulses);
    uint8_t     timeout              (uint16_t value);
    uint8_t     ReadObject           (uint8_t object, uint8_t index);
    uint8_t     WriteObject          (uint8_t object, uint8_t index, uint16_t data);
    uint8_t     WriteContrast        (uint8_t value);
    uint8_t     WriteStr             (uint8_t index, char *string);
	uint8_t	WriteStr			(uint8_t index, long n) ;
	uint8_t	WriteStr			(uint8_t index, long n, int base) ;
	uint8_t	WriteStr			(uint8_t index, unsigned long n) ;
	uint8_t	WriteStr			(uint8_t index, unsigned long n, int base) ;
	uint8_t	WriteStr			(uint8_t index, int n) ;
	uint8_t	WriteStr			(uint8_t index, int n, int base) ;
	uint8_t	WriteStr			(uint8_t index, unsigned int n) ;
	uint8_t	WriteStr			(uint8_t index, unsigned int n, int base) ;
	uint8_t	WriteStr			(uint8_t index, const String &s);
#ifdef AVR
	uint8_t	WriteStr			(uint8_t index, const __FlashStringHelper *ifsh);
#endif
	uint8_t	WriteStr			(uint8_t index, double n, int digits);
	uint8_t	WriteStr			(uint8_t index, double n);	
    uint8_t    WriteStrU             (uint8_t index, uint16_t *string);
    uint8_t     EventIs              (genieFrame * e, uint8_t cmd, uint8_t object, uint8_t index);
    uint16_t    GetEventData         (genieFrame * e);
    uint8_t     EnqueueEvent         (uint8_t * data);
    uint8_t     DequeueEvent         (genieFrame * buff);
    uint8_t     DoEvents             (uint8_t flag);//<-- Editor Alejandro Garcia: Flag for avoid autopinger while we are waiting for an ACK from WriteObject
    uint8_t     autoPinger           ();
    uint16_t    Ping                 (uint16_t interval);
    void        AttachEventHandler             (UserEventHandlerPtr userHandler);
    void        AttachMagicByteReader          (UserBytePtr userHandler);
    void        AttachMagicDoubleByteReader    (UserDoubleBytePtr userHandler);

    // Genie Magic functions (ViSi-Genie Pro Only)

    uint8_t    WriteMagicBytes     (uint8_t index, uint8_t *bytes, uint8_t len, uint8_t report = 0);
    uint8_t    WriteMagicDBytes    (uint8_t index, uint16_t *bytes, uint8_t len, uint8_t report = 0);
    uint8_t    GetNextByte         (void);
    uint16_t   GetNextDoubleByte   (void);


protected:
    //////////////////////////////////////////////////////////////
    // A structure to hold up to MAX_GENIE_EVENTS events receive
    // from the display
    //
    EventQueueStruct EventQueue;

    Stream* deviceSerial;
    Stream* debugSerial;

    UserEventHandlerPtr UserHandler;
    UserBytePtr UserByteReader;
    UserDoubleBytePtr UserDoubleByteReader;

    // used internally by the library, do not modify!
    volatile bool     pendingACK = 0; // prevent userhandler if waiting for ACK, and end recursion.
    volatile bool     pingRequest = 0; // used internally by the library, do not touch.
    volatile uint8_t  recover_pulse = 50; // pulse for offline autoping, use genie.recover(x) to change it from sketch.
    volatile bool     autoPing = 0; // used internally by the library, do not touch.
    volatile uint16_t GENIE_CMD_TIMEOUT = 1250; // force disconnection trigger if ACK times out
    volatile uint32_t autoPingTimer = millis(); // timer for autoPinger() function
    volatile bool     displayDetected = 0; // display is online/offline state
    volatile uint32_t displayDetectTimer = millis(); // timer for lcd to be aware if connected
    volatile uint8_t  currentForm = -1; // current form thats loaded
    volatile uint8_t  nakInj = 0; // nak injection counter
    volatile uint8_t  badByteCounter = 0; // used for disconnection/debugging purposes
    volatile uint32_t delayedCycles = millis(); // session protection if latency in user code
    volatile uint32_t display_uptime = 0; // uptime of display
    volatile uint32_t ping_spacer; // prevent flooding the uart during recovery. non-blocking.
    volatile bool     genieStart = 1;
    volatile uint8_t  debug_level = 0;
};
#endif