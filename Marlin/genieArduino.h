/////////////////////// GenieArduino 06/10/2015 ///////////////////////
//
//      Library to utilize the 4D Systems Genie interface to displays
//      that have been created using the Visi-Genie creator platform.
//      This is intended to be used with the Arduino platform.
//
//      Improvements/Updates by
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
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <inttypes.h>

#include <stdint.h>

#ifndef genieArduino_h
#define genieArduino_h

#undef GENIE_DEBUG

#define GENIE_VERSION    "GenieArduino 06-10-2015"

// Genie commands & replys:

#define GENIE_ACK               0x06
#define GENIE_NAK               0x15

#define TIMEOUT_PERIOD          1000
#define RESYNC_PERIOD           100

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

// Structure to store replys returned from a display

#define GENIE_FRAME_SIZE        6

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

#define MAX_GENIE_EVENTS    16    // MUST be a power of 2
#define MAX_GENIE_FATALS    10
#define MAX_LINK_STATES     20

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
    Genie();
    void        Begin               (Stream &serial);
    bool        ReadObject          (uint16_t object, uint16_t index);
    uint16_t    WriteObject         (uint16_t object, uint16_t index, uint16_t data);
    void        WriteContrast       (uint16_t value);
    uint16_t    WriteStr            (uint16_t index, char *string);
	uint16_t	WriteStr			(uint16_t index, long n) ;
	uint16_t	WriteStr			(uint16_t index, long n, int base) ;
	uint16_t	WriteStr			(uint16_t index, unsigned long n) ;
	uint16_t	WriteStr			(uint16_t index, unsigned long n, int base) ;
	uint16_t	WriteStr			(uint16_t index, int n) ;
	uint16_t	WriteStr			(uint16_t index, int n, int base) ;
	uint16_t	WriteStr			(uint16_t index, unsigned int n) ;
	uint16_t	WriteStr			(uint16_t index, unsigned int n, int base) ;
	uint16_t	WriteStr			(uint16_t index, const String &s);
#ifdef AVR
	uint16_t	WriteStr			(uint16_t index, const __FlashStringHelper *ifsh);
#endif
	uint16_t	WriteStr			(uint16_t index, double n, int digits);
	uint16_t	WriteStr			(uint16_t index, double n);	
    uint16_t    WriteStrU           (uint16_t index, uint16_t *string);
    bool        EventIs             (genieFrame * e, uint8_t cmd, uint8_t object, uint8_t index);
    uint16_t    GetEventData        (genieFrame * e);
    bool        DequeueEvent        (genieFrame * buff);
    uint16_t    DoEvents            (bool DoHandler = true);
    void        AttachEventHandler  (UserEventHandlerPtr userHandler);
    void        AttachMagicByteReader (UserBytePtr userHandler);
    void        AttachMagicDoubleByteReader (UserDoubleBytePtr userHandler);
    void        pulse               (int pin);
    void        assignDebugPort     (Stream &port);

    // Genie Magic functions (ViSi-Genie Pro Only)

    uint16_t    WriteMagicBytes     (uint16_t index, uint8_t *bytes, uint16_t len);
    uint16_t    WriteMagicDBytes    (uint16_t index, uint16_t *bytes, uint16_t len);

    uint8_t     GetNextByte         (void);
    uint16_t    GetNextDoubleByte   (void);

private:
    void        FlushEventQueue     (void);
    void        handleError         (void);
    void        SetLinkState        (uint16_t newstate);
    uint16_t    GetLinkState        (void);
    bool        EnqueueEvent        (uint8_t * data);
    uint8_t     Getchar             (void);
    uint16_t    GetcharSerial       (void);
    void        WaitForIdle         (void);
    void        PushLinkState       (uint8_t newstate);
    void        PopLinkState        (void);
    void        FatalError          (void);
    void        FlushSerialInput    (void);
    void        Resync              (void);
	

    //////////////////////////////////////////////////////////////
    // A structure to hold up to MAX_GENIE_EVENTS events receive
    // from the display
    //
    EventQueueStruct EventQueue;

    //////////////////////////////////////////////////////////////
    // Simple 5-deep stack for the link state, this allows
    // DoEvents() to save the current state, receive a frame,
    // then restore the state
    //
    uint8_t LinkStates[MAX_LINK_STATES];
    //
    // Stack pointer
    //
    uint8_t *LinkState;

    //////////////////////////////////////////////////////////////
    // Number of mS the GetChar() function will wait before
    // giving up on the display
    int Timeout;

    //////////////////////////////////////////////////////////////
    // Number of times we have had a timeout
    int Timeouts;

    //////////////////////////////////////////////////////////////
    // Global error variable
    int Error;


    uint8_t    rxframe_count;

    //////////////////////////////////////////////////////////////
    // Number of fatal errors encountered
    int FatalErrors;

    Stream* deviceSerial;
    Stream* debugSerial;

    UserEventHandlerPtr UserHandler;
    UserBytePtr UserByteReader;
    UserDoubleBytePtr UserDoubleByteReader;

};

#ifndef TRUE
#define TRUE    (1==1)
#define FALSE    (!TRUE)
#endif

#define ERROR_NONE           0
#define ERROR_TIMEOUT       -1    // 255  0xFF
#define ERROR_NOHANDLER     -2    // 254  0xFE
#define ERROR_NOCHAR        -3    // 253  0xFD
#define ERROR_NAK           -4    // 252  0xFC
#define ERROR_REPLY_OVR     -5    // 251  0xFB
#define ERROR_RESYNC        -6    // 250  0xFA
#define ERROR_NODISPLAY     -7    // 249  0xF9
#define ERROR_BAD_CS        -8    // 248  0xF8

#define GENIE_LINK_IDLE           0
#define GENIE_LINK_WFAN           1 // waiting for Ack or Nak
#define GENIE_LINK_WF_RXREPORT    2 // waiting for a report frame
#define GENIE_LINK_RXREPORT       3 // receiving a report frame
#define GENIE_LINK_RXEVENT        4 // receiving an event frame
#define GENIE_LINK_SHDN           5
#define GENIE_LINK_RXMBYTES       6 // receiving magic bytes
#define GENIE_LINK_RXMDBYTES      7 // receiving magic dbytes

#define GENIE_EVENT_NONE    0
#define GENIE_EVENT_RXCHAR  1

#endif
