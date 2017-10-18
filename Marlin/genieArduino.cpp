/////////////////////// GenieArduino 08/22/2017 ///////////////////////
//
//      Library to utilise the 4D Systems Genie interface to displays
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
//      Copyright (c) 2012-2014 4D Systems Pty Ltd, Sydney, Australia
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

#include "genieArduino.h"
#include <math.h>
#include <string.h>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2


// ######################################
// ## GENIE CLASS ####################### 
// ######################################
Genie::Genie() {
  UserHandler = NULL;
  UserByteReader = NULL;
  UserDoubleByteReader = NULL;
  debugSerial = NULL;
}



// ######################################
// ## Attach Event Handler ############## 
// ######################################
void Genie::AttachEventHandler (UserEventHandlerPtr handler) {
  UserHandler = handler;
  uint8_t rx_data[6];
  // display status already collected from Begin function, user just enabled handler, so give a status.
  if ( displayDetected ) {
    rx_data[0] = GENIE_PING; rx_data[1] = GENIE_READY; rx_data[2] = 0; rx_data[3] = 0; rx_data[4] = 0; rx_data[5] = 0;
    EnqueueEvent(rx_data); // send ready state to user handler.
  }
  else {
    rx_data[0] = GENIE_PING; rx_data[1] = GENIE_DISCONNECTED; rx_data[2] = 0; rx_data[3] = 0; rx_data[4] = 0; rx_data[5] = 0;
    EnqueueEvent(rx_data);
  }
}



// ######################################
// ## Attach Magic Byte Reader ########## 
// ######################################
void Genie::AttachMagicByteReader(UserBytePtr handler) {
  UserByteReader = handler;
}



// ######################################
// ## Attach Magic Double Byte Reader ### 
// ######################################
void Genie::AttachMagicDoubleByteReader(UserDoubleBytePtr handler) {
  UserDoubleByteReader = handler;
}



// ######################################
// ## Genie Uptime Request ############## 
// ######################################
uint32_t Genie::uptime() {
  if ( displayDetected ) return millis() - display_uptime;
  else return 0;
}



// ######################################
// ## GetNextByte ####################### 
// ######################################
uint8_t Genie::GetNextByte() {
  if ( !displayDetected ) return -1; // user code may keep requesting, block till ready.
  uint8_t rx_data[6];
  uint32_t timeout = millis();
  while (deviceSerial->available() < 1) {
    delayedCycles = millis();
    displayDetectTimer = millis();
    if ( millis() - timeout >= 2000 ) { // we issue an immediate manual disconnect.
      displayDetectTimer = millis();
      displayDetected = 0;
      while ( deviceSerial->available() > 0 ) deviceSerial->read();
      if ( debugSerial != NULL ) { debugSerial->println(F("*** Genie Disconnected from GetNextByte! ***")); }
      rx_data[0] = GENIE_PING; rx_data[1] = GENIE_DISCONNECTED; rx_data[2] = 0; rx_data[3] = 0; rx_data[4] = 0; rx_data[5] = 0;
      EnqueueEvent(rx_data);
      return -1;
    }
    continue;
  }
  delayedCycles = millis();
  displayDetectTimer = millis();
  return deviceSerial->read();
}



// ######################################
// ## GetNextDoubleByte ################# 
// ######################################
uint16_t Genie::GetNextDoubleByte() {
  if ( !displayDetected ) return -1; // user code may keep requesting, block till ready.
  uint8_t rx_data[6];
  uint16_t out;
  uint32_t timeout = millis();
  while (deviceSerial->available() < 2) {
    delayedCycles = millis();
    displayDetectTimer = millis();
    if ( millis() - timeout >= 2000 ) { // we issue an immediate manual disconnect.
      displayDetectTimer = millis();
      displayDetected = 0;
      while ( deviceSerial->available() > 0 ) deviceSerial->read();
      if ( debugSerial != NULL ) { debugSerial->println(F("*** Genie Disconnected from GetNextByte! ***")); }
      rx_data[0] = GENIE_PING; rx_data[1] = GENIE_DISCONNECTED; rx_data[2] = 0; rx_data[3] = 0; rx_data[4] = 0; rx_data[5] = 0;
      EnqueueEvent(rx_data);
      return -1;
    }
    continue;
  }
  delayedCycles = millis();
  displayDetectTimer = millis();
  out = (deviceSerial->read()) << 8;
  out |= deviceSerial->read();
  return out;
}



// ######################################
// ## Setup ############################# 
// ######################################
bool Genie::Begin(Stream &serial) {
  deviceSerial = &serial;
  display_uptime = millis(); // start uptime timer (ms)
  genieStart = 1; // start form request on startup
  ReadObject(GENIE_OBJ_FORM, (uint8_t)0x00); // send form request
  uint32_t timeout_start = millis(); // timeout timer
  while ( millis() - timeout_start <= 250 ) { // blocking loop, releases after 150ms to timeout, or sooner if display's detected.
   if ( DoEvents(0) == GENIE_REPORT_OBJ && !genieStart ) return 1; // form is updated.
  }
  displayDetected = 0;
  return 0; // timeout occured, status offline.
}



// ######################################
// ## Debug ############################# 
// ######################################
void Genie::debug(Stream &serial, uint8_t level) {
  debugSerial = &serial;
  debug_level = level;
}



// ######################################
// ## Timeout ########################### 
// ######################################
uint8_t Genie::timeout(uint16_t value) {
    if ( value < 50 ) return 0; // no less than 50 recommended! this will trigger the disconnect flag!
    GENIE_CMD_TIMEOUT = value;
    return 1;
}



// ######################################
// ## Online Status ##################### 
// ######################################
bool Genie::online() {
  return displayDetected;
}



// ######################################
// ## Online and Active Form ############ 
// ######################################
bool Genie::online(uint8_t activeForm) {
  if ( displayDetected && currentForm == activeForm ) return 1;
  return 0;
}



// ######################################
// ## Current Form ###################### 
// ######################################
uint8_t Genie::form() {
  return currentForm;
}



// ######################################
// ## Change Form ####################### 
// ######################################
void Genie::form(uint8_t newForm) {
  WriteObject(GENIE_OBJ_FORM, newForm, (uint8_t)0x00);
}



// ######################################
// ## Recovery Pulses ################### 
// ######################################
void Genie::recover(uint8_t pulses) {
  recover_pulse = pulses;
}



// ######################################
// ## Get Event Data #################### 
// ######################################
uint16_t Genie::GetEventData (genieFrame * e) {
  return (e->reportObject.data_msb << 8) + e->reportObject.data_lsb;
}



// ######################################
// ## Event Is ########################## 
// ######################################
uint8_t Genie::EventIs(genieFrame * e, uint8_t cmd, uint8_t object, uint8_t index) {
  return (e->reportObject.cmd == cmd &&
          e->reportObject.object == object &&
          e->reportObject.index == index);
}



// ######################################
// ## Read Object ####################### 
// ######################################
uint8_t Genie::ReadObject(uint8_t object, uint8_t index) {
  if ( !displayDetected ) return -1;
  uint8_t checksum; 
  deviceSerial->write((uint8_t)GENIE_READ_OBJ);
  checksum = GENIE_READ_OBJ;
  deviceSerial->write(object);
  checksum ^= object;
  deviceSerial->write(index);
  checksum ^= index;
  deviceSerial->write(checksum);
  return 0;
}



// ######################################
// ## Write Object ######################
// ######################################
uint8_t Genie::WriteObject(uint8_t object, uint8_t index, uint16_t data) {
  if ( !displayDetected ) return -1;
  uint8_t checksum;
  pendingACK = 1;
  deviceSerial->write(GENIE_WRITE_OBJ); checksum = GENIE_WRITE_OBJ;
  deviceSerial->write(object); checksum ^= object;
  deviceSerial->write(index); checksum ^= index;
  deviceSerial->write(highByte(data)); checksum ^= highByte(data);
  deviceSerial->write(lowByte(data)); checksum ^= lowByte(data);
  deviceSerial->write(checksum);
  uint32_t timeout_write = millis();
  while ( millis() - timeout_write <= GENIE_CMD_TIMEOUT ) {
    uint8_t command_return = DoEvents(GENIE_ACK);
    if ( command_return == GENIE_ACK ) { if ( ( debug_level == 6 || debug_level == 1 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie WriteObject *** --->")); debugSerial->print(object); debugSerial->println(F("... ACK !")); } return 1; }
    if ( command_return == GENIE_NAK ) { if ( ( debug_level == 6 || debug_level == 1 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie WriteObject *** --->")); debugSerial->print(object); debugSerial->println(F("... NAK !")); } return 0; }
  }
  displayDetectTimer = millis() + DISPLAY_TIMEOUT + 10000; //manual disconnect
  if ( ( debug_level == 6 || debug_level == 1 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie WriteObject *** --->")); debugSerial->print(object); debugSerial->println(F("... Timeout!")); }
  return -1; // timeout
}



// ######################################
// ## Write Contrast #################### 
// ######################################
uint8_t Genie::WriteContrast(uint8_t value) {
  if ( !displayDetected ) return -1;
  uint8_t checksum;
  pendingACK = 1;
  deviceSerial->write(GENIE_WRITE_CONTRAST);
  checksum = GENIE_WRITE_CONTRAST;
  deviceSerial->write(value);
  checksum ^= value;
  deviceSerial->write(checksum);
  uint32_t timeout_write = millis();
  while ( millis() - timeout_write <= GENIE_CMD_TIMEOUT ) {
    uint8_t command_return = DoEvents(GENIE_ACK);
    if ( command_return == GENIE_ACK ) { if ( ( debug_level == 6 || debug_level == 1 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie Contrast *** --->")); debugSerial->println(F("... ACK !")); } return 1; }
    if ( command_return == GENIE_NAK ) { if ( ( debug_level == 6 || debug_level == 1 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie Contrast *** --->")); debugSerial->println(F("... NAK !")); } return 0; }
  }
  displayDetectTimer = millis() + DISPLAY_TIMEOUT + 10000; //manual disconnect
  if ( ( debug_level == 6 || debug_level == 1 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie Contrast *** --->")); debugSerial->println(F("... Timeout!")); }
  return -1; // timeout
}



// ######################################
// ## Auto Pinger #######################
// ######################################
uint8_t Genie::autoPinger() {
  uint16_t geniePingTimerChanger;
  if ( displayDetected ) geniePingTimerChanger = AUTO_PING_CYCLE; // preset online pinger
  if ( !displayDetected ) geniePingTimerChanger = recover_pulse; // 50ms offline pinger
  if ( millis() - autoPingTimer > geniePingTimerChanger ) {
    autoPingTimer = millis();
    autoPing = 1;
    uint8_t checksum;
    deviceSerial->write((uint8_t)GENIE_READ_OBJ);
    checksum = GENIE_READ_OBJ;
    deviceSerial->write((uint8_t)GENIE_OBJ_FORM);
    checksum ^= (uint8_t)GENIE_OBJ_FORM;
    deviceSerial->write((uint8_t)0x00);
    checksum ^= (uint8_t)0x00;
    deviceSerial->write(checksum);
	//if ( ( debug_level == 6 || debug_level == 1 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie AutoPinger *** --->"));debugSerial->println(F("... Sending !")); }
  }
  return 1;
}



// ######################################
// ## User Ping #########################
// ######################################
uint16_t Genie::Ping(uint16_t interval) {
  if ( displayDetected ) {
    if ( millis() - ping_spacer < interval ) return 0;
    ping_spacer = millis();
    pingRequest = 1;
    uint8_t checksum;
    deviceSerial->write((uint8_t)GENIE_READ_OBJ);
    checksum = GENIE_READ_OBJ;
    deviceSerial->write((uint8_t)GENIE_OBJ_FORM);
    checksum ^= (uint8_t)GENIE_OBJ_FORM;
    deviceSerial->write((uint8_t)0x00);
    checksum ^= (uint8_t)0x00;
    deviceSerial->write(checksum);
    return 1;
  }
  if ( !displayDetected ) {
    if ( millis() - ping_spacer > interval ) {
      ping_spacer = millis();
      uint8_t rx_data[6];
      rx_data[0] = GENIE_PING; rx_data[1] = GENIE_NAK; rx_data[2] = 0;
      rx_data[3] = 0; rx_data[4] = 0; rx_data[5] = 0;
      EnqueueEvent(rx_data);
    }
  }
  return 0;
}


// ######################################
// ## Do Events #########################
// ######################################
uint8_t Genie::DoEvents(uint8_t flag) {
  
  if(flag != GENIE_ACK) autoPinger(); // used to keep lcd connection alive

  uint8_t rx_data[6]; // array for receiving command, payload, and crc.
  uint8_t checksumVerify; // used to calculate a matching (or not) checksum.

  // ######################################
  // ## SLOW USER CODE? NO PROBLEM! #######
  // ######################################

  if ( millis() - delayedCycles >= DISPLAY_TIMEOUT ) {
    displayDetectTimer = millis(); // reset counter to prevent false disconnections.
    if ( debugSerial != NULL ) debugSerial->println(F("*** Genie Took too long to re-enter DoEvents(), reduce delays in code, self-protection enabled. ***")); 
  }
  delayedCycles = millis(); // reset the doevents function timeout, every cycle.


  if ( online() ) {
    if ( millis() - displayDetectTimer > DISPLAY_TIMEOUT ) { // code online, but lcd is not?
      displayDetectTimer = millis();
      displayDetected = 0;
      pingRequest = 0;
      if ( debugSerial != NULL ) { debugSerial->println(F("*** Genie Disconnected from DoEvents! ***")); }
      rx_data[0] = GENIE_PING; rx_data[1] = GENIE_DISCONNECTED; rx_data[2] = 0; rx_data[3] = 0; rx_data[4] = 0; rx_data[5] = 0;
      EnqueueEvent(rx_data);
      currentForm = -1; // reset form holder
      return -1;
    }
  }

  if ( !displayDetected  ) {                         // not online?
    pendingACK = 0;                          // reset pending ACK check
    currentForm = -1;                        // reset form holder
    display_uptime = 0;               // keeps timer reset
  }



  // ######################################
  // ## Main State Machine ################
  // ######################################

  if ( deviceSerial->available() > 0 ) {
    uint8_t b = deviceSerial->peek(); // Look at the next byte but don't pull it yet.
	//if ( ( debug_level == 6 || debug_level == 1 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie PrePull *** --->")); debugSerial->print(b); debugSerial->println(F("... Byte !")); }
    if ( !displayDetected && ( b == GENIEM_REPORT_BYTES || b == GENIEM_REPORT_DBYTES ) ) b = 0xFF; // force bad bytes instead of false triggering genie magic switches.
    switch ( b ) { // We're going to parse what we see into the proper switch.



      case GENIE_ACK:
        displayDetectTimer = millis(); // reset display timeout since the packet is good.
        deviceSerial->read(); // remove ACK
        badByteCounter = 0; // reset the bad byte counter
        pendingACK = 0;
        nakInj = 0; // reset NAK counter
        return GENIE_ACK; 



      case GENIE_NAK:
        displayDetectTimer = millis(); // reset display timeout since the packet is good.
        deviceSerial->read(); // remove NAK
        nakInj++; // increment consecutive NAK counter.
        while ( deviceSerial->peek() == GENIE_NAK ) deviceSerial->read(); // remove trailing naks for next test
        if ( nakInj >= 2 ) { // if NAK's are consecutive 2 or more times...
          nakInj = 0; // reset the counter
          if ( debugSerial != NULL ) debugSerial->print(F("*** Genie NAK Recovery !!! ***"));
          deviceSerial->write(0xFF); // inject a byte into the tx buffer to attempt recovery.
        }
        pendingACK = 0;
        return GENIE_NAK;



      case GENIEM_REPORT_BYTES:
        if ( deviceSerial->available() < 3 ) break; // magic report event less than 3 bytes? check again.
        rx_data[0] = deviceSerial->read(); rx_data[1] = deviceSerial->read(); rx_data[2] = deviceSerial->read();
        displayDetectTimer = millis(); // reset display timeout since the packet is good.
        badByteCounter = 0; // reset the bad byte counter
        if ( ( debug_level == 6 || debug_level == 5 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie Received Magic Bytes *** Index: ")); debugSerial->print(rx_data[1]); debugSerial->print(F(" Length: ")); debugSerial->println(rx_data[2]); }
        if ( UserByteReader != NULL ) UserByteReader( rx_data[1], rx_data[2] );
        else for ( int i = 0; i < rx_data[2]; i++) deviceSerial->read();
        (void)GetNextByte();
        return GENIEM_REPORT_BYTES;



      case GENIEM_REPORT_DBYTES:
        if ( deviceSerial->available() < 3 ) break; // magic report event less than 3 bytes? check again.
        rx_data[0] = deviceSerial->read(); rx_data[1] = deviceSerial->read(); rx_data[2] = deviceSerial->read();
        displayDetectTimer = millis(); // reset display timeout since the packet is good.
        badByteCounter = 0; // reset the bad byte counter
        if ( ( debug_level == 6 || debug_level == 5 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie Received Magic Double Bytes *** Index: ")); debugSerial->print(rx_data[1]); debugSerial->print(F(" Length: ")); debugSerial->println(rx_data[2]); }
        if ( UserDoubleByteReader != NULL ) UserDoubleByteReader( rx_data[1], rx_data[2] );
        else for ( int i = 0; i < 2 * rx_data[2]; i++) deviceSerial->read();
        (void)GetNextByte();
        return GENIEM_REPORT_DBYTES;



      case GENIE_REPORT_EVENT:
        if ( deviceSerial->available() < 6 ) break; // report event less than 6 bytes? check again.
        rx_data[0] = deviceSerial->read(); rx_data[1] = deviceSerial->read();
        rx_data[2] = deviceSerial->read(); rx_data[3] = deviceSerial->read();
        rx_data[4] = deviceSerial->read(); rx_data[5] = deviceSerial->read();
        checksumVerify = rx_data[0]; checksumVerify ^= rx_data[1]; checksumVerify ^= rx_data[2]; checksumVerify ^= rx_data[3]; checksumVerify ^= rx_data[4];
        if ( checksumVerify != rx_data[5] ) return 0; //discard this packet, CRC is bad.
        displayDetectTimer = millis(); // reset display timeout since the packet is good.
        badByteCounter = 0; // reset the bad byte counter
        if ( rx_data[1] == GENIE_OBJ_FORM ) currentForm = rx_data[2];
        if ( ( debug_level == 6 || debug_level == 2 ) && debugSerial != NULL ) {
          debugSerial->print(F("*** Genie Report Event *** Object: "));
          debugSerial->print(rx_data[1]);
          debugSerial->print(F(" Index: "));
          debugSerial->print(rx_data[2]);
          debugSerial->print(F(" Value: "));
          debugSerial->println(256U*rx_data[3]+rx_data[4]);
        }
        EnqueueEvent(rx_data); return GENIE_REPORT_EVENT;



      case GENIE_REPORT_OBJ:
        if ( deviceSerial->available() < 6 ) break; // report event less than 6 bytes? check again.
        rx_data[0] = deviceSerial->read(); rx_data[1] = deviceSerial->read();
        rx_data[2] = deviceSerial->read(); rx_data[3] = deviceSerial->read();
        rx_data[4] = deviceSerial->read(); rx_data[5] = deviceSerial->read();
        checksumVerify = rx_data[0]; checksumVerify ^= rx_data[1]; checksumVerify ^= rx_data[2]; checksumVerify ^= rx_data[3]; checksumVerify ^= rx_data[4];
        if ( checksumVerify != rx_data[5] ) return 0; //discard this packet, CRC is bad.
        displayDetectTimer = millis(); // reset display timeout since the packet is good.
        badByteCounter = 0; // reset the bad byte counter
        if ( rx_data[1] == GENIE_OBJ_FORM ) currentForm = rx_data[4];
        // if ( genieStart ) { genieStart = 0; return GENIE_REPORT_OBJ; } // disable startup form checker

        if ( ( autoPing || pingRequest ) && rx_data[1] == GENIE_OBJ_FORM ) {
          if ( autoPing ) {
            autoPing = 0; //switch off after queueing event
            if ( !displayDetected ) { // if previously disconnected and now is connected...
              display_uptime = millis(); // start uptime timer (ms)
              if ( debugSerial != NULL ) { debugSerial->println(F("*** Genie Connected / Sync! ***")); }
              rx_data[0] = GENIE_PING; rx_data[1] = GENIE_READY; rx_data[2] = 0; rx_data[3] = 0; rx_data[4] = 0; rx_data[5] = 0;
              EnqueueEvent(rx_data); // send ready state to user handler.
              while ( deviceSerial->available() > 0 ) deviceSerial->read(); // clear on new connect
              displayDetected = 1; // turn on functions
            }
            if ( genieStart ) { genieStart = 0; return GENIE_REPORT_OBJ; } 
            break;
          }

          if ( pingRequest ) {
            pingRequest = 0; //switch off after queueing event
            rx_data[0] = GENIE_PING; rx_data[1] = GENIE_ACK; rx_data[2] = 0; rx_data[3] = 0; rx_data[4] = 0; rx_data[5] = 0;
            EnqueueEvent(rx_data); // send ACK to user ping request in handler.
          }
          break;
        }

        if ( ( debug_level == 6 || debug_level == 3 ) && debugSerial != NULL ) {
          debugSerial->print(F("*** Genie Report Object *** Object: "));
          debugSerial->print(rx_data[1]);
          debugSerial->print(F(" Index: "));
          debugSerial->print(rx_data[2]);
          debugSerial->print(F(" Value: "));
          debugSerial->println(256U*rx_data[3]+rx_data[4]);
        }
        EnqueueEvent(rx_data); return GENIE_REPORT_OBJ; // all other reading of object data flow to event handler



      default: // unknown bytes found, shift out and start count for possible disconnection.
        uint8_t bad_byte = deviceSerial->read();
        if ( debugSerial != NULL ) { debugSerial->print(F("*** Genie Bad Byte *** ---> 0x")); debugSerial->println(bad_byte,HEX); }
        badByteCounter++; // We count consecutively to 10 bytes in a row and assume display offline.
        if ( badByteCounter > 10 ) { badByteCounter = 0; displayDetectTimer = millis() + DISPLAY_TIMEOUT + 10000; } // let DoEvents do the disconnection.
        return GENIE_NAK;
    }
  }
  if ( !pendingACK && EventQueue.n_events > 0 && UserHandler != NULL ) UserHandler(); // trigger userhandler if queues exist.
  return 0;
}



// ######################################
// ## DeQueue Event #####################
// ######################################
uint8_t Genie::DequeueEvent(genieFrame * buff) {
  if (EventQueue.n_events > 0) {
    memcpy(buff, &EventQueue.frames[EventQueue.rd_index], GENIE_FRAME_SIZE);
    EventQueue.rd_index++;
    EventQueue.rd_index &= MAX_GENIE_EVENTS - 1;
    EventQueue.n_events--;
  }
  return 0;
}



// ######################################
// ## EnQueue Event #####################
// ######################################
uint8_t Genie::EnqueueEvent (uint8_t * data) { 
  if ( UserHandler == NULL ) { if ( debugSerial != NULL ) debugSerial->println(F("*** Genie Enable AttachEventHandler !!! ***")); return -1; }
  if (EventQueue.n_events < MAX_GENIE_EVENTS - 2) {
    int i, j;
    bool fnd = 0;
    j = EventQueue.wr_index;
    for (i = EventQueue.n_events; i > 0; i--) {
      j--;
      if (j < 0) j = MAX_GENIE_EVENTS - 1;
      if ( ( EventQueue.frames[j].reportObject.cmd == data[0] ) &&
           ( EventQueue.frames[j].reportObject.object == data[1] ) &&
           ( EventQueue.frames[j].reportObject.index == data[2] ) ) {
        EventQueue.frames[j].reportObject.data_msb = data[3];
        EventQueue.frames[j].reportObject.data_lsb = data[4];
        fnd = 1;
        break;
      }
    }
    if (!fnd) {
      memcpy(&EventQueue.frames[EventQueue.wr_index], data, GENIE_FRAME_SIZE);
      EventQueue.wr_index++;
      EventQueue.wr_index &= MAX_GENIE_EVENTS - 1;
      EventQueue.n_events++;
    }
  }
  return 0;
}



// ######################################
// ## Write Strings #####################
// ######################################

uint8_t Genie::WriteStr(uint8_t index, char *string) {
  if ( !displayDetected ) return -1;
  char *p;
  uint8_t checksum;
  pendingACK = 1;
  int len = strlen(string);
  if (len > 255) return -1;
  deviceSerial->write(GENIE_WRITE_STR);
  checksum = GENIE_WRITE_STR;
  deviceSerial->write(index);
  checksum ^= index;
  deviceSerial->write((unsigned char)len);
  checksum ^= len;
  for (p = string ; *p ; ++p) {
    deviceSerial->write(*p);
    checksum ^= *p;
  }
  deviceSerial->write(checksum);
  uint32_t timeout_write = millis();
  while ( millis() - timeout_write <= GENIE_CMD_TIMEOUT ) {
    uint8_t command_return = DoEvents(GENIE_ACK);
    if ( command_return == GENIE_ACK ) return 1;
    if ( command_return == GENIE_NAK ) return 0;
  }
  return -1; // timeout
}



#ifdef AVR
uint8_t Genie::WriteStr(uint8_t index, const __FlashStringHelper *ifsh) {
  PGM_P p = reinterpret_cast<PGM_P>(ifsh);
  PGM_P p2 = reinterpret_cast<PGM_P>(ifsh);
  size_t n = 0;
  int len = 0;
  while (1) {
    unsigned char d = pgm_read_byte(p2++);
    len++;
    if (d == 0) break;
  }
  char arr[len];
  int x = 0;
  while (1) {
    unsigned char c = pgm_read_byte(p++);
    arr[x] = c;
    x++;
    if (c == 0) break;
  }
  WriteStr(index, arr);
  return 0;
}
#endif

uint8_t Genie::WriteStr(uint8_t index, const String &s) {
  int len = s.length();
  char arr[len + 1];
  s.toCharArray(arr, len + 1);
  WriteStr(index, arr);
  return 0;
}

uint8_t Genie::WriteStr (uint8_t index, long n) {
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];
  long N = n;
  n = abs(n);
  *str = '\0';
  do {
    unsigned long m = n;
    n /= 10;
    char c = m - 10 * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while (n);
  if (N < 0) *--str = '-';
  WriteStr(index, str);
  return 0;
}

uint8_t Genie::WriteStr (uint8_t index, long n, int base) {
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];
  long N;
  *str = '\0';
  if (n >= 0) {
    // prevent crash if called with base == 1
    if (base < 2) base = 10;
    if (base == 10) {
      N = n;
      n = abs(n);
    }
    do {
      unsigned long m = n;
      n /= base;
      char c = m - base * n;
      *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while (n);
    if (base == 10) if (N < 0) *--str = '-';
  }
  else if (n < 0) {
    unsigned long n2 = (unsigned long)n;
    uint8_t base2 = base;
    do {
      unsigned long m = n2;
      n2 /= base2;
      char c = m - base2 * n2;
      *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while (n2);
  }
  WriteStr(index, str);
  return 0;
}

uint8_t Genie::WriteStr (uint8_t index, int n) {
  WriteStr (index, (long) n);
  return 0;
}

uint8_t Genie::WriteStr (uint8_t index, int n, int base) {
  WriteStr (index, (long) n, base);
  return 0;
}

uint8_t Genie::WriteStr (uint8_t index, unsigned long n) {
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];
//  long N = n; // unused?
  n = abs(n);
  *str = '\0';
  do {
    unsigned long m = n;
    n /= 10;
    char c = m - 10 * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while (n);
  WriteStr(index, str);
  return 0;
}

uint8_t Genie::WriteStr (uint8_t index, unsigned long n, int base) {
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];
  *str = '\0';
  // prevent crash if called with base == 1
  if (base < 2) base = 10;
  do {
    unsigned long m = n;
    n /= base;
    char c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while (n);
  WriteStr(index, str);
  return 0;
}

uint8_t Genie::WriteStr (uint8_t index, unsigned int n) {
  WriteStr (index, (unsigned long) n);
  return 0;
}

uint8_t Genie::WriteStr (uint8_t index, unsigned n, int base) {
  WriteStr (index, (unsigned long) n, base);
  return 0;
}

uint8_t Genie::WriteStr (uint8_t index, double number, int digits) {
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];
  *str = '\0';
  double number2 = number;
  if (number < 0.0) number = -number;
  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (int i = 0; i < digits; ++i) rounding /= 10.0;
  number += rounding;
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  // Extract digits from the remainder one at a time
  int digits2 = digits;
  str = &buf[sizeof(buf) - 1 - digits2];
  while (digits2-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    char c = toPrint + 48;
    *str++ = c;
    remainder -= toPrint;
  }
  str = &buf[sizeof(buf) - 1 - digits];
  if (digits > 0) *--str = '.';
  // Extract the integer part of the number and print it
  do {
    unsigned long m = int_part;
    int_part /= 10;
    char c = m - 10 * int_part;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while (int_part);
  // Handle negative numbers
  if (number2 < 0.0) *--str = '-';
  WriteStr(index, str);
  return 0;
}

uint8_t Genie::WriteStr (uint8_t index, double n) {
  WriteStr(index, n, 2);
}

uint8_t Genie::WriteStrU (uint8_t index, uint16_t *string) {
  if ( !displayDetected ) return -1;
  uint16_t *p;
  uint8_t checksum;
  int len = 0;
  p = string;
  while (*p++) len++;
  if (len > 255) return -1;
  deviceSerial->write(GENIE_WRITE_STRU);
  checksum  = GENIE_WRITE_STRU;
  deviceSerial->write(index);
  checksum ^= index;
  deviceSerial->write((unsigned char)(len));
  checksum ^= (len);
  p = string;
  while (*p) {
    deviceSerial->write (*p >> 8);
    checksum ^= *p >> 8;
    deviceSerial->write (*p);
    checksum ^= *p++ & 0xff;
  }
  deviceSerial->write(checksum);
  return 1;
}



// ######################################
// ## Write Magic Bytes #################
// ######################################

uint8_t Genie::WriteMagicBytes (uint8_t index, uint8_t *bytes, uint8_t len, uint8_t report) {
  if ( UserByteReader == NULL ) { if ( ( debug_level == 6 || debug_level == 5 ) && debugSerial != NULL ) debugSerial->println(F("*** Genie Enable AttachMagicByteReader !!! ***")); return -1; }
  if ( !displayDetected ) return -1;
  uint8_t checksum;
  deviceSerial->write(GENIEM_WRITE_BYTES);
  checksum = GENIEM_WRITE_BYTES;
  deviceSerial->write(index);
  checksum ^= index;
  deviceSerial->write(len);
  checksum ^= len;
  for (int i = 0; i < len; i++) {
    deviceSerial->write(bytes[i]);
    checksum ^= bytes[i];
  }
  deviceSerial->write(checksum);
  uint32_t timeout_write = millis();
  if (!report) pendingACK = 1;

  while ( millis() - timeout_write <= GENIE_CMD_TIMEOUT ) {
    uint8_t command_return = DoEvents(GENIE_ACK);
    if (!report) {
      if ( command_return == GENIE_ACK ) { if ( ( debug_level == 6 || debug_level == 5 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie WriteMagicBytes *** --->")); debugSerial->print(index); debugSerial->println(F("... ACK !")); } return 1; }
      if ( command_return == GENIE_NAK ) { if ( ( debug_level == 6 || debug_level == 5 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie WriteMagicBytes *** --->")); debugSerial->print(index); debugSerial->println(F("... NAK !")); } return 0; }
    } else {
      if ( command_return == report) { if ( ( debug_level == 6 || debug_level == 5 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie WriteMagicBytes *** --->")); debugSerial->print(index); debugSerial->println(F("... OK  !")); } return 1; }
    }
  }

  displayDetectTimer = millis() + DISPLAY_TIMEOUT + 10000; //manual disconnect
  if ( ( debug_level == 6 || debug_level == 5 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie WriteMagicBytes *** --->")); debugSerial->print(index); debugSerial->println(F("... Timeout!")); }
  return -1; // timeout
}

// ######################################
// ## Write Magic Double Bytes ##########
// ######################################

uint8_t Genie::WriteMagicDBytes(uint8_t index, uint16_t *shorts, uint8_t len, uint8_t report) {
  if ( UserDoubleByteReader == NULL ) { if ( ( debug_level == 6 || debug_level == 5 ) && debugSerial != NULL ) debugSerial->println(F("*** Genie Enable AttachMagicDoubleByteReader !!! ***")); return -1; }
  if ( !displayDetected ) return -1;
  uint8_t checksum;
  deviceSerial->write(GENIEM_WRITE_DBYTES);
  checksum = GENIEM_WRITE_DBYTES;
  deviceSerial->write(index);
  checksum ^= index;
  deviceSerial->write(len);
  checksum ^= len;
  for (int i = 0; i < len; i++) {
    deviceSerial->write (shorts[i] >> 8);
    checksum ^= shorts[i] >> 8;
    deviceSerial->write (shorts[i] & 0xFF);
    checksum ^= shorts[i] & 0xff;
  }
  deviceSerial->write(checksum);
  uint32_t timeout_write = millis();


  if (!report) pendingACK = 1;

  while ( millis() - timeout_write <= GENIE_CMD_TIMEOUT ) {
    uint8_t command_return = DoEvents(GENIE_ACK);
    if (!report) {
      if ( command_return == GENIE_ACK ) { if ( ( debug_level == 6 || debug_level == 5 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie WriteMagicDBytes *** --->")); debugSerial->print(index); debugSerial->println(F("... ACK !")); } return 1; }
      if ( command_return == GENIE_NAK ) { if ( ( debug_level == 6 || debug_level == 5 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie WriteMagicDBytes *** --->")); debugSerial->print(index); debugSerial->println(F("... NAK !")); } return 0; }
    } else {
      if ( command_return == report) { if ( ( debug_level == 6 || debug_level == 5 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie WriteMagicDBytes *** --->")); debugSerial->print(index); debugSerial->println(F("... OK  !")); } return 1; }
    }
  }

  displayDetectTimer = millis() + DISPLAY_TIMEOUT + 10000; //manual disconnect
  if ( ( debug_level == 6 || debug_level == 5 ) && debugSerial != NULL ) { debugSerial->print(F("*** Genie WriteMagicDBytes *** --->")); debugSerial->print(index); debugSerial->println(F("... Timeout!")); }
  return -1; // timeout
}