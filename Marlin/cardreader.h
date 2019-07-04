/*
- cardreader.h - This library manage the communication between MCU and SD card module
Last Update: 01/08/2018
Author: Alejandro Garcia (S3mt0x)
*/
#ifndef CARDREADER_H
#define CARDREADER_H

#ifdef SDSUPPORT

#define MAX_DIR_DEPTH 10

#include "SdFile.h"
enum LsAction {LS_SerialPrint,LS_Count,LS_GetFilename};
class CardReader
{
public:
  CardReader();
  
  void initsd();
  void write_command(char *buf);
  //files auto[0-9].g on the sd card are performed in a row
  //this is to delay autostart and hence the initialisaiton of the sd card to some seconds after the normal init, so the device is available quick after a reset

  void checkautostart(bool x); 
  void openFile(char* name,bool read,bool replace_current=true);
  void openLogFile(char* name);
  void removeFile(char* name);
  void closefile(bool store_location=false);
  void release();
  void startFileprint();
  void pauseSDPrint();
  void getStatus();
  void printingHasFinished();
  
	//Rapduch
	uint32_t getFileSize();
	uint32_t getSdPosition();

  void getfilename(uint16_t nr, const char* const match=NULL);
  uint16_t getnrfilenames();
  
  void getAbsFilename(char *t);
  

  void ls();
  int chdir(const char * relpath);
  int updir();
  void setroot();
  FORCE_INLINE bool isEndFile() { if(filesize > file.curPosition()) return 0; else return 1;}
  FORCE_INLINE void sdfinalline() {file.seekEnd(); file.seekSet(file.curPosition()-300);};    ////Go to the end for Simplify extraction	  

#ifdef DEV_BER
  FORCE_INLINE bool isBeginFile() { if(0 <= file.curPosition()) return 0; else return 1;}  
  FORCE_INLINE void sdfinalline_v2() {file.seekEnd(); file.seekSet(file.curPosition()-10);};    ////Go to the end for Cura info JSON extraction
  FORCE_INLINE void feedback_ending() {file.seekEnd(); sdpos = file.curPosition(); file.close();};    ////ending for feedback  
#endif		  

	  
  FORCE_INLINE bool isFileOpen() { return file.isOpen(); }
  FORCE_INLINE bool eof() { return sdpos>=filesize ;};
  FORCE_INLINE int16_t get() {  sdpos = file.curPosition();return (int16_t)file.read();};
  FORCE_INLINE uint32_t getIndex() {return sdpos;};
  FORCE_INLINE void setIndex(long index) {sdpos = index;file.seekSet(index);};
  FORCE_INLINE uint8_t percentDone(){if(!isFileOpen()) return 0; if(filesize) return sdpos/((filesize+99)/100); else return 0;};
  FORCE_INLINE char* getWorkDirName(){workDir.getFilename(filename);return filename;};
public:
  bool saving;
  bool logging;
  bool sdprinting ; 
  bool sdispaused; 
  bool cardOK ;
  char filename[13];
  char longFilename[LONG_FILENAME_LENGTH];
  bool filenameIsDir;
  int lastnr; //last number of the autostart;
private:
  SdFile root,*curDir,workDir,workDirParents[MAX_DIR_DEPTH];
  uint16_t workDirDepth;
  Sd2Card card;
  SdVolume volume;
  SdFile file;
  #define SD_PROCEDURE_DEPTH 1
  #define MAXPATHNAMELENGTH (13*MAX_DIR_DEPTH+MAX_DIR_DEPTH+1)
  uint8_t file_subcall_ctr;
  uint32_t filespos[SD_PROCEDURE_DEPTH];
  char filenames[SD_PROCEDURE_DEPTH][MAXPATHNAMELENGTH];
  uint32_t filesize;
  //int16_t n;
  unsigned long autostart_atmillis;
  uint32_t sdpos ;

  bool autostart_stilltocheck; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.
  
  LsAction lsAction; //stored for recursion.
  int16_t nrFiles; //counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
  char* diveDirName;
  void lsDive(const char *prepend, SdFile parent, const char * const match=NULL);
};
extern CardReader card;
#define IS_SD_PRINTING (card.sdprinting)

#if (SDCARDDETECT > -1)
# ifdef SDCARDDETECTINVERTED 
#  define IS_SD_INSERTED (READ(SDCARDDETECT)!=0)
# else
#  define IS_SD_INSERTED (READ(SDCARDDETECT)==0)
# endif //SDCARDTETECTINVERTED
#else
//If we don't have a card detect line, aways asume the card is inserted
# define IS_SD_INSERTED true
#endif

#else

#define IS_SD_PRINTING (false)

#endif //SDSUPPORT
#endif
