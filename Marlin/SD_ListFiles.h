/*
	SD_ListFiles.h - A class that manages the parsing of time printing duration, filament consumption and if an gcode is a duplication print.
	Last Update: 20/06/2017
	Author: Alejandro Garcia (S3mt0x)
*/

#ifndef _SD_LISTFILES_h
#define _SD_LISTFILES_h


	#include "genieArduino.h"
	#include "Touch_Screen_Definitions.h"
	#include "Marlin.h"
	#include "Configuration.h"
	#include "stepper.h"
	#include "temperature.h"
	#include "cardreader.h"
	
	
	
	
class Listfiles
{
	public:
	Listfiles();
	void get_lineduration(bool fromfilepoiter, char* name);
	int check_extract_ensure_duplication_print();
	int get_hours(void);
	int get_minutes(void);
	int get_filmetros1(void);
	int get_filmetros2(void);
	int get_filgramos1(void);
	int get_filgramos2(void);
	int get_hoursremaining(void);
	int get_minutesremaining(void);
	int get_hoursremaining_save(long position);
	int get_minutesremaining_save(long position);
	int get_percentage_save(long position);
	public:
	char commandline[50];
	char commandline2[25];
	
	private:
	int dias, horas, minutos, simplify3D;
	long segundos;
	int filmetros1, filmetros2;
	int filgramos1, filgramos2;
	int search_line_data_code(char code);
	int search_line_data_commentary(void);
	void extract_data(void);
	void extract_data1(void);
	int extract_ensure_duplication_print(void);
	int extract_ensure_duplication_print_with_raft_simplify(void);
	int extract_ensure_duplication_print_with_raft_cura(void);
	uint32_t get_firstdigit_from_integer(uint32_t num_input);
	int extract_data_Simplify(void);
};
	
extern Listfiles listsd;	
	
	
	



#endif

