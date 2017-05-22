// SD_ListFiles.h

#ifndef _SD_LISTFILES_h
#define _SD_LISTFILES_h


	#include "arduino.h"
	
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
	void extract_data(void);
	void extract_data1(void);
	uint32_t get_firstdigit_from_integer(uint32_t num_input);
	int extract_data_Symplify(void);
	int extract_data_fromCura(void);
};
	
extern Listfiles listsd;	
	
	
	



#endif

