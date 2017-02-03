// SD_ListFiles.h

#ifndef _SD_LISTFILES_h
#define _SD_LISTFILES_h


	#include "Arduino.h"

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
	void get_lineduration(void);
	int get_hours(void);
	int get_minutes(void);
	int get_filmetros1(void);
	int get_filmetros2(void);
	int get_filgramos1(void);
	int get_filgramos2(void);
	int get_hoursremaning(void);
	int get_minutesremanig(void);
	public:
	char comandline[50];
	char comandline2[25];

	private:
	int dias, horas, minutos, simplify3D;
	int filmetros1, filmetros2;
	int filgramos1, filgramos2;
	void extract_data(void);
	void extract_data1(void);
	int extract_data_Symplify(void);
	int extract_data_fromCura(void);
};

extern Listfiles listsd;






#endif
