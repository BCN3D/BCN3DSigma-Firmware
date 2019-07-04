/*
- StatusPrintBCN3D.cpp - A header class that manages the serial print job
Last Update: 16/04/2019
Author: Alejandro Garcia (S3mt0x)
*/

#ifndef _STATUSPRINTBCN3D_h
#define _STATUSPRINTBCN3D_h

#include "Configuration.h"
#include "Marlin.h"
#include "temperature.h"
#include "cardreader.h"
#include "SD_ListFiles.h"

#define MAX_CHAR_JOB 30

class StatusPrintBCN3D
{
	public:
	StatusPrintBCN3D();
	void calc_remaining_time(long seconds);
	unsigned int get_remaining_h();
	unsigned int get_remaining_min();
	void calc_remaining_time_from_sd();
	void set_progress(unsigned int p);
	unsigned int get_progress();
	uint8_t get_status();
	void set_status(uint8_t s);
	void update_temps();
	int get_temp0();
	int get_temp1();
	int get_tempbed();
	
	public:
	char printjobname[MAX_CHAR_JOB];
	
	private:
	long printjob_timeremaining_s;   // s
	unsigned long  printjob_timeremaining_min; //min
	unsigned long  printjob_timeremaining_h;   //hours
	unsigned int printjob_progress; // %
	int printjob_temp_hot0; // last temp t0
	int printjob_temp_hot1; // last temp t1
	int printjob_temp_bed;  // last temp bed
	uint8_t print_job_status;
};

extern StatusPrintBCN3D StatusJob;

#endif

