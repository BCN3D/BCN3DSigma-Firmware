/*
- StatusPrintBCN3D.cpp - A class that manages the serial print job
Last Update: 16/04/2019
Author: Alejandro Garcia (S3mt0x)
*/

#include "StatusPrintBCN3D.h"
#include "Configuration.h"
#include "Marlin.h"
#include "temperature.h"
#include "cardreader.h"
#include "SD_ListFiles.h"

StatusPrintBCN3D::StatusPrintBCN3D(){
	memset(printjobname, '\0', sizeof(printjobname));
	printjob_timeremaining_s = 0;
	printjob_timeremaining_min = 0;
	printjob_timeremaining_h = 0;
	printjob_temp_bed = 0;
	printjob_temp_hot0 = 0;
	printjob_temp_hot1 = 0;
	printjob_progress = 0;
	print_job_status = 0;
}
void StatusPrintBCN3D::calc_remaining_time(long seconds){
	printjob_timeremaining_s = seconds;
	printjob_timeremaining_min = (unsigned int) (seconds/60); // total min remaining
	printjob_timeremaining_h = (unsigned int) (seconds/3600); // total h remaining
}
unsigned int StatusPrintBCN3D::get_remaining_h(){
	return (unsigned int)(printjob_timeremaining_h);
}
unsigned int StatusPrintBCN3D::get_remaining_min(){
	return (unsigned int)(printjob_timeremaining_min%60);
}
void StatusPrintBCN3D::set_status(uint8_t s){ 
	print_job_status = s;
}
uint8_t StatusPrintBCN3D::get_status(){ // 1: printing, 2:pausing, 3:resuming, 4: paused, 5: canceling, 0: none
	return print_job_status;
}
unsigned int StatusPrintBCN3D::get_progress(){
	return printjob_progress;
}
void StatusPrintBCN3D::set_progress(unsigned int p){
	printjob_progress = p;
}
void StatusPrintBCN3D::update_temps(){
	printjob_temp_bed = (int)(degBed()+0.5);
	printjob_temp_hot0 = (int)(degHotend(0)+0.5);
	printjob_temp_hot1 = (int)(degHotend(1)+0.5);
}
int StatusPrintBCN3D::get_temp0(){
	return printjob_temp_hot0;
}
int StatusPrintBCN3D::get_temp1(){
	return printjob_temp_hot1;
}
int StatusPrintBCN3D::get_tempbed(){
	return printjob_temp_bed;
}
void StatusPrintBCN3D::calc_remaining_time_from_sd(){
	float mincalc = 0.0;
	
	if(listsd.get_seconds() == 0) return;
			
	mincalc = (float)card.getSdPosition()/card.getFileSize();
	mincalc = listsd.get_seconds()*(1.0-mincalc); // total min remaining	
	
	printjob_timeremaining_min = (unsigned long) (mincalc/60); 	
	
	printjob_timeremaining_h = (unsigned long) (printjob_timeremaining_min/60); // total h remaining
	
	return;
}