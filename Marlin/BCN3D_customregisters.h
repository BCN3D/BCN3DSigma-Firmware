/*
- BCN3D_customregisters.h - Custom register for flags
Last Update: 01/08/2018
Author: Alejandro Garcia (S3mt0x)
*/


#ifndef BCN3D_customregisters_H_
#define BCN3D_customregisters_H_

//////BIT REGISTER NAME//////////////////		Nºbit


////screen_change_register/////
#define screen_change_register_flowup			0
#define screen_change_register_flowdown			1
#define screen_change_register_fanup			2
#define screen_change_register_fandown			3
#define screen_change_register_speeddown		4
#define screen_change_register_speedup			5
#define screen_change_register_nozz1up			6
#define screen_change_register_nozz2up			7
#define screen_change_register_nozz1down		8
#define screen_change_register_nozz2down		9
#define screen_change_register_bedup			10
#define screen_change_register_beddown			11
#define screen_change_register_flowupr			12
#define screen_change_register_flowdownr		13
#define screen_change_register_speedupr			14
#define screen_change_register_speeddownr		15
#define screen_change_register_fanupr			16
#define screen_change_register_fandownr			17
//// flag_sdprinting_register/////
#define flag_sdprinting_register_pausepause				0
#define flag_sdprinting_register_printstop				1
#define flag_sdprinting_register_printsavejob			2
#define flag_sdprinting_register_printpause				3
#define flag_sdprinting_register_printresume			4
#define flag_sdprinting_register_pauseresume			5
#define flag_sdprinting_register_temps					6
#define flag_sdprinting_register_control				7
#define flag_sdprinting_register_utilities				8
#define flag_sdprinting_register_showdata				9
#define flag_sdprinting_register_printsavejobcommand	10
#define flag_sdprinting_register_datarefresh			11
#define flag_sdprinting_register_light					12
#define flag_sdprinting_register_pausetest				13


//// flag_sdprinting_register/////
#define flag_utilities_filament_register_home				0
#define flag_utilities_filament_register_acceptok			1
#define flag_utilities_filament_register_purgeselect0		2
#define flag_utilities_filament_register_purgeselect1		3
#define flag_utilities_filament_register_purgeload			4
#define flag_utilities_filament_register_purgeunload		5

//// flag_utilities_calibration/////
#define flag_utilities_calibration_register_calibfull					0
#define flag_utilities_calibration_register_calibfull_skipZcalib		1
#define flag_utilities_calibration_register_calibbeddone				2
#define flag_utilities_calibration_register_bedcomensationmode			3

//// flag_sdprinting_register/////
#define flag_sdlist_resgiter_filesupdown					0
#define flag_sdlist_resgiter_select0						1
#define flag_sdlist_resgiter_select1						2
#define flag_sdlist_resgiter_select2						3		
#define flag_sdlist_resgiter_select3						4
#define flag_sdlist_resgiter_select4						5
#define flag_sdlist_resgiter_select5						6
#define flag_sdlist_resgiter_godown							7
#define flag_sdlist_resgiter_goup							8
#define flag_sdlist_resgiter_goinit							9
#define flag_sdlist_resgiter_gofolderback					10

//// flag_utilities_maintenance_register/////
#define flag_utilities_maintenance_register_nyloncleaning			0
#define flag_utilities_maintenance_register_autotune				1
#define flag_utilities_maintenance_register_zdjust100up				2
#define flag_utilities_maintenance_register_zdjust10up				3
#define flag_utilities_maintenance_register_zdjust100down			4
#define flag_utilities_maintenance_register_zdjust10down			5
#define flag_utilities_maintenance_register_changehotend			6




#endif /* BCN3D_customregisters_H_ */