/*
 * LCD_Handler.h
 * A place to hold all interactions between LCD and printer. It is called from Marlin_main.cpp
 * Created: 12/12/2014 12:48:04
 * Author: Jordi Calduch (Dryrain)
 */ 

#ifdef SIGMA_TOUCH_SCREEN

#ifndef LCD_HANDLER_H_
#define LCD_HANDLER_H_

//Rapduch
#include "genieArduino.h"
#include "Touch_Screen_Definitions.h"
#include "Marlin.h"
#include "Configuration.h"
#include "stepper.h"
#include "temperature.h"
//#include "ultralcd.h"

extern bool cancel_heatup;
void myGenieEventHandler();

//Created by Jordi Calduch for RepRapBCN SIGMA 12/2014
void myGenieEventHandler(void)
{
	genieFrame Event;
	genie.DequeueEvent(&Event);
	//static long waitPeriod = millis();
	int move_mm = 10;
	//If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
	if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
	{
		//Winbuttons
		#pragma region Winbuttons
	if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)                // If the Reported Message was from a winbutton
		{
			float modified_position;
			if (Event.reportObject.index == BUTTON_MOVE_AXIS_X)                              // If Winbutton1
			{
				Serial.println("Right");
				modified_position=current_position[X_AXIS]+move_mm;
				if (modified_position < X_MIN_POS)modified_position = X_MIN_POS;
				if (modified_position > X_MAX_POS)modified_position = X_MAX_POS;
				plan_buffer_line(modified_position, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 600, active_extruder);
				current_position[X_AXIS]=modified_position;
				//enquecommand_P(PSTR("G1 F1200 X100"));
			}
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_minusX)                              // If Winbutton0
			{
				Serial.println("Left");
				modified_position=current_position[X_AXIS]-move_mm;
				if (modified_position < X_MIN_POS)modified_position = X_MIN_POS;
				if (modified_position > X_MAX_POS)modified_position = X_MAX_POS;
				plan_buffer_line(modified_position, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 600, active_extruder);
				current_position[X_AXIS]=modified_position;
				//enquecommand_P(PSTR("G1 F1200 X150"));
			}
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_Y)                              // If Winbutton2
			{
				Serial.println("Down");
				modified_position=current_position[Y_AXIS]+move_mm;
				if (modified_position < Y_MIN_POS)modified_position = Y_MIN_POS;
				if (modified_position > Y_MAX_POS)modified_position = Y_MAX_POS;
				plan_buffer_line(current_position[X_AXIS], modified_position, current_position[Z_AXIS], current_position[E_AXIS], 600, active_extruder);
				current_position[Y_AXIS]=modified_position;
			}
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_minusY)                              // If Winbutton3
			{
				//if (millis() >= waitPeriod){
				Serial.println("Up");
				modified_position=current_position[Y_AXIS]-move_mm;
				if (modified_position < Y_MIN_POS)modified_position = Y_MIN_POS;
				if (modified_position > Y_MAX_POS)modified_position = Y_MAX_POS;
				plan_buffer_line(current_position[X_AXIS], modified_position, current_position[Z_AXIS], current_position[E_AXIS], 600, active_extruder);
				current_position[Y_AXIS]=modified_position;
				//waitPeriod=millis()+50;
				//}
			}
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_HOME)                              // If Winbutton4
			{
				//if (millis() >= waitPeriod){
				Serial.println("HOME");
				enquecommand_P((PSTR("G28")));
				//waitPeriod=millis()+50;
				//}
			}
			
			
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_Z)                              // If Winbutton7 Button Z
			{
				//if (millis() >= waitPeriod){
				Serial.println("ZUp");
				modified_position=current_position[Z_AXIS]+move_mm;
				if (modified_position < Z_MIN_POS)modified_position = Z_MIN_POS;
				if (modified_position > Z_MAX_POS)modified_position = Z_MAX_POS;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], modified_position, current_position[E_AXIS], 600, active_extruder);
				current_position[Z_AXIS]=modified_position;
				//waitPeriod=millis()+50;
				//}
			}
			
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_minusZ)                              // If Winbutton8
			{
				//if (millis() >= waitPeriod){
				Serial.println("ZDown");
				modified_position=current_position[Z_AXIS]-move_mm;
				if (modified_position < Z_MIN_POS)modified_position = Z_MIN_POS;
				if (modified_position > Z_MAX_POS)modified_position = Z_MAX_POS;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], modified_position, current_position[E_AXIS], 600, active_extruder);
				current_position[Z_AXIS]=modified_position;
				//waitPeriod=millis()+50;
				//}
			}
			
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_E)                              
			{				
				Serial.println("EUp");
				modified_position=current_position[E_AXIS]+move_mm;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], modified_position, 500/60, active_extruder);
				current_position[E_AXIS]=modified_position;								
			}
			
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_minusE)                              
			{				
				Serial.println("EDown");
				modified_position=current_position[E_AXIS]-move_mm;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], modified_position, 500/60, active_extruder);
				current_position[E_AXIS]=modified_position;								
			}
		}
#pragma endregion Winbuttons
		
		
		//USERBUTTONS------------------------------------------------------
		if (Event.reportObject.object == GENIE_OBJ_USERBUTTON) //Userbuttons to select GCODE from SD
		{
			if (Event.reportObject.index == BUTTON_SD_SELECTED )
			{
				if(card.cardOK)
				{
					//THESE FOLLOWING COMMANDS CAN BE DONE VIA GCODE - It seems safer to do it here because avoids user errors
					
					//enquecommand_P((PSTR("G28"))); // Force home to clean the buffer and avoid problems with  Dual X
			
					//if (enable_autobed_level)
					//{
						//enquecommand_P((PSTR("G29"))); // Command to enable Z autoleveling - probes 3 points		
					//}
										
					//st_synchronize(); //Wait before starting to print.
					
					
					//genie.WriteObject(GENIE_OBJ_FORM,FORM_START_PRINT,0);
					//genie.WriteObject(GENIE_OBJ_FORM,9,0); //Printing FORM
					
					//genie.WriteStr(STRINGS_PRINTING_GCODE,card.longFilename);//Printing form
					
					wake_RELAY();	
						
					if (!card.filenameIsDir){ //If the filename is a gcode we start printing
						char cmd[30];
						char* c;
						card.getfilename(filepointer);
						sprintf_P(cmd, PSTR("M23 %s"), card.filename);
						for(c = &cmd[4]; *c; c++)
						{
							*c = tolower(*c);
						}
						enquecommand(cmd);
						
						is_on_printing_screen=true;//We are entering printing screen
						enquecommand_P(PSTR("M24")); // It also sends you to PRINTING screen
						
						screen_status="Ready...";//Write the selected SD file to all strings						
					}				
					//genie.WriteStr(8,"Ready...");
					//genie.WriteStr(7,card.longFilename);
					//Reset Time LEDs
					
					//genie.WriteStr(STRINGS_PRINTING_GCODE,card.longFilename);//Printing form
					//genie.WriteStr(6,"Ready");
					
					//genie.WriteObject(GENIE_OBJ_LED_DIGITS,12,0);
					//genie.WriteObject(GENIE_OBJ_LED_DIGITS,11,0);
					
					//StartPrint form
					//genie.WriteStr(2,card.longFilename);//Printing form
					//genie.WriteStr(10,card.longFilename);//PausedPrint form
				}
			}
			
			else if (Event.reportObject.index == BUTTON_START_PRINTING )
			{
				//DEPRECATED 1/02/2015
				genie.WriteObject(GENIE_OBJ_FORM,9,0); //Printing FORM
				genie.WriteStr(2,card.longFilename);//Printing form
				char cmd[30];
				char* c;
				card.getfilename(filepointer);
				sprintf_P(cmd, PSTR("M23 %s"), card.filename);
				for(c = &cmd[4]; *c; c++)
				{
					*c = tolower(*c);
				}
				enquecommand(cmd);
				enquecommand_P(PSTR("M24"));
			}
			
			
			else if (Event.reportObject.index == BUTTON_SD_LEFT || Event.reportObject.index == BUTTON_SD_RIGHT) //TODO: control if SD is out
			{
				if (Event.reportObject.index == BUTTON_SD_LEFT) //LEFT button pressed
				{
					if (filepointer == 0)
					{
						filepointer=card.getnrfilenames()-1; //Last SD file
						}else{
						filepointer--;
					}
				}
				else if (Event.reportObject.index == BUTTON_SD_RIGHT) //RIGHT button pressed
				{
					if (filepointer == card.getnrfilenames()-1)
					{
						filepointer=0; //First SD file
						}else{
						filepointer++;
					}
				}
				
				card.getfilename(filepointer);
				if (card.filenameIsDir)
				{
					//Is a folder
					//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,1);
					}else{
					//Is a file
					//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
				}
				int count = 12;
				char buffer[count];
				if (String(card.longFilename).length()>count){
					for (int i = 0; i<count ; i++)
					{
						buffer[i]=card.longFilename[i];
					}
					buffer[count]='\0';
					char* buffer2 = strcat(buffer,"...\0");
					genie.WriteStr(1,buffer2);//Printing form
				}else{
					genie.WriteStr(1,card.longFilename);//Printing form
				}
				//Keep in mind to control the length of the string displayed!
				//genie.WriteStr(2,card.longFilename);
				Serial.print("Image n: ");
				Serial.println(filepointer);
			}
			
			else if (Event.reportObject.index == BUTTON_RESUME )
			{
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_PAUSE,1);
				Serial.println("RESUME!");
				card.startFileprint();
				genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING,0);
			}
			
			else if (Event.reportObject.index == BUTTON_STOP_YES )
			{
				is_on_printing_screen=false;
				card.sdprinting = false;
				card.closefile();
				
				//plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS]+10,current_position[E_AXIS], 600, active_extruder);
				quickStop();
				
				enquecommand_P(PSTR("G28 X0 Y0")); //Home X and Y
				enquecommand_P(PSTR("T0")); //The default states is Left Extruder active
				
				if(SD_FINISHED_STEPPERRELEASE)
				{
					enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
				}
				autotempShutdown();
				//setTargetHotend0(0);
				//setTargetHotend1(0);
				//setTargetHotend2(0);
				//setTargetBed(0);
				card.sdispaused = false;	
				cancel_heatup = true;	
				
				sleep_RELAY();
							
				//Rapduch
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);			
			}
			
			else if (Event.reportObject.index == BUTTON_SPEED_UP )
			{
				int value=5;
				if (feedmultiply<200)
				{
					feedmultiply+=value;
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FEEDRATE,feedmultiply);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_SPEED_DOWN )
			{
				int value=5;
				if (feedmultiply>50)
				{
					feedmultiply-=value;
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FEEDRATE,feedmultiply);
				}
			}
			
			
			
			else if (Event.reportObject.index == BUTTON_BED_UP )
			{
				int value=5;
				//if (target_temperature_bed<BED_MAXTEMP)
				if (target_temperature_bed<120)//MaxTemp
				{
					target_temperature_bed+=value;
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_BED,target_temperature_bed);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_BED_DOWN )
			{
				int value=5;
				//if (target_temperature_bed>BED_MINTEMP)
				if (target_temperature_bed>5)//Mintemp
				{
					target_temperature_bed-=value;
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_BED,target_temperature_bed);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_FAN_UP )
			{
				int value=5;
				if (fanSpeed<255)
				{
					fanSpeed+=value;
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FAN,fanSpeed);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_FAN_DOWN )
			{
				int value=5;
				if (fanSpeed>0)
				{
					fanSpeed-=value;
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FAN,fanSpeed);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_SETUP_BACK )
			{
				//Get status string
				char buffer[15];
				screen_status.toCharArray(buffer,15,0);
				
				if (card.sdprinting || card.sdispaused)
				{
					genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING,0);
					//Restore Strings after form update
					genie.WriteStr(2,card.longFilename);
					//genie.WriteStr(6,buffer);
				}else
				{
					genie.WriteObject(GENIE_OBJ_FORM,FORM_START_PRINT,0);
					//Restore Strings
					genie.WriteStr(7,card.longFilename);
					genie.WriteStr(8,buffer);
				}
			}
			
			
			else if (Event.reportObject.index == BUTTON_SETUP_BACK_NOZZLE || Event.reportObject.index == BUTTON_SETUP_BACK_BED )
			{
				if (surfing_utilities) // Check if we are backing from utilities or print setup
				{
					genie.WriteObject(GENIE_OBJ_FORM,FORM_TEMPERATURE,0);
				}else
				{
					genie.WriteObject(GENIE_OBJ_FORM,FORM_SETUP,0);
				}
			}
			
			
			else if (Event.reportObject.index == BUTTON_PAUSE_RESUME )
			{
				int value = genie.GetEventData(&Event);
				if (value == 1) // Need to pause
				{
					//I believe it is a really unsafe way to do it
					//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+20, current_position[E_AXIS], homing_feedrate[Z_AXIS]/60, RIGHT_EXTRUDER);
					//st_synchronize();
					card.pauseSDPrint();				
					//genie.WriteStr(6,"Pausing...");
					Serial.println("PAUSE!");
				}else{
					if(card.sdispaused)
					{
						//I believe it is a really unsafe way to do it
						//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]-20, current_position[E_AXIS], homing_feedrate[Z_AXIS]/60, RIGHT_EXTRUDER);
						//st_synchronize();
						card.startFileprint();
						//genie.WriteStr(6,"Printing...");
						Serial.println("RESUME!");
					}else{
						Serial.println("We have stopped");
					}
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PREHEAT_PLA)
			{
				//Reset other buttons
				//genie.WriteObject(GENIE_OBJ_USERIMAGES,BUTTON_PREHEAT_ABS,0);
				//int value = genie.GetEventData(&Event);
				//if (value == 1) // Need to preheat
				//{
					setTargetHotend0(PLA_PREHEAT_HOTEND_TEMP);
					setTargetHotend1(PLA_PREHEAT_HOTEND_TEMP);
					setTargetBed(PLA_PREHEAT_HPB_TEMP);
					Serial.println("Preheating PLA");
				//}
				//else
				//{
					//setTargetHotend(0,active_extruder);
					//setTargetBed(0);
					//Serial.println("Cooling down PLA");
				//}
				genie.WriteObject(GENIE_OBJ_FORM,FORM_TEMPERATURE,0);
			}
				
			else if (Event.reportObject.index == BUTTON_CHANGE_EXTRUDER)
			{
				int value = genie.GetEventData(&Event);
				if (value == 1)
				{ //Second extruder
					enquecommand_P(((PSTR("T 1"))));
					which_extruder=1;
				}else{
					enquecommand_P(((PSTR("T 0"))));
					which_extruder=0;
				}
			}
			
			else if (Event.reportObject.index == BUTTON_CAL_WIZARD)
			{
				//genie.WriteStr(STRING_GPURPOSE_WAIT,"Processing");
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_CAL_WIZARD_WAIT,0);
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("T0"));
				enquecommand_P(PSTR("G34"));	//Start Calibration Wizard	
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);			
			}
			
			
			//NOZZLE BUTTONS reusability---------------------------------------------------
			
			else if (Event.reportObject.index == BUTTON_NOZZLE1_PRINT || Event.reportObject.index == BUTTON_NOZZLE2_PRINT || Event.reportObject.index == BUTTON_NOZZLE1_TEMP || Event.reportObject.index == BUTTON_NOZZLE2_TEMP)
			{
				if (Event.reportObject.index == BUTTON_NOZZLE1_PRINT || Event.reportObject.index == BUTTON_NOZZLE1_TEMP)
				{
					which_extruder=0;
				}else{
					which_extruder=1;
				}				
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[which_extruder]);
				genie.WriteObject(GENIE_OBJ_FORM,FORM_NOZZLE,0);
			}
			
			else if (Event.reportObject.index == BUTTON_NOZZLE_UP )
			{
				int value=5;
				if (target_temperature[which_extruder]<HEATER_0_MAXTEMP)
				{
					target_temperature[which_extruder]+=value;
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[which_extruder]);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_NOZZLE_DOWN )
			{
				int value=5;
				if (target_temperature[which_extruder]>HEATER_0_MINTEMP)
				{
					target_temperature[which_extruder]-=value;
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[which_extruder]);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_COOLDOWN )
			{
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);		
			}
			
			//NOZZLEBUTTONS-------
			
			
			//INSERT/REMOVE FILAMENT
			
			else if (Event.reportObject.index == BUTTON_INSERT_FIL || Event.reportObject.index == BUTTON_REMOVE_FIL || Event.reportObject.index == BUTTON_PURGE_FIL  )
			{
				//setTargetHotend0(ABS_PREHEAT_HOTEND_TEMP);
				//setTargetHotend1(ABS_PREHEAT_HOTEND_TEMP);							
				if (Event.reportObject.index == BUTTON_INSERT_FIL) filament_mode = 'I'; //Insert Mode
				else if (Event.reportObject.index == BUTTON_REMOVE_FIL) filament_mode = 'R'; //Remove Mode
				else filament_mode = 'P'; //Purge Mode
				genie.WriteObject(GENIE_OBJ_FORM,FORM_SELECT_EXTRUDER,0);
			}
			
			else if (Event.reportObject.index == BUTTON_FILAMENT_NOZZLE1 || Event.reportObject.index == BUTTON_FILAMENT_NOZZLE2)
			{
				if (Event.reportObject.index == BUTTON_FILAMENT_NOZZLE1) //Left Nozzle
				{
					which_extruder=0;
					//Heat
					//if (filament_is_inserted[which_extruder])
					//setTargetHotend0(ABS_PREHEAT_HOTEND_TEMP);
				} 
				else //Right Nozzle
				{
					which_extruder=1;
					//Heat
					//setTargetHotend1(ABS_PREHEAT_HOTEND_TEMP);
				}		
				
				//if (!filament_is_inserted[which_extruder])
				//{	
					//ATTENTION : Order here is important		
					setTargetHotend(ABS_PREHEAT_HOTEND_TEMP,which_extruder); //First it is important to set the temp
					is_changing_filament=true; //We are changing filament
					genie.WriteObject(GENIE_OBJ_FORM,FORM_INSERT_FIL_PREHEAT,0);
				//} 
				//else
				//{
					//writeInfoString("Error: Remove filament first");
				//}				
			}		
			
			else if (Event.reportObject.index == BUTTON_INSERT_BACK)
			{
				is_changing_filament=false; //We are no longer waiting for heat
				setTargetHotend(0,which_extruder); //ATTENTION : Order here is important
				genie.WriteObject(GENIE_OBJ_FORM,FORM_SELECT_EXTRUDER,0); 
			}
			
			else if (Event.reportObject.index == BUTTON_INSERT )
			{// We should have already checked if filament is inserted				
				if (filament_mode =='I')
				{
					//current_position[E_AXIS] += (BOWDEN_LENGTH+100);
					current_position[E_AXIS] += (BOWDEN_LENGTH);
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_FAST_SPEED/60, which_extruder);
					current_position[E_AXIS] += EXTRUDER_LENGTH;//Extra extrusion at low feedrate
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_SLOW_SPEED/60, which_extruder);
					//filament_is_inserted[which_extruder]=true;
				}else if (filament_mode =='R')
				{
					//current_position[E_AXIS] -= (BOWDEN_LENGTH+100);
					//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 500/60, which_extruder);
					//current_position[E_AXIS] -= EXTRUDER_LENGTH;//Extra extrusion at low feedrate
					//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  150/60, which_extruder);
					//filament_is_inserted[which_extruder]=false;
					
					current_position[E_AXIS] -= EXTRUDER_LENGTH;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], INSERT_SLOW_SPEED/60, which_extruder);
					current_position[E_AXIS] -= BOWDEN_LENGTH;//Extra extrusion at low feedrate
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],  INSERT_FAST_SPEED/60, which_extruder);
					
				}	
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
				st_synchronize();		
				setTargetHotend(0,which_extruder);
				//put_info_text("Filament DONE");
				genie.WriteObject(GENIE_OBJ_FORM,FORM_UTILITIES,0);
			}
			
			
			
			
			//Extruder Calibrations-------------------------------------------------
			else if (Event.reportObject.index == BUTTON_CAL_EXTRUDERS)
			{
				enquecommand_P(PSTR("G28"));
				enquecommand_P(PSTR("G40"));
				
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
				
				//Rapduch first try 20/02/2015
				//homeFromMain();
				//enquecommand_P(PSTR("G28"));
				//st_synchronize();
				//Preheat
				//setTargetHotend0(PLA_PREHEAT_HOTEND_TEMP);
				//setTargetHotend1(PLA_PREHEAT_HOTEND_TEMP);
				
				//while (degHotend(0)<degTargetHotend(0)){ //Waiting to heat the extruder
					//manage_heater();
				//}
				//plan_buffer_line(0, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 1500/60, active_extruder);
				//st_synchronize();
				//plan_buffer_line(50, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 1500/60, active_extruder);
				
				//We will have to do it by plan buffers
				
				
				
				
				//st_synchronize();
				
				
				//homeaxis(X_AXIS);
				
				//enquecommand_P(PSTR("G28"));
				//enquecommand("G28 X0");
				//enquecommand("G28 Y0");
				//enquecommand("G28 Z0");
				
				
				
				//st_synchronize();
				//enquecommand_P(PSTR("G1 X100 Y100"));
				//st_synchronize();
				
				//plan_buffer_line(1,current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 600, active_extruder);
				
				
				//enquecommand_P(PSTR("G28"));
				//enquecommand_P(((PSTR("T 1"))));
				//enquecommand_P(((PSTR("T 0"))));
				////enquecommand_P(PSTR("G90"));
				////enquecommand_P(PSTR("M83"));
				//st_synchronize();
				////
				//////Set starting position 
				////enquecommand("G90");
				//enquecommand_P(PSTR("G1 X20 Y100 F9000"));
				//
				//st_synchronize();
				//enquecommand_P(PSTR("G91"));
				////
				//for (int i=0; i<4;i++)
				//{
					//enquecommand_P(PSTR("G1 Y100 E1 F9000"));				
					//enquecommand_P(PSTR("G1 X40 F9000"));
				//}
				
				
				//enquecommand("G28");
				//enquecommand("T 1");
				//enquecommand("T 0");
				//enquecommand_P(PSTR("G90"));
				//enquecommand_P(PSTR("M83"));				
				//
				////Set starting position
				//enquecommand("G90");
				//enquecommand("G1 X20 Y100 F9000");
				//st_synchronize();
				
				//while (getBuflen()>=4){
					//Serial.print("Waiting");
				//}
				
				
				
				
				
				
				
				//float mm_second_extruder[4] = {40.5, 39.5, 40, 39};
				//
				//float mm_each_extrusion =40;
				//float mm_left_offset = 10;
				//for (int i=1; i<5;i++) //4 times
				//{
					//plan_buffer_line(mm_left_offset+(mm_each_extrusion*i), 200, current_position[Z_AXIS], current_position[E_AXIS], 2000/60, active_extruder);			
					//plan_buffer_line(mm_left_offset+(mm_each_extrusion*i), 150, current_position[Z_AXIS], current_position[E_AXIS], 2500/60, active_extruder);		
					//st_synchronize();
				//}				
				//current_position[X_AXIS]=mm_left_offset+4*mm_each_extrusion;
				//current_position[Y_AXIS]=150;
				//
				////plan_set_position(current_position[X_AXIS]+(4*mm_each_extrusion), 150, current_position[Z_AXIS], current_position[E_AXIS]);
				//
				//changeTool(1);
				//
				////Second Extruder (correcting)
				//for (int i=1; i<5;i++) //4 times
				//{
					//plan_buffer_line(mm_left_offset+(mm_second_extruder[i-1]*i), 200, current_position[Z_AXIS], current_position[E_AXIS], 2000/60, active_extruder);
					//plan_buffer_line(mm_left_offset+(mm_second_extruder[i-1]*i), 150, current_position[Z_AXIS], current_position[E_AXIS], 2500/60, active_extruder);
					//st_synchronize();
				//}
				//current_position[X_AXIS]=mm_left_offset+4*mm_second_extruder[3];
				//current_position[Y_AXIS]=150;
				//
				//changeTool(0);
				//
				
				
				
				
				
				
				
				//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,current_position[E_AXIS], 6000/60, active_extruder);	
				//plan_buffer_line(MANUAL_X_HOME_POS, current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,current_position[E_AXIS], 6000/60, active_extruder);				
				//plan_buffer_line(MANUAL_X_HOME_POS, current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS], 6000/60, active_extruder);
				//st_synchronize();
				
				// Save current position to return to after applying extruder offset
				//memcpy(destination, current_position, sizeof(destination));
				//#ifdef DUAL_X_CARRIAGE
				//
					//// Park old head: 1) raise 2) move to park position 3) lower
					//plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,
					//current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
					//plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,
					//current_position[E_AXIS], max_feedrate[X_AXIS], active_extruder);
					//plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS],
					//current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
					//st_synchronize();
							//
				//active_extruder = tmp_extruder;
				//
				//
				
				
				
				//Now comes the correcting 
				
				
			}		
			
			
			else if (Event.reportObject.index == BUTTON_PRINT_SET_SPEED_UP )
			{
				char buffer[256];
				int value=5;
				if (feedmultiply<200)
				{			
					feedmultiply+=value;
					sprintf(buffer, "%3d %%",feedmultiply);
					genie.WriteStr(STRING_PRINT_SET_PERCENT,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PRINT_SET_SPEED_DOWN )
			{
				char buffer[256];
				int value=5;
				if (feedmultiply>50)
				{
					feedmultiply-=value;
					sprintf(buffer, "%3d %%",feedmultiply);
					genie.WriteStr(STRING_PRINT_SET_PERCENT,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ1_UP )
			{
				char buffer[256];
				int value=5;
				if (target_temperature[0]<HEATER_0_MAXTEMP)
				{
					target_temperature[0]+=value;
					sprintf(buffer, "%3d",target_temperature[0]);
					genie.WriteStr(STRING_PRINT_SET_NOZZ1,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ1_DOWN )
			{
				char buffer[256];
				int value=5;
				if (target_temperature[0]>HEATER_0_MINTEMP)
				{
					target_temperature[0]-=value;
					sprintf(buffer, "%3d",target_temperature[0]);
					genie.WriteStr(STRING_PRINT_SET_NOZZ1,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ2_UP )
			{
				char buffer[256];
				int value=5;
				if (target_temperature[1]<HEATER_1_MAXTEMP)
				{
					target_temperature[1]+=value;
					sprintf(buffer, "%3d",target_temperature[1]);
					genie.WriteStr(STRING_PRINT_SET_NOZZ2,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PRINT_SET_NOZZ2_DOWN )
			{
				char buffer[256];
				int value=5;
				if (target_temperature[1]>HEATER_1_MINTEMP)
				{
					target_temperature[1]-=value;
					sprintf(buffer, "%3d",target_temperature[1]);
					genie.WriteStr(STRING_PRINT_SET_NOZZ2,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PRINT_SET_BED_UP )
			{
				char buffer[256];
				int value=5;
				//if (target_temperature_bed<BED_MAXTEMP)
				if (target_temperature_bed<120)//MaxTemp
				{
					target_temperature_bed+=value;
					sprintf(buffer, "%3d",target_temperature_bed);
					genie.WriteStr(STRING_PRINT_SET_BED,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PRINT_SET_BED_DOWN )
			{
				char buffer[256];
				int value=5;
				//if (target_temperature_bed>BED_MINTEMP)
				if (target_temperature_bed>5)//Mintemp
				{
					target_temperature_bed-=value;
					sprintf(buffer, "%3d",target_temperature_bed);
					genie.WriteStr(STRING_PRINT_SET_BED,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PRINT_SETTINGS )
			{
				//Rapduch
				//Edit for final TouchScreen
				char buffer[256];
				int tHotend=target_temperature[0];
				int tHotend1=target_temperature[1];
				int tBed=target_temperature_bed;
				
				is_on_printing_screen=false;
				genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING_SETTINGS,0);
				
				sprintf(buffer, "%3d",tHotend);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PRINT_SET_NOZZ1,buffer);
				
				sprintf(buffer, "%3d",tHotend1);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PRINT_SET_NOZZ2,buffer);
				
				sprintf(buffer, "%3d",tBed);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PRINT_SET_BED,buffer);
				
				sprintf(buffer, "%3d %%",feedmultiply);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PRINT_SET_PERCENT,buffer);
				
				
			}
			
			else if (Event.reportObject.index == BUTTON_PRINT_SET_BACK )
			{
				is_on_printing_screen=true;
				int count = 12;
				char buffer[count];
				if (String(card.longFilename).length()>count){
					for (int i = 0; i<count ; i++)
					{
						buffer[i]=card.longFilename[i];
					}
					buffer[count]='\0';
					char* buffer2 = strcat(buffer,"...\0");
					genie.WriteStr(STRINGS_PRINTING_GCODE,buffer2);//Printing form
					}else{
					genie.WriteStr(STRINGS_PRINTING_GCODE,card.longFilename);//Printing form
				}
				genie.WriteObject(GENIE_OBJ_FORM,FORM_PRINTING,1);
			}
			
			
			//Buttons for preheat settings
			else if (Event.reportObject.index == BUTTON_PREHEAT_SET_NOZZ1_UP )
			{
				char buffer[256];
				int value=5;
				if (target_temperature[0]<HEATER_0_MAXTEMP)
				{
					target_temperature[0]+=value;
					sprintf(buffer, "%3d",target_temperature[0]);
					genie.WriteStr(STRING_PREHEAT_SET_NOZZ1,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PREHEAT_SET_NOZZ1_DOWN )
			{
				char buffer[256];
				int value=5;
				if (target_temperature[0]>HEATER_0_MINTEMP)
				{
					target_temperature[0]-=value;
					sprintf(buffer, "%3d",target_temperature[0]);
					genie.WriteStr(STRING_PREHEAT_SET_NOZZ1,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PREHEAT_SET_NOZZ2_UP )
			{
				char buffer[256];
				int value=5;
				if (target_temperature[1]<HEATER_1_MAXTEMP)
				{
					target_temperature[1]+=value;
					sprintf(buffer, "%3d",target_temperature[1]);
					genie.WriteStr(STRING_PREHEAT_SET_NOZZ2,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PREHEAT_SET_NOZZ2_DOWN )
			{
				char buffer[256];
				int value=5;
				if (target_temperature[1]>HEATER_1_MINTEMP)
				{
					target_temperature[1]-=value;
					sprintf(buffer, "%3d",target_temperature[1]);
					genie.WriteStr(STRING_PREHEAT_SET_NOZZ2,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PREHEAT_SET_BED_UP )
			{
				char buffer[256];
				int value=5;
				//if (target_temperature_bed<BED_MAXTEMP)
				if (target_temperature_bed<120)//MaxTemp
				{
					target_temperature_bed+=value;
					sprintf(buffer, "%3d",target_temperature_bed);
					genie.WriteStr(STRING_PREHEAT_SET_BED,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PREHEAT_SET_BED_DOWN )
			{
				char buffer[256];
				int value=5;
				//if (target_temperature_bed>BED_MINTEMP)
				if (target_temperature_bed>5)//Mintemp
				{
					target_temperature_bed-=value;
					sprintf(buffer, "%3d",target_temperature_bed);
					genie.WriteStr(STRING_PREHEAT_SET_BED,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PREHEAT_SET_BACK )
			{
				//Cooldown
				setTargetHotend0(0);
				setTargetHotend1(0);
				setTargetBed(0);
				genie.WriteObject(GENIE_OBJ_FORM,FORM_TEMP_MENU,0);
			}
			
			else if (Event.reportObject.index == BUTTON_REDO_BED_CALIB )
			{
				enquecommand_P((PSTR("G28")));
				enquecommand_P((PSTR("T0")));
				enquecommand_P((PSTR("G34")));
				genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
			}
			
			else if (Event.reportObject.index == BUTTON_BED_CALIB_SW2)
			{
				char buffer[256];
				if (vuitens2!=0){
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW2,0);
					sprintf(buffer, " %d / 8",vuitens2); //Printing how to calibrate on screen
					genie.WriteStr(STRING_BED_SCREW2,buffer);
					if (sentit2>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,1);} //The direction is inverted in Sigma's bed screws
					else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,0);}
				
				}else if (vuitens3!=0)
				{
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW3,0);
					sprintf(buffer, " %d / 8",vuitens3); //Printing how to calibrate on screen
					genie.WriteStr(STRING_BED_SCREW3,buffer);
					if (sentit3>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,1);} //The direction is inverted in Sigma's bed screws
					else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,0);}
				}else{
					enquecommand_P((PSTR("G28")));
					enquecommand_P((PSTR("T0")));
					enquecommand_P((PSTR("G34")));
					genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);	
				}	
			}
			
			else if (Event.reportObject.index == BUTTON_BED_CALIB_SW3)
			{
				char buffer[256];
				if (vuitens3!=0){
					genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_BED_SCREW3,0);
					sprintf(buffer, " %d / 8",vuitens3); //Printing how to calibrate on screen
					genie.WriteStr(STRING_BED_SCREW3,buffer);
					if (sentit3>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,1);} //The direction is inverted in Sigma's bed screws
					else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW3,0);}
				}else{
					enquecommand_P((PSTR("G28")));
					enquecommand_P((PSTR("T0")));
					enquecommand_P((PSTR("G34")));
					genie.WriteObject(GENIE_OBJ_FORM,FORM_WAITING_ROOM,0);
				}
			}	
			
			else if (Event.reportObject.index == BUTTON_BED_CALIB_SUCCESS)
			{
				enquecommand_P((PSTR("G28")));
				enquecommand_P((PSTR("T0")));
				Serial.println("Calibration Successful, going back to main menu");
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_X_LINE_SELECT1)
			{
				//char buffer[30];
				float calculus = extruder_offset[X_AXIS][1] + 0.4;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_X_LINE_SELECT2)
			{
				//enquecommand_P((PSTR("M218 T1 X-0.4")));
				float calculus = extruder_offset[X_AXIS][1] + 0.3;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_X_LINE_SELECT3)
			{
				float calculus = extruder_offset[X_AXIS][1] + 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//enquecommand_P((PSTR("M218 T1 X-0.3")));
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_X_LINE_SELECT4)
			{
				float calculus = extruder_offset[X_AXIS][1] + 0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//enquecommand_P((PSTR("M218 T1 X-0.2")));
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_X_LINE_SELECT5)
			{
				float calculus = extruder_offset[X_AXIS][1] ;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.1")));
				//enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_X_LINE_SELECT6)
			{
				float calculus = extruder_offset[X_AXIS][1] - 0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_X_LINE_SELECT7)
			{
				float calculus = extruder_offset[X_AXIS][1] - 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_X_LINE_SELECT8)
			{
				float calculus = extruder_offset[X_AXIS][1] - 0.3;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_X_LINE_SELECT9)
			{
				float calculus = extruder_offset[X_AXIS][1] - 0.4;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_X_LINE_SELECT10)
			{
				float calculus = extruder_offset[X_AXIS][1];
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT1)
			{
				//char buffer[30];
				float calculus = extruder_offset[Y_AXIS][1] + 0.4;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//sprintf(buffer, "M218 T1 X%f",calculus); //
				//sprintf_P(buffer, PSTR("M218 T1 X%s"), String(calculus));
				//enquecommand(buffer);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.5")));
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT2)
			{
				//enquecommand_P((PSTR("M218 T1 X-0.4")));
				float calculus = extruder_offset[Y_AXIS][1] + 0.3;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT3)
			{
				float calculus = extruder_offset[Y_AXIS][1] + 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//enquecommand_P((PSTR("M218 T1 X-0.3")));
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT4)
			{
				float calculus = extruder_offset[Y_AXIS][1] + 0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				//enquecommand_P((PSTR("M218 T1 X-0.2")));
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT5)
			{
				float calculus = extruder_offset[Y_AXIS][1] ;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X-0.1")));
				//enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT6)
			{
				float calculus = extruder_offset[Y_AXIS][1] - 0.1;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT7)
			{
				float calculus = extruder_offset[X_AXIS][1] - 0.2;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[X_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT8)
			{
				float calculus = extruder_offset[Y_AXIS][1] - 0.3;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT9)
			{
				float calculus = extruder_offset[Y_AXIS][1] - 0.4;
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			else if (Event.reportObject.index == BUTTON_Y_LINE_SELECT10)
			{
				float calculus = extruder_offset[Y_AXIS][1];
				Serial.print("Calculus:  ");
				Serial.println(calculus);
				extruder_offset[Y_AXIS][RIGHT_EXTRUDER]=calculus;
				//enquecommand_P((PSTR("M218 T1 X0.1")));
				enquecommand_P((PSTR("M500"))); //Store changes
				genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN_SCREEN,0);
			}
			
			
			else if (Event.reportObject.index == BUTTON_Z_CALIB_Z1_Up)
			{
				float feedrate = homing_feedrate[Z_AXIS];
				current_position[Z_AXIS] += 0.1;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				Serial.print("Z position: ");
				Serial.println(current_position[Z_AXIS]);
			}
			
			else if (Event.reportObject.index == BUTTON_Z_CALIB_Z1_Down)
			{
				float feedrate = homing_feedrate[Z_AXIS];
				if (current_position[Z_AXIS]>-0.5) current_position[Z_AXIS] -= 0.1; //Max down is Z=-0.5
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				Serial.print("Z position: ");
				Serial.println(current_position[Z_AXIS]);
			}
			
			else if (Event.reportObject.index == BUTTON_Z_CALIB_Z1_OK)
			{
				Serial.println("OK first Extruder!");
				//We have to override z_prove_offset
				zprobe_zoffset-=current_position[Z_AXIS]; //We are putting more offset if needed
				extruder_offset[Z_AXIS][LEFT_EXTRUDER]=0.0;//It is always the reference
				current_position[Z_AXIS]=0;//We are setting this position as Zero
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
				Serial.print("Z1 Probe offset: ");
				Serial.println(zprobe_zoffset);
				enquecommand_P(PSTR("M500"));//Store everything
				enquecommand_P(PSTR("T1"));
				enquecommand_P(PSTR("G43"));
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIB_Z_EXTRUDER2,0);
			}
			
			else if (Event.reportObject.index == BUTTON_Z_CALIB_Z2_Up)
			{
				float feedrate = homing_feedrate[Z_AXIS];
				current_position[Z_AXIS] += 0.1;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				Serial.print("Z position: ");
				Serial.println(current_position[Z_AXIS]);
			}
			
			else if (Event.reportObject.index == BUTTON_Z_CALIB_Z2_Down)
			{
				float feedrate = homing_feedrate[Z_AXIS];
				if (current_position[Z_AXIS]>-0.5) current_position[Z_AXIS] -= 0.1;  //Max down is Z=-0.5
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
				Serial.print("Z position: ");
				Serial.println(current_position[Z_AXIS]);
			}
			
			else if (Event.reportObject.index == BUTTON_Z_CALIB_Z2_OK)
			{
				Serial.println("OK second Extruder!");
				extruder_offset[Z_AXIS][RIGHT_EXTRUDER]-=current_position[Z_AXIS];//Add the difference to the current offset value
				Serial.print("Z2 Offset: ");
				Serial.println(extruder_offset[Z_AXIS][RIGHT_EXTRUDER]);
				enquecommand_P(PSTR("M500"));//Store everything
				enquecommand_P(PSTR("T0"));
				enquecommand_P(PSTR("G28 X0 Y0"));
				genie.WriteObject(GENIE_OBJ_FORM,FORM_CALIBRATION,0);
			}
		}	
		//USERBUTTONS------------------------------------------------------
		
		//ANIBUTONS--------------------------------------------------------
		//ANIBUTONS--------------------------------------------------------
		
		//FORMS--------------------------------------------------------
		if (Event.reportObject.object == GENIE_OBJ_FORM)
		{
			if (Event.reportObject.index == FORM_SDFILES)
			{
				//wake_RELAY();
				Serial.println("Form 2!");
				////Check sdcardFiles
				card.initsd();
				uint16_t fileCnt = card.getnrfilenames();
				//Declare filepointer
				card.getWorkDirName();
				//Text index starts at 0
				card.getfilename(filepointer);
				if (card.filenameIsDir)
				{
					//Is a folder
					//genie.WriteStr(1,card.longFilename);
					//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,1);
				}else{
					int count = 12;
					char buffer[count];
					if (String(card.longFilename).length()>count){
						for (int i = 0; i<count ; i++)
						{
							buffer[i]=card.longFilename[i];
						}
						buffer[count]='\0';
						char* buffer2 = strcat(buffer,"...\0");
						genie.WriteStr(1,buffer2);//Printing form
					}else{
						genie.WriteStr(1,card.longFilename);//Printing form
					}
					//Is a file
					//genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
				}
			}
			
			else if (Event.reportObject.index == FORM_PAUSE) //NOT USED!!!!!
			{
				//if (genie.ReadObject(GENIE_OBJ_USERBUTTON,BUTTON_PAUSE)){
				//genieFrame.reportObject.data_lsb
				//genie.GetcharSerial()
				//}
				//genie.
				//int value = genie.GetEventData(&Event);
				//if (value == 1) // Need to preheat
				//{
				//Button Pressed ON
				if (card.sdprinting)
				{
					card.pauseSDPrint();
				}
				//}
				//else
				//{
				//card.startFileprint();
				//}
				//genie.WriteObject(GENIE_OBJ_FORM,FORM_PAUSED_PRINT,1);
				genie.WriteStr(10,card.longFilename);//PausedPrint form
				//Many ways to achieve this, we could check ReadObject to know if resume is UP or NOT
			}
			
			else if (Event.reportObject.index == FORM_STOP)
			{
				//plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS]+10,current_position[E_AXIS], 600, active_extruder);
				card.sdprinting = false;
				card.closefile();
				
				quickStop();
				
				if(SD_FINISHED_STEPPERRELEASE)
				{
					enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
				}
				autotempShutdown();
				setTargetHotend(0,active_extruder);
				setTargetBed(0);
				
				//If we are paused, unpause.
				card.sdispaused=false;
				
				//Rapduch
				genie.WriteObject(GENIE_OBJ_USERBUTTON,BUTTON_PAUSE_RESUME,0);//Set pause Button to default position
				genie.WriteObject(GENIE_OBJ_FORM,5,0);//Exit to main menu
			}
			
			else if (Event.reportObject.index == FORM_START_PRINT)
			{
				//Not sure if useful to update strings here!!!
				//genie.WriteStr(7,card.longFilename);
				//genie.WriteStr(8,"Ready...");
			}
			
			else if (Event.reportObject.index == FORM_PRINTING)
			{
				//Restart the preheat buttons
				genie.WriteObject(GENIE_OBJ_USERIMAGES,BUTTON_PREHEAT_PLA,0);
				
				int count = 12;
				char buffer[count];
				if (String(card.longFilename).length()>count){
					for (int i = 0; i<count ; i++)
					{
						buffer[i]=card.longFilename[i];
					}
					buffer[count]='\0';
					char* buffer2 = strcat(buffer,"...\0");
					genie.WriteStr(STRINGS_PRINTING_GCODE,buffer2);//Printing form
					}else{
					genie.WriteStr(STRINGS_PRINTING_GCODE,card.longFilename);//Printing form
				}
				
				//genie.WriteStr(2,card.longFilename);
				//genie.WriteStr(6,"Printing...");
			}
			
			else if (Event.reportObject.index == FORM_FEEDRATE)
			{
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FEEDRATE,feedmultiply);
			}
			
			else if (Event.reportObject.index == FORM_NOZZLE)
			{			
				//genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[which_extruder]);
			}
			
			else if (Event.reportObject.index == FORM_BED)
			{
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_BED,target_temperature_bed);
			}
			
			else if (Event.reportObject.index == FORM_FAN)
			{
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FAN,fanSpeed);
			}
			
			
			else if (Event.reportObject.index == FORM_MAIN_SCREEN)
			{
				surfing_utilities=false;
				Serial.println("Surfing 0");
			}
			
			else if (Event.reportObject.index == FORM_UTILITIES)
			{
				surfing_utilities=true;
				Serial.println("Surfing 1");
			}	
			
			else if (Event.reportObject.index == FORM_INSERT_FIL_PREHEAT)
			{
				//setTargetHotend0(ABS_PREHEAT_HOTEND_TEMP);
				//setTargetHotend1(ABS_PREHEAT_HOTEND_TEMP);
			}	
			
			else if (Event.reportObject.index == FORM_PRINTING_SETTINGS)
			{
				//Rapduch
				//Edit for final TouchScreen
				char buffer[256];
				int tHotend=target_temperature[0];
				int tHotend1=target_temperature[1];
				int tBed=target_temperature_bed;
				
				sprintf(buffer, "%3d",tHotend);			
				//Serial.println(buffer);
				genie.WriteStr(STRING_PRINT_SET_NOZZ1,buffer);
				
				sprintf(buffer, "%3d",tHotend1);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PRINT_SET_NOZZ2,buffer);
				
				sprintf(buffer, "%3d",tBed);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PRINT_SET_BED,buffer);
				
				sprintf(buffer, "%3d %%",feedmultiply);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PRINT_SET_PERCENT,buffer);
			}
			
			else if (Event.reportObject.index == FORM_PREHEAT_SETTINGS)
			{
				//Rapduch	
				//First send the actual command
				setTargetHotend0(PLA_PREHEAT_HOTEND_TEMP);
				setTargetHotend1(PLA_PREHEAT_HOTEND_TEMP);
				setTargetBed(PLA_PREHEAT_HPB_TEMP);
				
				//Now let's print it on the touchscreen
				char buffer[256];
				int tHotend=target_temperature[0];
				int tHotend1=target_temperature[1];
				int tBed=target_temperature_bed;
				
				//Serial.println("TARGET TEMPS");
				
				sprintf(buffer, "%3d",tHotend);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PREHEAT_SET_NOZZ1,buffer);
				
				sprintf(buffer, "%3d",tHotend1);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PREHEAT_SET_NOZZ2,buffer);
				
				sprintf(buffer, "%3d",tBed);
				//Serial.println(buffer);
				genie.WriteStr(STRING_PREHEAT_SET_BED,buffer);
			}
			
			//else if (Event.reportObject.index == FORM_CALIB_BED_SCREW2)
			//{
				//char buffer[256];
				//sprintf(buffer, " %d / 8",vuitens2); //Printing how to calibrate on screen
				//genie.WriteStr(STRING_BED_SCREW2,buffer);
				//if (sentit2>0){genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,1);} //The direction is inverted in Sigma's bed screws
				//else{genie.WriteObject(GENIE_OBJ_USERIMAGES,USERIMAGE_SCREW2,0);}
					//
					//Serial.print("vuitens2:");
					//Serial.print(vuitens2);
			//}			
		}
	}
}



char* prepareString(char* text, int len){
	Serial.print("Text: ");
	Serial.println(text);
	if (String(text).length()>12){
		char buffer[len+1];
		for (int i = 0; i<len ; i++)
		{
			buffer[i]=card.longFilename[i];
		}
		buffer[len]='\0';
		Serial.print("Buffer temp: ");
		Serial.println(buffer);
		char* buffer2 = strcat(buffer,"...\0");
		Serial.print("Buffer returned: ");
		Serial.println(buffer2);
		return buffer2;
		//buffer2 = strcat(buffer,"...\0");
	}else{
		char* buffer2 = text;
		Serial.print("Buffer returned: ");
		Serial.println(buffer2);
		return buffer2;
	}
		
	
}



#endif /* INCLUDE */

#endif