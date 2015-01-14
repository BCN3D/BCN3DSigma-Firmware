/*
 * LCD_Handler.h
 *A place to hold all interactions between LCD and printer. It is called from Marlin_main.cpp
 * Created: 12/12/2014 12:48:04
 *  Author: jcalduch
 */ 


#ifndef LCD_HANDLER_H_
#define LCD_HANDLER_H_

//Rapduch
#include "genieArduino.h"
#include "Touch_Screen_Definitions.h"
#include "Marlin.h"


void myGenieEventHandler();

//Created by Jordi Calduch for RepRapBCN SIGMA 10/2014
void myGenieEventHandler(void)
{
	genieFrame Event;
	genie.DequeueEvent(&Event);
	//static long waitPeriod = millis();
	int move_mm = 10;
	//If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
	if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
	{
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
			}
			else if (Event.reportObject.index == BUTTON_MOVE_AXIS_minusX)                              // If Winbutton0
			{
				Serial.println("Left");
				modified_position=current_position[X_AXIS]-move_mm;
				if (modified_position < X_MIN_POS)modified_position = X_MIN_POS;
				if (modified_position > X_MAX_POS)modified_position = X_MAX_POS;
				plan_buffer_line(modified_position, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 600, active_extruder);
				current_position[X_AXIS]=modified_position;
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
			else if (Event.reportObject.index == 5)
			{
				int value = genie.GetEventData(&Event);
				if (value == 1) // Need to preheat
				{
					setTargetHotend0(200);
					setTargetBed(50);
					//Serial.println("Heating Baby!");
				}
				else
				{
					setTargetHotend0(0);
					setTargetBed(0);
					//Serial.println("Cooling !!!");
				}
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
		}
		
		//USERBUTTONS------------------------------------------------------
		if (Event.reportObject.object == GENIE_OBJ_USERBUTTON) //Userbuttons to select GCODE from SD
		{
			if (Event.reportObject.index == BUTTON_SD_SELECTED )
			{
				if(card.cardOK)
				{
					genie.WriteObject(GENIE_OBJ_FORM,FORM_START_PRINT,0);
					screen_status="Ready...";//Write the selected SD file to all strings
					genie.WriteStr(8,"Ready...");
					genie.WriteStr(7,card.longFilename);
					//Reset Time LEDs
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,12,0);
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,11,0);
					
					//StartPrint form
					//genie.WriteStr(2,card.longFilename);//Printing form
					//genie.WriteStr(10,card.longFilename);//PausedPrint form
				}
			}
			
			else if (Event.reportObject.index == BUTTON_START_PRINTING )
			{
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
					genie.WriteObject(GENIE_OBJ_USERIMAGES,0,1);
					}else{
					//Is a file
					genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
				}
				genie.WriteStr(1,card.longFilename);
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
			
			else if (Event.reportObject.index == BUTTON_STOP )
			{
				card.sdprinting = false;
				card.closefile();
				
				//plan_buffer_line(current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS]+10,current_position[E_AXIS], 600, active_extruder);
				quickStop();
				
				if(SD_FINISHED_STEPPERRELEASE)
				{
					enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
				}
				autotempShutdown();
				//setTargetHotend0(0);
				//setTargetHotend1(0);
				//setTargetHotend2(0);
				//setTargetBed(0);
				
				//Rapduch
				genie.WriteObject(GENIE_OBJ_FORM,5,0);
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
			
			else if (Event.reportObject.index == BUTTON_NOZZLE_UP )
			{
				int value=5;
				if (target_temperature[active_extruder]<HEATER_0_MAXTEMP)
				{
					target_temperature[active_extruder]+=value;
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[active_extruder]);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_NOZZLE_DOWN )
			{
				int value=5;
				if (target_temperature[active_extruder]>HEATER_0_MINTEMP)
				{
					target_temperature[active_extruder]-=value;
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[active_extruder]);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_BED_UP )
			{
				int value=5;
				if (target_temperature_bed<BED_MAXTEMP)
				{
					target_temperature_bed+=value;
					genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_BED,target_temperature_bed);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_BED_DOWN )
			{
				int value=5;
				if (target_temperature_bed>BED_MINTEMP)
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
					genie.WriteStr(6,buffer);
				}else
				{
					genie.WriteObject(GENIE_OBJ_FORM,FORM_START_PRINT,0);
					//Restore Strings
					genie.WriteStr(7,card.longFilename);
					genie.WriteStr(8,buffer);
				}
			}
			
			else if (Event.reportObject.index == BUTTON_PAUSE_RESUME )
			{
				int value = genie.GetEventData(&Event);
				if (value == 1) // Need to preheat
				{
					card.pauseSDPrint();
					genie.WriteStr(6,"Pausing...");
					Serial.println("PAUSE!");
					}else{
					if(card.sdispaused)
					{
						card.startFileprint();
						genie.WriteStr(6,"Printing...");
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
				int value = genie.GetEventData(&Event);
				if (value == 1) // Need to preheat
				{
					setTargetHotend(220,active_extruder);
					setTargetBed(50);
					Serial.println("Preheating PLA");
				}
				else
				{
					setTargetHotend(0,active_extruder);
					setTargetBed(0);
					Serial.println("Cooling down PLA");
				}
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
					genie.WriteStr(1,card.longFilename);
					genie.WriteObject(GENIE_OBJ_USERIMAGES,0,1);
					}else{
					//Is a file
					genie.WriteStr(1,card.longFilename);
					genie.WriteObject(GENIE_OBJ_USERIMAGES,0,0);
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
				genie.WriteObject(GENIE_OBJ_FORM,FORM_PAUSED_PRINT,1);
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
				//genie.WriteStr(2,card.longFilename);
				//genie.WriteStr(6,"Printing...");
			}
			
			else if (Event.reportObject.index == FORM_FEEDRATE)
			{
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FEEDRATE,feedmultiply);
			}
			
			else if (Event.reportObject.index == FORM_NOZZLE)
			{
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_NOZZLE,target_temperature[active_extruder]);
			}
			
			else if (Event.reportObject.index == FORM_BED)
			{
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_BED,target_temperature_bed);
			}
			
			else if (Event.reportObject.index == FORM_FAN)
			{
				genie.WriteObject(GENIE_OBJ_LED_DIGITS,LEDDIGITS_FAN,fanSpeed);
			}
		}
		

		//If the cmd received is from a Reported Object, which occurs if a Read Object (genie.ReadOject) is requested in the main code, reply processed here.
		//if (Event.reportObject.cmd == GENIE_REPORT_OBJ)
		//{
		//if (Event.reportObject.object == GENIE_OBJ_USER_LED)              // If the Reported Message was from a User LED
		//{
		//if (Event.reportObject.index == 0)                              // If UserLed0
		//{
		//bool UserLed0_val = genie.GetEventData(&Event);               // Receive the event data from the UserLed0
		//UserLed0_val = !UserLed0_val;                                 // Toggle the state of the User LED Variable
		//genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, UserLed0_val);    // Write UserLed0_val value back to to UserLed0
		//}
		//}
		//}

		//This can be expanded as more objects are added that need to be captured

		//Event.reportObject.cmd is used to determine the command of that event, such as an reported event
		//Event.reportObject.object is used to determine the object type, such as a Slider
		//Event.reportObject.index is used to determine the index of the object, such as Slider0
		//genie.GetEventData(&Event) us used to save the data from the Event, into a variable.
	}
}






#endif /* INCLUDE */