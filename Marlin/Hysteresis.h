/*
  Hysteresis.h - A class that manages hysteresis by correcting extruder position when direction is changed.
 
  Copyright (c) 2012 Neil James Martin. Updated by fsantini (2013).
  Version made by Jordi Calduch Casas ( dryrain ) for RepRapBCN. Includes changes to control hysteresis it by menu LCD.
 
 Grbl is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 Grbl is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

//If defined - enables hyst menu config and save
#ifndef HYSTERESIS_H
#define HYSTERESIS_H


#include "Configuration.h"

//Edit these constants if needed to set a default hysteresis. It is normally done trough LCD menu.
#define X_DEFAULT_HYSTERESIS_MM 0
#define Y_DEFAULT_HYSTERESIS_MM 0.15

//===========================================================================

class Hysteresis
{
public:
	Hysteresis( float x_mm, float y_mm, float z_mm, float e_mm );
  
	void Set( float x_mm, float y_mm, float z_mm, float e_mm );
	void SetAxis( int axis, float mm );
	void ReportToSerial();
	void InsertCorrection(const float &x, const float &y, const float &z, const float &e);
  

private:
	void calcSteps();
	
	float         m_hysteresis_mm[NUM_AXIS];
	long          m_hysteresis_steps[NUM_AXIS];
	unsigned char m_prev_direction_bits;
	unsigned char m_hysteresis_bits;
};

//===========================================================================

 void update_hysteresis_circles();
 void update_hysteresis_off();

extern Hysteresis hysteresis;
extern float max_xy_jerk;
extern float menu_hysteresis_X;
extern float menu_hysteresis_Y;
extern long position[4]; // defined in planner.cpp

#endif