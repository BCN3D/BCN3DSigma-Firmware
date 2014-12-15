/*
  Hysteresis.cpp - A class that manages hysteresis by correcting extruder position when direction is changed
 
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

#include "Hysteresis.h"
#include "Configuration.h"
#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"

float menu_hysteresis_X=X_DEFAULT_HYSTERESIS_MM;
float menu_hysteresis_Y=Y_DEFAULT_HYSTERESIS_MM;

//===========================================================================

Hysteresis hysteresis( X_DEFAULT_HYSTERESIS_MM, Y_DEFAULT_HYSTERESIS_MM, 0.0f, 0.0f );
float axis_shift[ NUM_AXIS ] = { 0.0f, 0.0f, 0.0f, 0.0f };

//===========================================================================
Hysteresis::Hysteresis( float x_mm, float y_mm, float z_mm, float e_mm )
{
	m_prev_direction_bits = 0;
	Set( x_mm, y_mm, z_mm, e_mm );
}

//===========================================================================
void Hysteresis::Set( float x_mm, float y_mm, float z_mm, float e_mm )
{
	m_hysteresis_mm[X_AXIS] = x_mm;
	m_hysteresis_mm[Y_AXIS] = y_mm;
	m_hysteresis_mm[Z_AXIS] = z_mm;
	m_hysteresis_mm[E_AXIS] = e_mm;
	m_hysteresis_bits = ((m_hysteresis_mm[X_AXIS]!=0.0f)?(1<<X_AXIS):0)
	| ((m_hysteresis_mm[Y_AXIS]!=0.0f)?(1<<Y_AXIS):0)
	| ((m_hysteresis_mm[Z_AXIS]!=0.0f)?(1<<Z_AXIS):0)
	| ((m_hysteresis_mm[E_AXIS]!=0.0f)?(1<<E_AXIS):0);
	calcSteps();
}

//===========================================================================
void Hysteresis::SetAxis( int axis, float mm )
{
	m_hysteresis_mm[axis] = mm;
	if( mm != 0.0f )
	{
		m_hysteresis_bits |= (1<<axis);
	}
	else
	{
		m_hysteresis_bits &= ~(1<<axis);
	}
	calcSteps();
}

// convert mm to steps
void Hysteresis::calcSteps()
{
	for (int i=0; i<NUM_AXIS; i++)
	{
		m_hysteresis_steps[i] = (long)(m_hysteresis_mm[i]*axis_steps_per_unit[i]);
	}
}


//===========================================================================
void Hysteresis::ReportToSerial()
{
	SERIAL_PROTOCOLPGM("H=X");
	SERIAL_PROTOCOL(m_hysteresis_mm[X_AXIS]);
	SERIAL_PROTOCOLPGM(" Y");
	SERIAL_PROTOCOL(m_hysteresis_mm[Y_AXIS]);
	SERIAL_PROTOCOLPGM(" Z");
	SERIAL_PROTOCOL(m_hysteresis_mm[Z_AXIS]);
	SERIAL_PROTOCOLPGM(" E");
	SERIAL_PROTOCOL(m_hysteresis_mm[E_AXIS]);
	SERIAL_PROTOCOLLN("");
}

//===========================================================================
// direction 0: positive, 1: negative
unsigned char calc_direction_bits(const long* current_position, const long* destination )
{
	unsigned char direction_bits = 0;
	if (destination[X_AXIS] < current_position[X_AXIS]) {
		direction_bits |= (1<<X_AXIS);
	}
	if (destination[Y_AXIS] < current_position[Y_AXIS]) {
		direction_bits |= (1<<Y_AXIS);
	}
	if (destination[Z_AXIS] < current_position[Z_AXIS]) {
		direction_bits |= (1<<Z_AXIS);
	}
	if (destination[E_AXIS] < current_position[E_AXIS]) {
		direction_bits |= (1<<E_AXIS);
	}
	return direction_bits;
}

unsigned char calc_move_bits(const long* current_position, const long* destination )
{
	unsigned char move_bits = 0;
	if (destination[X_AXIS] != current_position[X_AXIS]) {
		move_bits |= (1<<X_AXIS);
	}
	if (destination[Y_AXIS] != current_position[Y_AXIS]) {
		move_bits |= (1<<Y_AXIS);
	}
	if (destination[Z_AXIS] != current_position[Z_AXIS]) {
		move_bits |= (1<<Z_AXIS);
	}
	if (destination[E_AXIS] != current_position[E_AXIS]) {
		move_bits |= (1<<E_AXIS);
	}
	return move_bits;
}
//===========================================================================


//===========================================================================
void Hysteresis::InsertCorrection(const float &x, const float &y, const float &z, const float &e)
{
	//Set the correction from menu. This allows us to change hysteresis setting while printing
	Set(menu_hysteresis_X,menu_hysteresis_Y,0,0);
	//We calculate steps now just to ensure "axis_steps_per_unit" is available
	calcSteps();
	
	//Destination is calculated in steps to compare it with position
	long destination[NUM_AXIS] = {x*axis_steps_per_unit[X_AXIS],y*axis_steps_per_unit[Y_AXIS],z*axis_steps_per_unit[Z_AXIS],e*axis_steps_per_unit[E_AXIS]};	
	unsigned char direction_bits = calc_direction_bits( position, destination );
	//Move bits controls if the position is the same last buffer line was.
	unsigned char move_bits = calc_move_bits(position, destination);
	
	//For serial debugging
	//SERIAL_PROTOCOL("Destination X: ");
	//SERIAL_PROTOCOL(destination[0]); //X_AXIS=0
	//SERIAL_PROTOCOL("    Current position X: ");
	//SERIAL_PROTOCOLLN(position[X_AXIS]);
	//SERIAL_PROTOCOLLN("");
	//
	//SERIAL_PROTOCOL("Destination Y: ");
	//SERIAL_PROTOCOL(destination[1]); //Y_AXIS=1
	//SERIAL_PROTOCOL("    Current positionY: ");
	//SERIAL_PROTOCOLLN(position[Y_AXIS]);
	//SERIAL_PROTOCOLLN("");
	//SERIAL_PROTOCOLLN("");
		
	// If the direction has changed in any of the axis that need hysteresis corrections...
	unsigned char direction_change_bits = (direction_bits ^ m_prev_direction_bits) & move_bits;
	if( (direction_change_bits & m_hysteresis_bits) != 0 )
	{	
		// Calculate the position to move to that will fix the hysteresis
		for(int axis=0;axis<NUM_AXIS;++axis)
		{
			// If this axis changed direction...
			if( direction_change_bits & (1<<axis) )
			{
				long fix = (((direction_bits&(1<<axis))!=0)?(-m_hysteresis_steps[axis]):(m_hysteresis_steps[axis]));
				
				//For serial debugging
				//SERIAL_PROTOCOL("Axis :  ");
				//SERIAL_PROTOCOLLN(axis);
				//
				//SERIAL_PROTOCOL("Fix :  ");
				//SERIAL_PROTOCOLLN((long int)fix);				
								
				//if (fix<0)
				//{					
					//SERIAL_PROTOCOLLN("------------Correction Up------------");					
				//}
				//else if (fix>0)
				//{				
					//SERIAL_PROTOCOLLN("------------Correction Down----------");					
				//}
				
				//Add the hysteresis: move the current position in the opposite direction so that the next travel move is longer
				position[axis] -= fix;				
				axis_shift[axis] += fix;
			}
		}
	}
	
	//SERIAL_PROTOCOLLN("");
	//SERIAL_PROTOCOLLN("");
	//SERIAL_PROTOCOLLN("");
	//Save direction for next comparison
	m_prev_direction_bits = (direction_bits & move_bits) | (m_prev_direction_bits & ~move_bits);
}


//Extra functions for LCD menu
void update_hysteresis_circles()
{	
	menu_hysteresis_X=0.05;
	menu_hysteresis_Y=0.15;
	//max_xy_jerk=1;
}

void update_hysteresis_off()
{
	menu_hysteresis_X=0;
	menu_hysteresis_Y=0;
	//max_xy_jerk=5;
}
