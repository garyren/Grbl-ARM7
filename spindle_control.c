/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include "types.h"
#include "settings.h"
#include "stepper.h"
#include "spindle_control.h"
#include "planner.h"

static uint8_t current_direction;


void spindle_init(void)
{
  current_direction = 0;
  IODIR |= bfSPINDLE;
  //SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT);  
  spindle_stop();
}


void spindle_run(int8_t direction) //, uint16_t rpm) 
{
	IOSET |= bfSPINDLE;		
}

void spindle_stop(void)
{
	IOCLR |= bfSPINDLE;		
}
